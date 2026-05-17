#pragma once

#include <algorithm>
#include <cmath>
#include <cctype>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/inference/onnx_multi_batch/onnx_multi_batch_infer.h"
#include "modules/perception/traffic_light/ports/provider_ports.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class InMemoryDataProviderPort : public IDataProviderPort,
                                 public IFrameInputPort {
 public:
  void SetFrameStalenessToleranceSec(double tolerance_sec) {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_staleness_tolerance_sec_ = std::max(0.0, tolerance_sec);
  }

  void SetCameraOrder(const std::vector<std::string>& camera_order) {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_order_ = camera_order;
  }

  bool PushCameraFrame(uint64_t frame_id,
                       const CameraFrameState& frame) override {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_id_ = frame_id;
    timestamp_ns_ = frame.timestamp_ns;
    primary_camera_name_ = frame.camera_name;
    latest_frames_[frame.camera_name] = frame;
    return true;
  }

  bool PopulateFrameData(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    context->frame_id = frame_id_;
    context->timestamp = timestamp_ns_;
    context->primary_camera_name = primary_camera_name_;
    context->camera_frames.clear();

    if (primary_camera_name_.empty()) {
      if (!camera_order_.empty()) {
        context->primary_camera_name = camera_order_.front();
      } else if (!latest_frames_.empty()) {
        context->primary_camera_name = latest_frames_.begin()->first;
      }
    }

    const double latest_ts_sec = static_cast<double>(timestamp_ns_) * 1e-9;
    auto append_if_fresh = [&](const std::string& camera_name) {
      const auto it = latest_frames_.find(camera_name);
      if (it == latest_frames_.end()) {
        return;
      }
      const double frame_ts_sec =
          static_cast<double>(it->second.timestamp_ns) * 1e-9;
      if (latest_ts_sec > 0.0 &&
          latest_ts_sec - frame_ts_sec > frame_staleness_tolerance_sec_) {
        return;
      }
      context->camera_frames.push_back(it->second);
    };

    if (!context->primary_camera_name.empty()) {
      append_if_fresh(context->primary_camera_name);
    }
    for (const auto& camera_name : camera_order_) {
      if (camera_name == context->primary_camera_name) {
        continue;
      }
      append_if_fresh(camera_name);
    }
    for (const auto& item : latest_frames_) {
      if (item.first == context->primary_camera_name ||
          std::find(camera_order_.begin(), camera_order_.end(), item.first) !=
              camera_order_.end()) {
        continue;
      }
      append_if_fresh(item.first);
    }

    if (!context->camera_frames.empty()) {
      context->image_tele = context->camera_frames.front().image;
    } else {
      context->image_tele = Image();
    }
    context->image_wide = Image();
    context->status.image_healthy = !context->camera_frames.empty();
    return context->status.image_healthy;
  }

 private:
  std::mutex mutex_;
  uint64_t frame_id_ = 0;
  uint64_t timestamp_ns_ = 0;
  double frame_staleness_tolerance_sec_ = 0.2;
  std::string primary_camera_name_;
  std::vector<std::string> camera_order_;
  std::map<std::string, CameraFrameState> latest_frames_;
};

class StaticPoseProviderPort : public IPoseProviderPort {
 public:
  void SetEgoState(const VehicleState& ego_state) { ego_state_ = ego_state; }

  void SetCameraPoses(
      const std::vector<std::pair<std::string, Pose3d>>& camera_poses) {
    camera_poses_.clear();
    for (const auto& item : camera_poses) {
      camera_poses_[item.first] = item.second;
    }
  }

  bool PopulatePose(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }
    context->ego_state = ego_state_;
    for (auto& camera_frame : context->camera_frames) {
      const auto it = camera_poses_.find(camera_frame.camera_name);
      if (it != camera_poses_.end()) {
        camera_frame.camera_pose = it->second;
      }
    }
    context->status.tf_available = context->ego_state.pose.valid;
    if (!context->status.tf_available) {
      context->AppendDegradeReason("pose unavailable");
    }
    return context->status.tf_available;
  }

 private:
  VehicleState ego_state_;
  std::map<std::string, Pose3d> camera_poses_;
};

class CachedMapProviderPort : public IMapProviderPort {
 public:
  void SetSignals(std::vector<SignalCandidate> signals) {
    signals_ = std::move(signals);
  }

  void SetValidCacheWindowSec(double valid_cache_window_sec) {
    valid_cache_window_sec_ = std::max(0.0, valid_cache_window_sec);
  }

  bool PopulateSignals(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }
    if (!signals_.empty()) {
      context->map_signals = signals_;
      context->status.hdmap_available = true;
      if (context->runtime_state != nullptr) {
        context->runtime_state->cached_signals = signals_;
        context->runtime_state->last_signals_ts_sec =
            static_cast<double>(context->timestamp) * 1e-9;
      }
      return true;
    }

    if (context->runtime_state != nullptr &&
        !context->runtime_state->cached_signals.empty()) {
      const double frame_ts_sec =
          static_cast<double>(context->timestamp) * 1e-9;
      const double dt =
          frame_ts_sec - context->runtime_state->last_signals_ts_sec;
      if (dt >= 0.0 && dt <= valid_cache_window_sec_) {
        context->map_signals = context->runtime_state->cached_signals;
        context->status.hdmap_available = true;
        context->AppendDegradeReason("hdmap fallback cache");
        return true;
      }
    }

    context->status.hdmap_available = false;
    context->AppendDegradeReason("hdmap unavailable");
    return false;
  }

 private:
  double valid_cache_window_sec_ = 1.5;
  std::vector<SignalCandidate> signals_;
};

class StaticDetectorProviderPort : public IDetectorProviderPort {
 public:
  void SetDetections(std::vector<YoloLightCandidate> detections) {
    detections_ = std::move(detections);
  }

  bool PopulateDetections(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }
    context->raw_yolo_lights = detections_;
    context->status.neural_detector_ran = true;
    return !context->raw_yolo_lights.empty();
  }

 private:
  std::vector<YoloLightCandidate> detections_;
};

class NeuralDetectorProviderPort : public IDetectorProviderPort {
 public:
  explicit NeuralDetectorProviderPort(NeuralDetectorOptions options)
      : options_(std::move(options)) {}

  bool PopulateDetections(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    context->raw_yolo_lights.clear();
    context->status.neural_detector_ran = false;

    const CameraFrameState* frame = SelectPrimaryFrame(*context);
    if (frame == nullptr || frame->image.data == nullptr || frame->image.cols <= 0 ||
        frame->image.rows <= 0) {
      context->AppendDegradeReason("neural detector missing image");
      return false;
    }

    if (!EnsureDetectorInitialized()) {
      context->AppendDegradeReason("neural detector init failed");
      return false;
    }

    LetterboxTransform transform;
    if (!PopulateInputBlob(frame->image, &transform)) {
      context->AppendDegradeReason("neural detector preprocessing failed");
      return false;
    }

    inference_->Infer();
    context->raw_yolo_lights = DecodeOutputs(*frame, transform);
    context->status.neural_detector_ran = true;
    return true;
  }

 private:
  struct LetterboxTransform {
    float resize_scale = 1.0f;
    float pad_x = 0.0f;
    float pad_y = 0.0f;
  };

  struct DecodedBox {
    Rect2f bbox;
    float score = 0.0f;
    int class_id = -1;
  };

  const CameraFrameState* SelectPrimaryFrame(const PipelineContext& context) const {
    for (const auto& frame : context.camera_frames) {
      if (frame.camera_name == context.primary_camera_name) {
        return &frame;
      }
    }
    if (!context.camera_frames.empty()) {
      return &context.camera_frames.front();
    }
    return nullptr;
  }

  bool EnsureDetectorInitialized() {
    if (inference_ != nullptr) {
      return true;
    }

    const std::string model_path =
        cyber::common::GetAbsolutePath(options_.model_root_dir, options_.onnx_file);
    if (!cyber::common::PathExists(model_path)) {
      AERROR << "Traffic-light ONNX model not found: " << model_path;
      return false;
    }

    inference_ = std::make_unique<apollo::perception::inference::MultiBatchInference>();
    inference_->set_gpu_id(options_.gpu_id);
    inference_->set_max_batch_size(1);
    inference_->set_enable_fp16(options_.enable_fp16);
    inference_->set_model_info(model_path, {options_.input_name},
                               {options_.output_name});

    const std::map<std::string, std::vector<int>> shapes = {
        {options_.input_name,
         {1, 3, options_.resize_image_height, options_.resize_image_width}},
        {options_.output_name,
         {1, 4 + options_.num_classes, options_.num_predictions}},
    };
    if (!inference_->Init(shapes)) {
      AERROR << "Failed to initialize traffic-light ONNX inference";
      inference_.reset();
      return false;
    }

    input_blob_ = inference_->get_blob(options_.input_name);
    output_blob_ = inference_->get_blob(options_.output_name);
    if (input_blob_ == nullptr || output_blob_ == nullptr) {
      AERROR << "Traffic-light detector blobs are unavailable after init";
      inference_.reset();
      input_blob_.reset();
      output_blob_.reset();
      return false;
    }
    return true;
  }

  bool PopulateInputBlob(const Image& image, LetterboxTransform* transform) {
    cv::Mat model_image;
    if (!PrepareModelImage(image, &model_image)) {
      return false;
    }

    const int target_width = options_.resize_image_width;
    const int target_height = options_.resize_image_height;
    const float scale = std::min(
        static_cast<float>(target_width) / static_cast<float>(model_image.cols),
        static_cast<float>(target_height) / static_cast<float>(model_image.rows));
    const int resized_width =
        std::max(1, static_cast<int>(std::round(model_image.cols * scale)));
    const int resized_height =
        std::max(1, static_cast<int>(std::round(model_image.rows * scale)));

    cv::Mat resized;
    cv::resize(model_image, resized, cv::Size(resized_width, resized_height), 0.0,
               0.0, cv::INTER_LINEAR);

    cv::Mat letterbox(target_height, target_width, CV_8UC3,
                      cv::Scalar(options_.pad_value, options_.pad_value,
                                 options_.pad_value));
    const int pad_x = (target_width - resized_width) / 2;
    const int pad_y = (target_height - resized_height) / 2;
    resized.copyTo(
        letterbox(cv::Rect(pad_x, pad_y, resized_width, resized_height)));

    if (transform != nullptr) {
      transform->resize_scale = scale;
      transform->pad_x = static_cast<float>(pad_x);
      transform->pad_y = static_cast<float>(pad_y);
    }

    float* input_data = input_blob_->mutable_cpu_data();
    const int image_area = target_width * target_height;
    for (int y = 0; y < target_height; ++y) {
      const auto* row_ptr = letterbox.ptr<cv::Vec3b>(y);
      for (int x = 0; x < target_width; ++x) {
        const cv::Vec3b& pixel = row_ptr[x];
        for (int c = 0; c < 3; ++c) {
          input_data[c * image_area + y * target_width + x] =
              static_cast<float>(pixel[c]) * options_.scale;
        }
      }
    }
    return true;
  }

  bool PrepareModelImage(const Image& image, cv::Mat* model_image) const {
    if (model_image == nullptr || image.data == nullptr || image.cols <= 0 ||
        image.rows <= 0) {
      return false;
    }

    const std::string encoding = ToLowerAscii(image.encoding);
    const int mat_type = image.channels == 1 ? CV_8UC1
                         : image.channels == 2 ? CV_8UC2
                         : image.channels == 3 ? CV_8UC3
                         : image.channels == 4 ? CV_8UC4
                                               : -1;
    if (mat_type < 0) {
      AERROR << "Unsupported traffic-light image channels: " << image.channels;
      return false;
    }

    const cv::Mat raw(image.rows, image.cols, mat_type,
                      const_cast<uint8_t*>(image.data));
    const bool expect_bgr = options_.is_bgr;
    if (image.channels == 3 &&
        (encoding == "rgb8" || encoding == "bgr8" || encoding == "type_8uc3" ||
         encoding.empty())) {
      if ((encoding == "rgb8" && !expect_bgr) ||
          ((encoding == "bgr8" || encoding == "type_8uc3" || encoding.empty()) &&
           expect_bgr)) {
        *model_image = raw;
        return true;
      }
      cv::cvtColor(raw, *model_image,
                   expect_bgr ? cv::COLOR_RGB2BGR : cv::COLOR_BGR2RGB);
      return true;
    }
    if (image.channels == 4 &&
        (encoding == "rgba8" || encoding == "bgra8" || encoding == "type_8uc4")) {
      cv::cvtColor(raw, *model_image,
                   encoding == "rgba8"
                       ? (expect_bgr ? cv::COLOR_RGBA2BGR : cv::COLOR_RGBA2RGB)
                       : (expect_bgr ? cv::COLOR_BGRA2BGR
                                     : cv::COLOR_BGRA2RGB));
      return true;
    }
    if (image.channels == 1 &&
        (encoding == "mono8" || encoding == "type_8uc1")) {
      cv::cvtColor(raw, *model_image,
                   expect_bgr ? cv::COLOR_GRAY2BGR : cv::COLOR_GRAY2RGB);
      return true;
    }
    if (image.channels == 2 && encoding == "yuv422") {
      cv::cvtColor(raw, *model_image,
                   expect_bgr ? cv::COLOR_YUV2BGR_UYVY
                              : cv::COLOR_YUV2RGB_UYVY);
      return true;
    }

    AERROR << "Unsupported traffic-light image encoding: " << image.encoding
           << " with channels=" << image.channels;
    return false;
  }

  std::vector<YoloLightCandidate> DecodeOutputs(
      const CameraFrameState& frame, const LetterboxTransform& transform) const {
    std::vector<DecodedBox> decoded_boxes;
    if (output_blob_ == nullptr || output_blob_->num_axes() < 3) {
      return {};
    }

    const int output_dim = output_blob_->shape(1);
    const int num_predictions = output_blob_->shape(2);
    if (output_dim < 4 + options_.num_classes || num_predictions <= 0) {
      AERROR << "Unexpected traffic-light detector output shape";
      return {};
    }

    const float* output = output_blob_->cpu_data();
    decoded_boxes.reserve(num_predictions);
    for (int prediction_index = 0; prediction_index < num_predictions;
         ++prediction_index) {
      int best_class_id = -1;
      float best_score = 0.0f;
      for (int class_offset = 0; class_offset < options_.num_classes;
           ++class_offset) {
        const float score =
            output[(4 + class_offset) * num_predictions + prediction_index];
        if (score > best_score) {
          best_score = score;
          best_class_id = class_offset;
        }
      }
      if (best_class_id < 0 || best_score < options_.conf_threshold) {
        continue;
      }
      if (!IsAcceptedClassId(best_class_id)) {
        continue;
      }

      const float center_x = output[prediction_index];
      const float center_y = output[num_predictions + prediction_index];
      const float width = output[2 * num_predictions + prediction_index];
      const float height = output[3 * num_predictions + prediction_index];
      if (width <= 0.0f || height <= 0.0f || transform.resize_scale <= 0.0f) {
        continue;
      }

      const float x_min =
          (center_x - 0.5f * width - transform.pad_x) / transform.resize_scale;
      const float y_min =
          (center_y - 0.5f * height - transform.pad_y) / transform.resize_scale;
      const float x_max =
          (center_x + 0.5f * width - transform.pad_x) / transform.resize_scale;
      const float y_max =
          (center_y + 0.5f * height - transform.pad_y) / transform.resize_scale;

      Rect2f bbox = ClampBox(
          Rect2f{x_min, y_min, x_max - x_min, y_max - y_min}, frame.image.cols,
          frame.image.rows);
      if (!bbox.IsValid() || bbox.Area() < options_.min_box_area) {
        continue;
      }

      decoded_boxes.push_back({bbox, best_score, best_class_id});
    }

    ApplyNms(&decoded_boxes);

    std::vector<YoloLightCandidate> candidates;
    candidates.reserve(decoded_boxes.size());
    for (const auto& box : decoded_boxes) {
      YoloLightCandidate candidate;
      candidate.bbox = box.bbox;
      candidate.objectness = box.score;
      candidate.semantic_confidence = box.score;
      candidate.class_id = box.class_id;
      candidate.class_name = ClassNameForClassId(box.class_id);
      candidate.color = ColorForClassId(box.class_id);
      candidate.shape = LightShape::CIRCLE;
      candidate.camera_name = frame.camera_name;
      candidates.push_back(candidate);
    }
    return candidates;
  }

  void ApplyNms(std::vector<DecodedBox>* boxes) const {
    if (boxes == nullptr || boxes->empty()) {
      return;
    }
    std::sort(boxes->begin(), boxes->end(),
              [](const DecodedBox& lhs, const DecodedBox& rhs) {
                return lhs.score > rhs.score;
              });

    std::vector<DecodedBox> kept_boxes;
    kept_boxes.reserve(boxes->size());
    for (const auto& candidate : *boxes) {
      bool suppressed = false;
      for (const auto& kept : kept_boxes) {
        if (candidate.class_id == kept.class_id &&
            ComputeIou(candidate.bbox, kept.bbox) >= options_.iou_nms_threshold) {
          suppressed = true;
          break;
        }
      }
      if (!suppressed) {
        kept_boxes.push_back(candidate);
      }
    }
    boxes->swap(kept_boxes);
  }

  static float ComputeIou(const Rect2f& lhs, const Rect2f& rhs) {
    const float x1 = std::max(lhs.x, rhs.x);
    const float y1 = std::max(lhs.y, rhs.y);
    const float x2 = std::min(lhs.x + lhs.width, rhs.x + rhs.width);
    const float y2 = std::min(lhs.y + lhs.height, rhs.y + rhs.height);
    const float intersection_width = std::max(0.0f, x2 - x1);
    const float intersection_height = std::max(0.0f, y2 - y1);
    const float intersection_area = intersection_width * intersection_height;
    const float union_area = lhs.Area() + rhs.Area() - intersection_area;
    if (union_area <= 0.0f) {
      return 0.0f;
    }
    return intersection_area / union_area;
  }

  static Rect2f ClampBox(const Rect2f& box, int image_width, int image_height) {
    const float x1 = std::max(0.0f, std::min(box.x, static_cast<float>(image_width)));
    const float y1 =
        std::max(0.0f, std::min(box.y, static_cast<float>(image_height)));
    const float x2 = std::max(
        x1, std::min(box.x + box.width, static_cast<float>(image_width)));
    const float y2 = std::max(
        y1, std::min(box.y + box.height, static_cast<float>(image_height)));
    return Rect2f{x1, y1, x2 - x1, y2 - y1};
  }

  static std::string ToLowerAscii(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) {
                     return static_cast<char>(std::tolower(ch));
                   });
    return value;
  }

  LightColor ColorForClassId(int class_id) const {
    const NeuralDetectorOptions::ClassLabel* label = FindClassLabel(class_id);
    if (label != nullptr) {
      return label->color;
    }
    if (class_id == options_.green_class_id) {
      return LightColor::GREEN;
    }
    if (class_id == options_.red_class_id) {
      return LightColor::RED;
    }
    if (class_id == options_.yellow_class_id) {
      return LightColor::YELLOW;
    }
    return LightColor::UNKNOWN;
  }

  std::string ClassNameForClassId(int class_id) const {
    const NeuralDetectorOptions::ClassLabel* label = FindClassLabel(class_id);
    if (label != nullptr && !label->class_name.empty()) {
      return label->class_name;
    }
    if (class_id == options_.green_class_id) {
      return "green";
    }
    if (class_id == options_.red_class_id) {
      return "red";
    }
    if (class_id == options_.yellow_class_id) {
      return "yellow";
    }
    return "unknown";
  }

  bool IsAcceptedClassId(int class_id) const {
    if (options_.class_labels.empty()) {
      return true;
    }
    const NeuralDetectorOptions::ClassLabel* label = FindClassLabel(class_id);
    return label != nullptr && label->accepted;
  }

  const NeuralDetectorOptions::ClassLabel* FindClassLabel(int class_id) const {
    for (const auto& label : options_.class_labels) {
      if (label.class_id == class_id) {
        return &label;
      }
    }
    return nullptr;
  }

  NeuralDetectorOptions options_;
  std::mutex mutex_;
  std::unique_ptr<apollo::perception::inference::MultiBatchInference> inference_;
  std::shared_ptr<apollo::perception::base::Blob<float>> input_blob_;
  std::shared_ptr<apollo::perception::base::Blob<float>> output_blob_;
};

class BufferedV2XProviderPort : public IV2XProviderPort, public IV2XInputPort {
 public:
  void SetMaxBufferSize(size_t max_buffer_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    max_buffer_size_ = max_buffer_size;
    TrimLocked();
  }

  void PushV2XEvidence(const V2XLightEvidence& evidence) override {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.push_back(evidence);
    TrimLocked();
  }

  bool PopulateV2X(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    context->v2x_lights.assign(buffer_.begin(), buffer_.end());
    if (context->runtime_state != nullptr) {
      context->runtime_state->v2x_buffer.assign(buffer_.begin(), buffer_.end());
      context->runtime_state->TrimV2XBuffer(max_buffer_size_);
    }
    context->status.v2x_available = !context->v2x_lights.empty();
    if (!context->status.v2x_available) {
      context->AppendDegradeReason("v2x unavailable");
    }
    return context->status.v2x_available;
  }

 private:
  void TrimLocked() {
    while (buffer_.size() > max_buffer_size_) {
      buffer_.pop_front();
    }
  }

  std::mutex mutex_;
  size_t max_buffer_size_ = 64;
  std::deque<V2XLightEvidence> buffer_;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
