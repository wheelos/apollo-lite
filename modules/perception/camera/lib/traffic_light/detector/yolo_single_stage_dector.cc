/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/camera/lib/traffic_light/detector/yolo_single_stage_dector.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/inference/utils/resize.h"

namespace apollo {
namespace perception {
namespace camera {

namespace {

std::map<std::string, std::vector<int>> BuildShapeMapOrDefault(
    const YoloSingleStageDectorTrafficLightDetectionConfig& cfg) {
  std::map<std::string, std::vector<int>> shapes;
  for (const auto& blob : cfg.blobs()) {
    std::vector<int> shape(blob.shape().begin(), blob.shape().end());
    shapes.emplace(blob.name(), std::move(shape));
  }
  if (!shapes.empty()) {
    return shapes;
  }

  shapes.emplace(cfg.input_name(),
                 std::vector<int>{1, 3, cfg.resize_image_height(),
                                  cfg.resize_image_width()});
  shapes.emplace(cfg.output_name(), std::vector<int>{1, 4 + cfg.num_classes(),
                                                     cfg.num_predictions()});
  return shapes;
}

float IoU(const base::RectI& lhs, const base::RectI& rhs) {
  const auto inter = lhs & rhs;
  const float inter_area = static_cast<float>(inter.Area());
  const float union_area =
      static_cast<float>(lhs.Area() + rhs.Area() - inter.Area());
  if (inter_area <= 0.0f || union_area <= 0.0f) {
    return 0.0f;
  }
  return inter_area / union_area;
}

}  // namespace

TrafficLightYoloSingleStageDector::~TrafficLightYoloSingleStageDector() {
  if (owns_stream_ && stream_ != 0) {
    cudaStreamDestroy(stream_);
    stream_ = 0;
  }
}

bool TrafficLightYoloSingleStageDector::Init(
    const TrafficLightDetectionConfig& config) {
  config_ = config;
  yolo_single_stage_dector_config_ = config.yolo_single_stage_dector_config();

  gpu_id_ = config.gpu_id();
  conf_threshold_ = yolo_single_stage_dector_config_.conf_threshold();
  iou_threshold_ = yolo_single_stage_dector_config_.iou_nms_threshold();
  scale_ = yolo_single_stage_dector_config_.scale();
  distance_weight_ =
      yolo_single_stage_dector_config_.projection_distance_weight();
  min_box_area_ = yolo_single_stage_dector_config_.min_box_area();
  resize_height_ = yolo_single_stage_dector_config_.resize_image_height();
  resize_width_ = yolo_single_stage_dector_config_.resize_image_width();
  num_classes_ = yolo_single_stage_dector_config_.num_classes();
  num_predictions_ = yolo_single_stage_dector_config_.num_predictions();
  pad_value_ = yolo_single_stage_dector_config_.pad_value();
  is_bgr_ = yolo_single_stage_dector_config_.is_bgr();
  green_class_id_ = yolo_single_stage_dector_config_.green_class_id();
  red_class_id_ = yolo_single_stage_dector_config_.red_class_id();
  yellow_class_id_ = yolo_single_stage_dector_config_.yellow_class_id();

  if (cudaSetDevice(gpu_id_) != cudaSuccess) {
    AERROR << "Failed to set device to: " << gpu_id_;
    return false;
  }

  if (stream_ == 0) {
    cudaError_t err = cudaStreamCreate(&stream_);
    if (err != cudaSuccess) {
      AERROR << "Failed to create cuda stream: " << cudaGetErrorString(err);
      return false;
    }
    owns_stream_ = true;
  }

  image_options_.do_crop = false;
  image_options_.target_color = is_bgr_ ? base::Color::BGR : base::Color::RGB;

  input_names_ = {yolo_single_stage_dector_config_.input_name()};
  output_names_ = {yolo_single_stage_dector_config_.output_name()};

  const std::string model_root_dir =
      yolo_single_stage_dector_config_.model_root_dir();
  const std::string onnx_file = cyber::common::GetAbsolutePath(
      model_root_dir, yolo_single_stage_dector_config_.onnx_file());

  infer_ = std::make_shared<inference::MultiBatchInference>();
  infer_->set_gpu_id(gpu_id_);
  infer_->set_max_batch_size(1);
  infer_->set_enable_fp16(yolo_single_stage_dector_config_.enable_fp16());
  infer_->set_model_info(onnx_file, input_names_, output_names_);
  infer_->SetStream(stream_);

  if (!infer_->Init(BuildShapeMapOrDefault(yolo_single_stage_dector_config_))) {
    AERROR << "Failed to init MultiBatchInference for YOLO single-stage TL "
              "detection";
    return false;
  }

  input_blob_ = infer_->get_blob(input_names_[0]);
  output_blob_ = infer_->get_blob(output_names_[0]);
  association_optimizer_.costs()->Reserve(32, 256);
  return input_blob_ != nullptr && output_blob_ != nullptr;
}

bool TrafficLightYoloSingleStageDector::Detect(CameraFrame* frame) {
  return DetectTL(frame);
}

bool TrafficLightYoloSingleStageDector::BuildLetterBoxImage(
    const base::Image8U& src, base::Image8U* dst,
    YoloSingleStageDectorLetterBoxParam* param) const {
  if (dst == nullptr || param == nullptr) {
    return false;
  }
  if (src.channels() != dst->channels()) {
    AERROR << "Letterbox image channel mismatch";
    return false;
  }
  if (src.rows() <= 0 || src.cols() <= 0 || dst->rows() <= 0 ||
      dst->cols() <= 0) {
    return false;
  }

  param->src_width = src.cols();
  param->src_height = src.rows();
  param->scale = std::min(
      static_cast<float>(dst->cols()) / static_cast<float>(src.cols()),
      static_cast<float>(dst->rows()) / static_cast<float>(src.rows()));
  param->resized_width =
      std::max(1, static_cast<int>(std::round(src.cols() * param->scale)));
  param->resized_height =
      std::max(1, static_cast<int>(std::round(src.rows() * param->scale)));
  const int dw = dst->cols() - param->resized_width;
  const int dh = dst->rows() - param->resized_height;
  param->left_pad = dw / 2;
  param->right_pad = dw - param->left_pad;
  param->top_pad = dh / 2;
  param->bottom_pad = dh - param->top_pad;

  std::memset(dst->mutable_cpu_data(), pad_value_,
              dst->rows() * dst->width_step());

  const int channels = src.channels();
  for (int y = 0; y < param->resized_height; ++y) {
    const float src_y = (static_cast<float>(y) + 0.5f) / param->scale - 0.5f;
    const int src_y0 = static_cast<int>(std::floor(src_y));
    const int src_y1 = src_y0 + 1;
    const int y0 = std::max(0, src_y0);
    const int y1 = std::min(src.rows() - 1, src_y1);
    const float ly = src_y - static_cast<float>(src_y0);
    const float hy = 1.0f - ly;
    uint8_t* dst_row =
        dst->mutable_cpu_ptr(y + param->top_pad) + param->left_pad * channels;
    for (int x = 0; x < param->resized_width; ++x) {
      const float src_x = (static_cast<float>(x) + 0.5f) / param->scale - 0.5f;
      const int src_x0 = static_cast<int>(std::floor(src_x));
      const int src_x1 = src_x0 + 1;
      const int x0 = std::max(0, src_x0);
      const int x1 = std::min(src.cols() - 1, src_x1);
      const float lx = src_x - static_cast<float>(src_x0);
      const float hx = 1.0f - lx;
      const uint8_t* src00 = src.cpu_ptr(y0) + x0 * channels;
      const uint8_t* src01 = src.cpu_ptr(y0) + x1 * channels;
      const uint8_t* src10 = src.cpu_ptr(y1) + x0 * channels;
      const uint8_t* src11 = src.cpu_ptr(y1) + x1 * channels;
      uint8_t* out = dst_row + x * channels;
      for (int channel = 0; channel < channels; ++channel) {
        const float value = hy * (hx * src00[channel] + lx * src01[channel]) +
                            ly * (hx * src10[channel] + lx * src11[channel]);
        out[channel] = static_cast<uint8_t>(std::round(value));
      }
    }
  }

  return true;
}

bool TrafficLightYoloSingleStageDector::DecodeDetections(
    const YoloSingleStageDectorLetterBoxParam& param,
    std::vector<base::TrafficLightPtr>* detections) const {
  if (detections == nullptr || output_blob_ == nullptr) {
    return false;
  }

  const int channels = output_blob_->shape(1);
  const int prediction_num = output_blob_->shape(2);
  if (channels < 5 || prediction_num <= 0) {
    AERROR << "Unexpected output shape for YOLO single-stage detector";
    return false;
  }

  const float* output = output_blob_->cpu_data();
  for (int pred_idx = 0; pred_idx < prediction_num; ++pred_idx) {
    const float cx = output[pred_idx];
    const float cy = output[prediction_num + pred_idx];
    const float width = output[2 * prediction_num + pred_idx];
    const float height = output[3 * prediction_num + pred_idx];

    float best_score = -std::numeric_limits<float>::infinity();
    int best_class = -1;
    for (int class_idx = 4; class_idx < channels; ++class_idx) {
      const float score = output[class_idx * prediction_num + pred_idx];
      if (score > best_score) {
        best_score = score;
        best_class = class_idx - 4;
      }
    }

    if (best_class < 0 || best_score < conf_threshold_) {
      continue;
    }

    float x1 = cx - width * 0.5f;
    float y1 = cy - height * 0.5f;
    float x2 = cx + width * 0.5f;
    float y2 = cy + height * 0.5f;

    x1 = (x1 - static_cast<float>(param.left_pad)) / param.scale;
    y1 = (y1 - static_cast<float>(param.top_pad)) / param.scale;
    x2 = (x2 - static_cast<float>(param.left_pad)) / param.scale;
    y2 = (y2 - static_cast<float>(param.top_pad)) / param.scale;

    base::RectI rect(static_cast<int>(std::floor(x1)),
                     static_cast<int>(std::floor(y1)),
                     static_cast<int>(std::ceil(x2 - x1)),
                     static_cast<int>(std::ceil(y2 - y1)));
    RefineBox(rect, param.src_width, param.src_height, &rect);
    if (rect.Area() <= 0 || static_cast<float>(rect.Area()) < min_box_area_) {
      continue;
    }

    base::TrafficLightPtr detection(new base::TrafficLight);
    detection->region.detection_roi = rect;
    detection->region.detect_score = best_score;
    detection->region.is_detected = true;
    detection->region.detect_class_id = GuessShapeClass(rect);
    detection->status.color = ClassIdToColor(best_class);
    detection->status.confidence = best_score;
    detections->push_back(detection);
  }

  return true;
}

void TrafficLightYoloSingleStageDector::ApplyNMS(
    std::vector<base::TrafficLightPtr>* detections) const {
  if (detections == nullptr || detections->empty()) {
    return;
  }

  std::vector<std::pair<float, int>> score_index(detections->size());
  for (size_t idx = 0; idx < detections->size(); ++idx) {
    score_index[idx] = {detections->at(idx)->region.detect_score,
                        static_cast<int>(idx)};
  }
  std::stable_sort(
      score_index.begin(), score_index.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.first > rhs.first; });

  std::vector<int> kept_indices;
  for (const auto& score_pair : score_index) {
    const int idx = score_pair.second;
    bool keep = true;
    for (int kept_idx : kept_indices) {
      if (IoU(detections->at(idx)->region.detection_roi,
              detections->at(kept_idx)->region.detection_roi) >=
          iou_threshold_) {
        keep = false;
        break;
      }
    }
    if (keep) {
      kept_indices.push_back(idx);
    }
  }

  std::vector<base::TrafficLightPtr> kept;
  kept.reserve(kept_indices.size());
  for (int idx : kept_indices) {
    kept.push_back(detections->at(idx));
  }
  detections->swap(kept);
}

float TrafficLightYoloSingleStageDector::ComputeAssociationScore(
    const base::TrafficLightPtr& hdmap_light,
    const base::TrafficLightPtr& detection, int image_width,
    int image_height) const {
  const float detection_score = detection->region.detect_score;
  if (hdmap_light == nullptr) {
    return detection_score;
  }

  const auto& projection = hdmap_light->region.projection_roi;
  const bool projection_valid =
      !hdmap_light->region.outside_image && projection.Area() > 0 &&
      !OutOfValidRegion(projection, image_width, image_height);
  if (!projection_valid) {
    return detection_score;
  }

  const auto center_hd = projection.Center();
  const auto center_det = detection->region.detection_roi.Center();
  const float sigma = std::max(
      1.0f, 0.25f * static_cast<float>(std::max(image_width, image_height)));
  const float dx = static_cast<float>(center_hd.x - center_det.x);
  const float dy = static_cast<float>(center_hd.y - center_det.y);
  const float distance_score =
      std::exp(-0.5f * (dx * dx + dy * dy) / (sigma * sigma));
  const float distance_weight =
      std::max(0.0f, std::min(distance_weight_, 1.0f));
  const float detection_weight = 1.0f - distance_weight;
  return detection_weight * detection_score + distance_weight * distance_score;
}

void TrafficLightYoloSingleStageDector::AssociateDetections(
    std::vector<base::TrafficLightPtr>* hdmap_lights,
    const std::vector<base::TrafficLightPtr>& detections, int image_width,
    int image_height) {
  if (hdmap_lights == nullptr) {
    return;
  }

  for (auto& light : *hdmap_lights) {
    light->region.is_selected = false;
    light->region.is_detected = false;
    light->status.color = base::TLColor::TL_UNKNOWN_COLOR;
    light->status.confidence = 0.0;
  }

  if (hdmap_lights->empty() || detections.empty()) {
    return;
  }

  association_optimizer_.costs()->Resize(hdmap_lights->size(),
                                         detections.size());
  std::vector<std::vector<float>> scores(
      hdmap_lights->size(), std::vector<float>(detections.size(), 0.0f));
  for (size_t row = 0; row < hdmap_lights->size(); ++row) {
    for (size_t col = 0; col < detections.size(); ++col) {
      scores[row][col] = ComputeAssociationScore(
          hdmap_lights->at(row), detections[col], image_width, image_height);
      (*association_optimizer_.costs())(row, col) = scores[row][col];
    }
  }

  std::vector<std::pair<size_t, size_t>> assignments;
  association_optimizer_.Maximize(&assignments);

  for (const auto& assignment : assignments) {
    if (assignment.first >= hdmap_lights->size() ||
        assignment.second >= detections.size()) {
      continue;
    }
    if (scores[assignment.first][assignment.second] <= 0.0f) {
      continue;
    }

    auto& hdmap_light = hdmap_lights->at(assignment.first);
    const auto& detection = detections[assignment.second];
    hdmap_light->region.detection_roi = detection->region.detection_roi;
    hdmap_light->region.detect_class_id = detection->region.detect_class_id;
    hdmap_light->region.detect_score = detection->region.detect_score;
    hdmap_light->region.is_detected = true;
    hdmap_light->region.is_selected = true;
    hdmap_light->status.color = detection->status.color;
    hdmap_light->status.confidence = detection->status.confidence;
  }
}

base::TLColor TrafficLightYoloSingleStageDector::ClassIdToColor(
    int class_id) const {
  if (class_id == green_class_id_) {
    return base::TLColor::TL_GREEN;
  }
  if (class_id == red_class_id_) {
    return base::TLColor::TL_RED;
  }
  if (class_id == yellow_class_id_) {
    return base::TLColor::TL_YELLOW;
  }
  return base::TLColor::TL_UNKNOWN_COLOR;
}

base::TLDetectionClass TrafficLightYoloSingleStageDector::GuessShapeClass(
    const base::RectI& roi) const {
  if (roi.height > roi.width * 3 / 2) {
    return base::TLDetectionClass::TL_VERTICAL_CLASS;
  }
  if (roi.width > roi.height * 3 / 2) {
    return base::TLDetectionClass::TL_HORIZONTAL_CLASS;
  }
  return base::TLDetectionClass::TL_QUADRATE_CLASS;
}

bool TrafficLightYoloSingleStageDector::DetectTL(CameraFrame* frame) {
  if (frame == nullptr) {
    return false;
  }
  if (frame->traffic_lights.empty()) {
    AINFO << "no hdmap lights to detect";
    return true;
  }
  if (frame->data_provider == nullptr) {
    AERROR << "data provider is null";
    return false;
  }
  if (cudaSetDevice(gpu_id_) != cudaSuccess) {
    AERROR << "Failed To Set Device To: " << gpu_id_;
    return false;
  }

  const auto& data_provider = frame->data_provider;
  const int image_width = data_provider->src_width();
  const int image_height = data_provider->src_height();
  const base::RectI full_image_roi(0, 0, image_width, image_height);

  for (auto& light : frame->traffic_lights) {
    light->region.detection_roi = light->region.projection_roi;
    light->region.crop_roi = full_image_roi;
    light->region.debug_roi.clear();
    light->region.debug_roi_detect_scores.clear();
    light->region.debug_roi.push_back(full_image_roi);
    light->region.debug_roi_detect_scores.push_back(0.0f);
    light->region.is_detected = false;
    light->status.color = base::TLColor::TL_UNKNOWN_COLOR;
    light->status.confidence = 0.0;
  }

  base::Image8U source_image;
  if (!data_provider->GetImage(image_options_, &source_image)) {
    AERROR << "Failed to get source image";
    return false;
  }
  base::Image8U letterbox_image(resize_height_, resize_width_,
                                image_options_.target_color);
  YoloSingleStageDectorLetterBoxParam letterbox_param;
  if (!BuildLetterBoxImage(source_image, &letterbox_image, &letterbox_param)) {
    AERROR << "Failed to build letterbox image";
    return false;
  }

  input_blob_->Reshape({1, 3, resize_height_, resize_width_});
  output_blob_->Reshape({1, 4 + num_classes_, num_predictions_});

#if USE_GPU == 0
  std::memset(input_blob_->mutable_cpu_data(), 0,
              input_blob_->count() * sizeof(float));
#else
  cudaMemsetAsync(input_blob_->mutable_gpu_data(), 0,
                  input_blob_->count() * sizeof(float), stream_);
  cudaStreamSynchronize(stream_);
#endif

  inference::ResizeGPU(letterbox_image, input_blob_, letterbox_image.cols(), 0,
                       0.0f, 0.0f, 0.0f, false, scale_);

  cudaDeviceSynchronize();
  infer_->Infer();
  cudaDeviceSynchronize();

  std::vector<base::TrafficLightPtr> detections;
  if (!DecodeDetections(letterbox_param, &detections)) {
    return false;
  }
  ApplyNMS(&detections);
  for (const auto& detection : detections) {
    frame->traffic_lights[0]->region.debug_roi.push_back(
        detection->region.detection_roi);
    frame->traffic_lights[0]->region.debug_roi_detect_scores.push_back(
        detection->region.detect_score);
  }
  AssociateDetections(&frame->traffic_lights, detections, image_width,
                      image_height);
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
