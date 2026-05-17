#include "modules/perception/traffic_light/traffic_light_component.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <functional>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/common_msgs/v2x_msgs/v2x_traffic_light.pb.h"
#include "modules/perception/traffic_light/ports/default_provider_ports.h"
#include "modules/perception/traffic_light/proto/traffic_light_component.pb.h"

namespace apollo {
namespace perception {
namespace traffic_light {

namespace {

std::string ToLowerAscii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return value;
}

int InferImageChannels(const apollo::drivers::Image& image) {
  const std::string encoding = ToLowerAscii(image.encoding());
  if (encoding == "rgb8" || encoding == "bgr8" || encoding == "type_8uc3") {
    return 3;
  }
  if (encoding == "rgba8" || encoding == "bgra8" || encoding == "type_8uc4") {
    return 4;
  }
  if (encoding == "mono8" || encoding == "type_8uc1") {
    return 1;
  }
  if (encoding == "yuv422" || encoding == "type_8uc2") {
    return 2;
  }
  if (image.width() > 0 && image.step() > 0) {
    const int inferred = static_cast<int>(image.step() / image.width());
    if (inferred >= 1 && inferred <= 4) {
      return inferred;
    }
  }
  if (image.width() > 0 && image.height() > 0 && !image.data().empty()) {
    const size_t pixel_count =
        static_cast<size_t>(image.width()) * static_cast<size_t>(image.height());
    if (pixel_count > 0) {
      const size_t inferred = image.data().size() / pixel_count;
      if (inferred >= 1 && inferred <= 4) {
        return static_cast<int>(inferred);
      }
    }
  }
  return 0;
}

std::shared_ptr<std::vector<uint8_t>> CopyImageStorage(
    const apollo::drivers::Image& image, int channels) {
  if (channels <= 0 || image.width() == 0 || image.height() == 0) {
    return nullptr;
  }

  const size_t compact_row_bytes =
      static_cast<size_t>(image.width()) * static_cast<size_t>(channels);
  const size_t source_row_bytes =
      image.step() > 0 ? static_cast<size_t>(image.step()) : compact_row_bytes;
  const size_t expected_bytes =
      source_row_bytes * static_cast<size_t>(image.height());
  if (image.data().size() < expected_bytes) {
    AERROR << "Invalid image buffer, expected at least " << expected_bytes
           << " bytes but got " << image.data().size();
    return nullptr;
  }

  if (source_row_bytes == compact_row_bytes) {
    return std::make_shared<std::vector<uint8_t>>(image.data().begin(),
                                                  image.data().end());
  }

  auto storage = std::make_shared<std::vector<uint8_t>>(
      compact_row_bytes * static_cast<size_t>(image.height()));
  const auto* source =
      reinterpret_cast<const uint8_t*>(image.data().data());
  for (uint32_t row = 0; row < image.height(); ++row) {
    std::memcpy(storage->data() + row * compact_row_bytes,
                source + row * source_row_bytes, compact_row_bytes);
  }
  return storage;
}

apollo::perception::TrafficLight::Color ConvertToProtoColor(LightColor color) {
  switch (color) {
    case LightColor::RED:
      return apollo::perception::TrafficLight::RED;
    case LightColor::YELLOW:
      return apollo::perception::TrafficLight::YELLOW;
    case LightColor::GREEN:
      return apollo::perception::TrafficLight::GREEN;
    case LightColor::BLACK:
      return apollo::perception::TrafficLight::BLACK;
    default:
      return apollo::perception::TrafficLight::UNKNOWN;
  }
}

apollo::perception::TrafficLightDetection::CameraID ConvertCameraId(
    const std::string& camera_name) {
  static const std::unordered_map<
      std::string, apollo::perception::TrafficLightDetection::CameraID>
      kCameraIdMap = {
          {"front_24mm",
           apollo::perception::TrafficLightDetection::CAMERA_FRONT_LONG},
          {"front_12mm",
           apollo::perception::TrafficLightDetection::CAMERA_FRONT_NARROW},
          {"front_6mm",
           apollo::perception::TrafficLightDetection::CAMERA_FRONT_SHORT},
          {"front_fisheye",
           apollo::perception::TrafficLightDetection::CAMERA_FRONT_WIDE},
      };
  const auto it = kCameraIdMap.find(camera_name);
  if (it == kCameraIdMap.end()) {
    return apollo::perception::TrafficLightDetection::CAMERA_FRONT_SHORT;
  }
  return it->second;
}

void FillBox(const Rect2f& roi, apollo::perception::TrafficLightBox* box) {
  if (box == nullptr) {
    return;
  }
  box->set_x(static_cast<int32_t>(roi.x));
  box->set_y(static_cast<int32_t>(roi.y));
  box->set_width(static_cast<int32_t>(roi.width));
  box->set_height(static_cast<int32_t>(roi.height));
}

LaneIntent ConvertV2XType(apollo::v2x::SingleTrafficLight::Type type) {
  switch (type) {
    case apollo::v2x::SingleTrafficLight::LEFT:
      return LaneIntent::LEFT;
    case apollo::v2x::SingleTrafficLight::RIGHT:
      return LaneIntent::RIGHT;
    case apollo::v2x::SingleTrafficLight::U_TURN:
      return LaneIntent::U_TURN;
    case apollo::v2x::SingleTrafficLight::STRAIGHT:
    default:
      return LaneIntent::STRAIGHT;
  }
}

DetectorBackendType ConvertDetectorBackend(
    apollo::perception::trafficlight::TrafficLightDetectorBackend backend) {
  switch (backend) {
    case apollo::perception::trafficlight::TL_DETECTOR_YOLO:
    default:
      return DetectorBackendType::YOLO;
  }
}

LightColor ConvertV2XColor(apollo::v2x::SingleTrafficLight::Color color) {
  switch (color) {
    case apollo::v2x::SingleTrafficLight::RED:
      return LightColor::RED;
    case apollo::v2x::SingleTrafficLight::YELLOW:
      return LightColor::YELLOW;
    case apollo::v2x::SingleTrafficLight::GREEN:
    case apollo::v2x::SingleTrafficLight::FLASH_GREEN:
      return LightColor::GREEN;
    case apollo::v2x::SingleTrafficLight::BLACK:
      return LightColor::BLACK;
    default:
      return LightColor::UNKNOWN;
  }
}

LightColor ConvertClassColor(
    apollo::perception::trafficlight::TrafficLightClassColor color) {
  switch (color) {
    case apollo::perception::trafficlight::TL_CLASS_RED:
      return LightColor::RED;
    case apollo::perception::trafficlight::TL_CLASS_YELLOW:
      return LightColor::YELLOW;
    case apollo::perception::trafficlight::TL_CLASS_GREEN:
      return LightColor::GREEN;
    case apollo::perception::trafficlight::TL_CLASS_BLACK:
      return LightColor::BLACK;
    case apollo::perception::trafficlight::TL_CLASS_UNKNOWN:
    default:
      return LightColor::UNKNOWN;
  }
}

ComponentOptions ToOptions(
    const apollo::perception::trafficlight::TrafficLightComponentConfig& proto) {
  ComponentOptions options;
  options.tf2_frame_id = proto.tl_tf2_frame_id();
  options.tf2_child_frame_id = proto.tl_tf2_child_frame_id();
  options.tf2_timeout_second = proto.tf2_timeout_second();
  options.max_process_image_fps = proto.max_process_image_fps();
  options.query_tf_interval_seconds = proto.query_tf_interval_seconds();
  options.valid_hdmap_interval_seconds = proto.valid_hdmap_interval_seconds();
  options.image_sys_ts_diff_threshold = proto.image_sys_ts_diff_threshold();
  options.frame_cache_tolerance_sec = proto.frame_cache_tolerance_sec();
  options.output_channel_name = proto.traffic_light_output_channel_name();
  options.debug_output_channel_name = proto.debug_output_channel_name();
  options.debug_image_channel_name = proto.debug_image_channel_name();
  options.v2x_channel_name = proto.v2x_trafficlights_input_channel_name();
  options.v2x_sync_interval_seconds = proto.v2x_sync_interval_seconds();
  options.max_v2x_msg_buff_size = proto.max_v2x_msg_buff_size();
  options.enable_debug_recording = proto.enable_debug_recording();
  options.enable_debug_image_stream = proto.enable_debug_image_stream();
  options.enable_perf_logging = proto.enable_perf_logging();
  options.enable_heuristic_stage = proto.enable_heuristic_stage();

  for (int i = 0; i < proto.cameras_size(); ++i) {
    const auto& camera = proto.cameras(i);
    CameraSourceOptions camera_option;
    camera_option.camera_name = camera.camera_name();
    camera_option.channel_name = camera.channel_name();
    camera_option.image_width = camera.image_width();
    camera_option.image_height = camera.image_height();
    camera_option.is_primary = camera.is_primary();
    options.cameras.push_back(camera_option);
  }
  if (!options.cameras.empty() &&
      std::none_of(options.cameras.begin(), options.cameras.end(),
                   [](const CameraSourceOptions& option) {
                     return option.is_primary;
                   })) {
    options.cameras.front().is_primary = true;
  }

  if (proto.has_prompter()) {
    const auto& cfg = proto.prompter();
    options.prompter.history_expand_ratio = cfg.history_expand_ratio();
    options.prompter.history_weight = cfg.history_weight();
    options.prompter.memory_expand_ratio = cfg.memory_expand_ratio();
    options.prompter.memory_weight = cfg.memory_weight();
    options.prompter.map_expand_ratio = cfg.map_expand_ratio();
    options.prompter.map_weight_floor = cfg.map_weight_floor();
    options.prompter.cold_start_distance_to_intersection_m =
        cfg.cold_start_distance_to_intersection_m();
    options.prompter.full_frame_weight = cfg.full_frame_weight();
    options.prompter.always_add_full_frame_fallback =
        cfg.always_add_full_frame_fallback();
  }
  if (proto.has_detector()) {
    const auto& cfg = proto.detector();
    options.detector.backend = ConvertDetectorBackend(cfg.backend());
    options.detector.min_prompt_weight = cfg.min_prompt_weight();
    options.detector.min_roi_area = cfg.min_roi_area();
    options.detector.min_objectness = cfg.min_objectness();
    options.detector.min_semantic_confidence = cfg.min_semantic_confidence();
    options.detector.prefer_raw_yolo_candidates =
        cfg.prefer_raw_yolo_candidates();
  }
  if (proto.has_neural_detector()) {
    const auto& cfg = proto.neural_detector();
    options.neural_detector.model_root_dir = cfg.model_root_dir();
    options.neural_detector.onnx_file = cfg.onnx_file();
    options.neural_detector.enable_fp16 = cfg.enable_fp16();
    options.neural_detector.input_name = cfg.input_name();
    options.neural_detector.output_name = cfg.output_name();
    options.neural_detector.resize_image_height = cfg.resize_image_height();
    options.neural_detector.resize_image_width = cfg.resize_image_width();
    options.neural_detector.conf_threshold = cfg.conf_threshold();
    options.neural_detector.iou_nms_threshold = cfg.iou_nms_threshold();
    options.neural_detector.pad_value = cfg.pad_value();
    options.neural_detector.scale = cfg.scale();
    options.neural_detector.is_bgr = cfg.is_bgr();
    options.neural_detector.num_classes = cfg.num_classes();
    options.neural_detector.num_predictions = cfg.num_predictions();
    options.neural_detector.green_class_id = cfg.green_class_id();
    options.neural_detector.red_class_id = cfg.red_class_id();
    options.neural_detector.yellow_class_id = cfg.yellow_class_id();
    options.neural_detector.min_box_area = cfg.min_box_area();
    options.neural_detector.gpu_id = cfg.gpu_id();
    for (int class_index = 0; class_index < cfg.class_labels_size();
         ++class_index) {
      const auto& label = cfg.class_labels(class_index);
      NeuralDetectorOptions::ClassLabel class_label;
      class_label.class_id = label.class_id();
      class_label.class_name = label.class_name();
      class_label.color = ConvertClassColor(label.color());
      class_label.accepted = label.accepted();
      options.neural_detector.class_labels.push_back(class_label);
    }
  }
  if (proto.has_binder()) {
    const auto& cfg = proto.binder();
    options.binder.min_bind_score = cfg.min_bind_score();
    options.binder.require_intent_match = cfg.require_intent_match();
    options.binder.min_signal_overlap = cfg.min_signal_overlap();
    options.binder.max_center_distance_ratio = cfg.max_center_distance_ratio();
    options.binder.min_topology_confidence = cfg.min_topology_confidence();
  }
  if (proto.has_tracker()) {
    const auto& cfg = proto.tracker();
    options.tracker.max_lost_frames = cfg.max_lost_frames();
    options.tracker.min_iou_match = cfg.min_iou_match();
    options.tracker.belief_decay = cfg.belief_decay();
    options.tracker.measurement_gain = cfg.measurement_gain();
    options.tracker.min_confirmed_visible_count =
        cfg.min_confirmed_visible_count();
  }
  if (proto.has_heuristic()) {
    const auto& cfg = proto.heuristic();
    options.heuristic.max_distance_to_intersection_m =
        cfg.max_distance_to_intersection_m();
    options.heuristic.min_starting_agents = cfg.min_starting_agents();
    options.heuristic.green_probability = cfg.green_probability();
    options.heuristic.red_hold_probability = cfg.red_hold_probability();
  }
  if (proto.has_fusion()) {
    const auto& cfg = proto.fusion();
    options.fusion.weak_vision_threshold = cfg.weak_vision_threshold();
    options.fusion.strong_v2x_threshold = cfg.strong_v2x_threshold();
    options.fusion.heuristic_trigger_threshold =
        cfg.heuristic_trigger_threshold();
    options.fusion.heuristic_accept_threshold =
        cfg.heuristic_accept_threshold();
    options.fusion.v2x_sync_window_sec = cfg.v2x_sync_window_sec();
    options.fusion.prefer_ego_intent = cfg.prefer_ego_intent();
    options.fusion.min_conflict_override_margin =
        cfg.min_conflict_override_margin();
  }
  return options;
}

void PopulateDebugInfo(const PipelineContext& context,
                       apollo::perception::TrafficLightDebug* debug) {
  if (debug == nullptr) {
    return;
  }

  debug->set_signal_num(static_cast<int32_t>(context.final_lights.size()));
  debug->set_valid_pos(context.status.tf_available ? 1 : 0);
  debug->set_project_error(context.status.hdmap_available ? 0 : 1);
  if (context.primary_decision.stopline_distance_m >= 0.0) {
    debug->set_distance_to_stop_line(context.primary_decision.stopline_distance_m);
  }
  debug->set_ts_diff_sys(context.primary_decision.freshness_sec);
  if (context.runtime_state != nullptr &&
      context.runtime_state->last_signals_ts_sec > 0.0) {
    debug->set_ts_diff_pos(static_cast<double>(context.timestamp) * 1e-9 -
                           context.runtime_state->last_signals_ts_sec);
  }

  for (const auto& prompt : context.prompts) {
    FillBox(prompt.roi_box, debug->add_debug_roi());
  }
  for (const auto& signal : context.map_signals) {
    FillBox(signal.projection_roi, debug->add_projected_roi());
  }
  for (const auto& visual_light : context.visual_lights) {
    auto* box = debug->add_box();
    FillBox(visual_light.bbox, box);
    box->set_color(ConvertToProtoColor(visual_light.color));
    box->set_selected(false);
    box->set_camera_name(visual_light.camera_name);
  }
  for (const auto& tracked_light : context.tracked_lights) {
    auto* rectified = debug->add_rectified_roi();
    FillBox(tracked_light.current_state.visual_light.bbox, rectified);
    rectified->set_color(ConvertToProtoColor(tracked_light.stabilized_color));
    rectified->set_selected(true);
  }
}

class CyberTrafficLightResultWriter final : public IResultWriterPort {
 public:
  CyberTrafficLightResultWriter(
      std::shared_ptr<
          apollo::cyber::Writer<apollo::perception::TrafficLightDetection>>
          writer,
      bool include_debug)
      : writer_(std::move(writer)), include_debug_(include_debug) {}

  bool Write(const PipelineContext& context,
             const TrafficLightResult& result) override {
    if (writer_ == nullptr) {
      return false;
    }

    auto message =
        std::make_shared<apollo::perception::TrafficLightDetection>();
    auto* header = message->mutable_header();
    header->set_timestamp_sec(static_cast<double>(context.timestamp) * 1e-9);
    header->set_camera_timestamp(context.timestamp);

    message->set_camera_id(ConvertCameraId(context.primary_camera_name));
    message->set_contain_lights(
        !context.final_lights.empty() &&
        context.primary_decision.existence_confidence > 0.10f);
    const std::vector<TrafficLightResult>* lights =
        context.final_lights.empty() ? nullptr : &context.final_lights;
    if (lights == nullptr && result.color != LightColor::UNKNOWN) {
      auto* light = message->add_traffic_light();
      light->set_id(result.signal_id);
      light->set_confidence(result.confidence);
      light->set_blink(result.blink);
      light->set_color(ConvertToProtoColor(result.color));
    } else if (lights != nullptr) {
      for (const auto& item : *lights) {
        auto* light = message->add_traffic_light();
        light->set_id(item.signal_id);
        light->set_confidence(item.confidence);
        light->set_blink(item.blink);
        light->set_color(ConvertToProtoColor(item.color));
      }
    }

    if (include_debug_) {
      PopulateDebugInfo(context, message->mutable_traffic_light_debug());
    }
    writer_->Write(message);
    return true;
  }

 private:
  std::shared_ptr<
      apollo::cyber::Writer<apollo::perception::TrafficLightDetection>>
      writer_;
  bool include_debug_ = false;
};

class FanoutResultWriter final : public IResultWriterPort {
 public:
  explicit FanoutResultWriter(
      std::vector<std::shared_ptr<IResultWriterPort>> delegates)
      : delegates_(std::move(delegates)) {}

  bool Write(const PipelineContext& context,
             const TrafficLightResult& result) override {
    bool all_ok = true;
    for (const auto& delegate : delegates_) {
      if (delegate == nullptr) {
        continue;
      }
      all_ok = delegate->Write(context, result) && all_ok;
    }
    return all_ok;
  }

 private:
  std::vector<std::shared_ptr<IResultWriterPort>> delegates_;
};

}  // namespace

bool TrafficLightComponent::Init() {
  if (!LoadOptions()) {
    return false;
  }
  pipeline_ = std::make_unique<PerceptionPipeline>();
  RegisterDefaultStages();
  if (!pipeline_->InitAll()) {
    return false;
  }
  if (!InitDefaultPorts()) {
    return false;
  }
  return InitReaders();
}

bool TrafficLightComponent::LoadOptions() {
  apollo::perception::trafficlight::TrafficLightComponentConfig config;
  if (!GetProtoConfig(&config)) {
    AERROR << "Failed to load traffic_light config proto";
    return false;
  }
  options_ = ToOptions(config);
  if (options_.cameras.empty()) {
    AERROR << "At least one camera must be configured";
    return false;
  }
  return true;
}

bool TrafficLightComponent::InitDefaultPorts() {
  if (data_provider_ == nullptr) {
    auto provider = std::make_shared<InMemoryDataProviderPort>();
    provider->SetFrameStalenessToleranceSec(options_.frame_cache_tolerance_sec);
    std::vector<std::string> camera_order;
    for (const auto& camera : options_.cameras) {
      camera_order.push_back(camera.camera_name);
    }
    provider->SetCameraOrder(camera_order);
    data_provider_ = provider;
  }
  frame_input_port_ = std::dynamic_pointer_cast<IFrameInputPort>(data_provider_);
  if (frame_input_port_ == nullptr) {
    AERROR << "Configured data provider does not implement frame input";
    return false;
  }

  if (pose_provider_ == nullptr) {
    auto pose_provider = std::make_shared<StaticPoseProviderPort>();
    VehicleState ego_state;
    ego_state.pose.valid = false;
    pose_provider->SetEgoState(ego_state);
    pose_provider_ = pose_provider;
  }

  if (map_provider_ == nullptr) {
    auto map_provider = std::make_shared<CachedMapProviderPort>();
    map_provider->SetValidCacheWindowSec(options_.valid_hdmap_interval_seconds);
    map_provider_ = map_provider;
  }

  if (detector_provider_ == nullptr) {
    detector_provider_ = std::make_shared<NeuralDetectorProviderPort>(
        options_.neural_detector);
  }

  if (v2x_provider_ == nullptr) {
    auto provider = std::make_shared<BufferedV2XProviderPort>();
    provider->SetMaxBufferSize(options_.max_v2x_msg_buff_size);
    v2x_provider_ = provider;
  }
  v2x_input_port_ = std::dynamic_pointer_cast<IV2XInputPort>(v2x_provider_);
  if (v2x_input_port_ == nullptr) {
    AERROR << "Configured V2X provider does not implement input port";
    return false;
  }

  if (result_writer_ == nullptr) {
    auto main_writer =
        std::make_shared<CyberTrafficLightResultWriter>(
            node_->CreateWriter<apollo::perception::TrafficLightDetection>(
                options_.output_channel_name),
            false);
    if (options_.enable_debug_recording) {
      auto debug_writer =
          std::make_shared<CyberTrafficLightResultWriter>(
              node_->CreateWriter<apollo::perception::TrafficLightDetection>(
                  options_.debug_output_channel_name),
              true);
      result_writer_ = std::make_shared<FanoutResultWriter>(
          std::vector<std::shared_ptr<IResultWriterPort>>{main_writer,
                                                          debug_writer});
    } else {
      result_writer_ = main_writer;
    }
  }

  if (options_.enable_debug_image_stream && debug_image_writer_ == nullptr) {
    debug_image_writer_ =
        node_->CreateWriter<apollo::drivers::Image>(
            options_.debug_image_channel_name);
  }
  return true;
}

bool TrafficLightComponent::InitReaders() {
  for (const auto& camera : options_.cameras) {
    if (camera.camera_name.empty() || camera.channel_name.empty()) {
      AERROR << "Invalid camera config";
      return false;
    }
    node_->CreateReader<apollo::drivers::Image>(
        camera.channel_name,
        std::bind(&TrafficLightComponent::OnReceiveImage, this,
                  std::placeholders::_1, camera.camera_name));
  }
  if (!options_.v2x_channel_name.empty()) {
    node_->CreateReader<apollo::v2x::IntersectionTrafficLightData>(
        options_.v2x_channel_name,
        std::bind(&TrafficLightComponent::OnReceiveV2XMsg, this,
                  std::placeholders::_1));
  }
  return true;
}

bool TrafficLightComponent::ProcessOnceFromPorts() {
  if (pipeline_ == nullptr || data_provider_ == nullptr) {
    return false;
  }

  PipelineContext context;
  context.runtime_state = &runtime_state_;
  if (!data_provider_->PopulateFrameData(&context)) {
    return false;
  }
  if (pose_provider_ != nullptr) {
    pose_provider_->PopulatePose(&context);
  }
  if (map_provider_ != nullptr) {
    map_provider_->PopulateSignals(&context);
  }
  if (detector_provider_ != nullptr) {
    detector_provider_->PopulateDetections(&context);
  }
  if (v2x_provider_ != nullptr) {
    v2x_provider_->PopulateV2X(&context);
  }
  for (const auto& frame : context.camera_frames) {
    runtime_state_.last_camera_timestamps_sec[frame.camera_name] =
        static_cast<double>(frame.timestamp_ns) * 1e-9;
  }
  return ProcessFrame(&context);
}

bool TrafficLightComponent::PublishDecision(const PipelineContext& context) {
  if (result_writer_ == nullptr) {
    return true;
  }
  return result_writer_->Write(context, context.primary_decision);
}

void TrafficLightComponent::OnReceiveImage(
    const std::shared_ptr<apollo::drivers::Image>& image,
    const std::string& camera_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_input_port_ == nullptr || image == nullptr) {
    return;
  }

  const double now_sec = apollo::cyber::Clock::NowInSeconds();
  const double& last_camera_process_ts =
      last_process_wall_ts_by_camera_sec_[camera_name];
  if (options_.max_process_image_fps > 0.0 && last_camera_process_ts > 0.0 &&
      now_sec - last_camera_process_ts <
          1.0 / options_.max_process_image_fps) {
    ++runtime_state_.dropped_frame_count;
    return;
  }

  const int channels = InferImageChannels(*image);
  auto storage = CopyImageStorage(*image, channels);
  if (storage == nullptr) {
    return;
  }
  Image frame_image;
  frame_image.storage = storage;
  frame_image.data = storage->empty() ? nullptr : storage->data();
  frame_image.rows = static_cast<int>(image->height());
  frame_image.cols = static_cast<int>(image->width());
  frame_image.channels = channels;
  frame_image.encoding = ToLowerAscii(image->encoding());

  CameraFrameState camera_frame;
  camera_frame.camera_name = camera_name;
  camera_frame.timestamp_ns =
      static_cast<uint64_t>(image->measurement_time() * 1e9);
  camera_frame.image = frame_image;

  if (!frame_input_port_->PushCameraFrame(++frame_counter_, camera_frame)) {
    return;
  }
  if (options_.enable_debug_image_stream && debug_image_writer_ != nullptr) {
    debug_image_writer_->Write(image);
  }

  last_process_wall_ts_sec_ = now_sec;
  last_process_wall_ts_by_camera_sec_[camera_name] = now_sec;
  ProcessOnceFromPorts();
}

void TrafficLightComponent::OnReceiveV2XMsg(
    const std::shared_ptr<apollo::v2x::IntersectionTrafficLightData>& v2x_msg) {
  if (v2x_input_port_ == nullptr || v2x_msg == nullptr) {
    return;
  }

  for (int road_index = 0; road_index < v2x_msg->road_traffic_light_size();
       ++road_index) {
    const auto& road_light = v2x_msg->road_traffic_light(road_index);
    for (int i = 0; i < road_light.single_traffic_light_size(); ++i) {
      const auto& single = road_light.single_traffic_light(i);
      V2XLightEvidence evidence;
      evidence.signal_id = single.id();
      evidence.color = ConvertV2XColor(single.color());
      evidence.blink =
          single.color() == apollo::v2x::SingleTrafficLight::FLASH_GREEN;
      evidence.confidence = static_cast<float>(v2x_msg->confidence());
      evidence.timestamp_sec = v2x_msg->header().timestamp_sec();
      if (single.traffic_light_type_size() > 0) {
        evidence.movement = ConvertV2XType(single.traffic_light_type(0));
        uint32_t movement_mask = kMovementMaskNone;
        for (int type_index = 0; type_index < single.traffic_light_type_size();
             ++type_index) {
          movement_mask |= MovementMaskFromLaneIntent(
              ConvertV2XType(single.traffic_light_type(type_index)));
        }
        evidence.movement_mask = movement_mask;
      }
      v2x_input_port_->PushV2XEvidence(evidence);
    }
  }
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
