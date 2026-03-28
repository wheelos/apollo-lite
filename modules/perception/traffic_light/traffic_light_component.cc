#include "modules/perception/traffic_light/traffic_light_component.h"

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/common_msgs/v2x_msgs/v2x_traffic_light.pb.h"

#include "cyber/common/log.h"
#include "modules/perception/traffic_light/ports/default_provider_ports.h"

namespace apollo {
namespace perception {
namespace traffic_light {

namespace {

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
  auto iter = kCameraIdMap.find(camera_name);
  if (iter == kCameraIdMap.end()) {
    return apollo::perception::TrafficLightDetection::CAMERA_FRONT_SHORT;
  }
  return iter->second;
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

void PopulateDebugInfo(const PipelineContext& context,
                       apollo::perception::TrafficLightDebug* debug) {
  if (debug == nullptr) {
    return;
  }

  debug->set_signal_num(static_cast<int32_t>(context.tracked_lights.size()));
  debug->set_valid_pos(context.status.tf_available ? 1 : 0);
  debug->set_project_error(context.status.hdmap_available ? 0 : 1);

  if (context.runtime_state != nullptr &&
      context.runtime_state->last_processed_ts_sec > 0.0) {
    const double frame_ts_sec = static_cast<double>(context.timestamp) * 1e-9;
    debug->set_ts_diff_sys(frame_ts_sec -
                           context.runtime_state->last_processed_ts_sec);
  }

  for (const auto& prompt : context.prompts) {
    FillBox(prompt.roi_box, debug->add_debug_roi());
  }

  for (const auto& map_signal : context.map_signals) {
    FillBox(map_signal.projection_roi, debug->add_projected_roi());
  }

  for (const auto& visual : context.visual_lights) {
    auto* box = debug->add_box();
    FillBox(visual.bbox, box);
    box->set_color(ConvertToProtoColor(visual.color));
    box->set_selected(false);
    box->set_camera_name(visual.camera_name);
  }

  for (const auto& tracked : context.tracked_lights) {
    const auto& tracked_box = tracked.current_state.visual_light.bbox;
    auto* rectified = debug->add_rectified_roi();
    FillBox(tracked_box, rectified);
    rectified->set_color(
        ConvertToProtoColor(tracked.current_state.visual_light.color));
    rectified->set_selected(true);

    auto* selected_box = debug->add_box();
    FillBox(tracked_box, selected_box);
    selected_box->set_color(
        ConvertToProtoColor(tracked.current_state.visual_light.color));
    selected_box->set_selected(true);
    selected_box->set_camera_name(
        tracked.current_state.visual_light.camera_name);
  }
}

class CyberTrafficLightResultWriter final : public IResultWriterPort {
 public:
  explicit CyberTrafficLightResultWriter(
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

    message->set_contain_lights(result.color != LightColor::UNKNOWN);
    message->set_camera_id(ConvertCameraId(context.primary_camera_name));

    if (result.color != LightColor::UNKNOWN) {
      auto* light = message->add_traffic_light();
      light->set_id(result.signal_id);
      light->set_confidence(result.confidence);
      light->set_blink(result.blink);
      light->set_color(ConvertToProtoColor(result.color));
    }

    if (include_debug_) {
      auto* debug = message->mutable_traffic_light_debug();
      PopulateDebugInfo(context, debug);
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

}  // namespace

bool TrafficLightComponent::Init() {
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

bool TrafficLightComponent::InitDefaultPorts() {
  if (data_provider_ == nullptr) {
    data_provider_ = std::make_shared<InMemoryDataProviderPort>();
  }
  frame_input_port_ =
      std::dynamic_pointer_cast<IFrameInputPort>(data_provider_);
  if (frame_input_port_ == nullptr) {
    AERROR << "Configured data_provider does not implement IFrameInputPort";
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
    map_provider_ = std::make_shared<CachedMapProviderPort>();
  }
  if (v2x_provider_ == nullptr) {
    v2x_provider_ = std::make_shared<BufferedV2XProviderPort>();
  }
  v2x_input_port_ = std::dynamic_pointer_cast<IV2XInputPort>(v2x_provider_);
  if (v2x_input_port_ == nullptr) {
    AERROR << "Configured v2x_provider does not implement IV2XInputPort";
    return false;
  }
  if (result_writer_ == nullptr) {
    auto writer =
        node_->CreateWriter<apollo::perception::TrafficLightDetection>(
            output_channel_name_);
    auto main_writer =
        std::make_shared<CyberTrafficLightResultWriter>(writer, false);

    if (enable_debug_recording_) {
      auto debug_writer =
          node_->CreateWriter<apollo::perception::TrafficLightDetection>(
              debug_output_channel_name_);
      auto rich_debug_writer =
          std::make_shared<CyberTrafficLightResultWriter>(debug_writer, true);
      result_writer_ = std::make_shared<FanoutResultWriter>(
          std::vector<std::shared_ptr<IResultWriterPort>>{main_writer,
                                                          rich_debug_writer});
    } else {
      result_writer_ = main_writer;
    }
  }

  if (enable_debug_image_stream_ && debug_image_writer_ == nullptr) {
    debug_image_writer_ =
        node_->CreateWriter<apollo::drivers::Image>(debug_image_channel_name_);
  }
  return true;
}

bool TrafficLightComponent::PublishDecision(const PipelineContext& context) {
  if (result_writer_ == nullptr) {
    return true;
  }
  return result_writer_->Write(context, context.final_decision);
}

bool TrafficLightComponent::InitReaders() {
  if (camera_names_.size() != camera_channel_names_.size() ||
      camera_names_.empty()) {
    AERROR << "Invalid camera config, camera_names size: "
           << camera_names_.size()
           << ", camera_channel_names size: " << camera_channel_names_.size();
    return false;
  }

  for (size_t i = 0; i < camera_names_.size(); ++i) {
    const auto& camera_name = camera_names_[i];
    const auto& camera_channel_name = camera_channel_names_[i];
    node_->CreateReader<apollo::drivers::Image>(
        camera_channel_name,
        std::bind(&TrafficLightComponent::OnReceiveImage, this,
                  std::placeholders::_1, camera_name));
  }

  node_->CreateReader<apollo::v2x::IntersectionTrafficLightData>(
      v2x_channel_name_, std::bind(&TrafficLightComponent::OnReceiveV2XMsg,
                                   this, std::placeholders::_1));
  return true;
}

void TrafficLightComponent::OnReceiveImage(
    const std::shared_ptr<apollo::drivers::Image>& image,
    const std::string& camera_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_input_port_ == nullptr || image == nullptr) {
    return;
  }

  Image frame_image;
  frame_image.data = const_cast<char*>(image->data().data());
  frame_image.rows = static_cast<int>(image->height());
  frame_image.cols = static_cast<int>(image->width());
  frame_image.channels = 3;
  frame_image.encoding = image->encoding();

  CameraFrameState camera_frame;
  camera_frame.camera_name = camera_name;
  camera_frame.timestamp_ns =
      static_cast<uint64_t>(image->measurement_time() * 1e9);
  camera_frame.image = frame_image;

  if (!frame_input_port_->PushCameraFrame(++frame_counter_, camera_frame)) {
    return;
  }

  if (enable_debug_image_stream_ && debug_image_writer_ != nullptr) {
    debug_image_writer_->Write(image);
  }

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
      v2x_input_port_->PushV2XEvidence(evidence);
    }
  }
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
