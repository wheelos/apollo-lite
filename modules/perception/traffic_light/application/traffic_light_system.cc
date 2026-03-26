/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#include "modules/perception/traffic_light/application/traffic_light_system.h"

#include <sys/stat.h>
#include <unistd.h>

#include <limits>
#include <map>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace application {

namespace {

using TLCamID = apollo::perception::TrafficLightDetection::CameraID;
using apollo::cyber::Clock;

class TLInfo {
 public:
  cv::Scalar tl_color;
  std::string tl_string;
  std::string tl_string_ex;
};

std::map<base::TLColor, TLInfo> kTlInfos = {
    {base::TLColor::TL_UNKNOWN_COLOR,
     {cv::Scalar(255, 255, 255), "UNKNOWN", "UNKNOWN traffic light"}},
    {base::TLColor::TL_RED,
     {cv::Scalar(0, 0, 255), "RED", "RED traffic light"}},
    {base::TLColor::TL_GREEN,
     {cv::Scalar(0, 255, 0), "GREEN", "GREEN traffic light"}},
    {base::TLColor::TL_YELLOW,
     {cv::Scalar(0, 255, 255), "YELLOW", "YELLOW traffic light"}}};

int GetTrafficGpuId(const pipeline::PipelineConfig& pipeline_config) {
  if (!pipeline_config.traffic_light_config()
           .trafficlights_perception_config()
           .has_gpu_id()) {
    return -1;
  }
  return pipeline_config.traffic_light_config()
      .trafficlights_perception_config()
      .gpu_id();
}

}  // namespace

bool TrafficLightSystem::Init(const domain::TrafficLightComponentConfig& config,
                              const pipeline::PipelineConfig& pipeline_config) {
  config_ = config;
  if (config_.camera_names.empty() ||
      config_.camera_names.size() != config_.camera_channel_names.size()) {
    AERROR << "Invalid traffic light camera config. names: "
           << config_.camera_names.size() << ", channels: "
           << config_.camera_channel_names.size();
    return false;
  }
  frame_.reset(new camera::CameraFrame);
  v2x_msg_buffer_.set_capacity(config.max_v2x_msg_buff_size);
  if (!CreateDebugDir()) {
    AERROR << "Failed to create traffic light debug directory.";
  }
  if (!pipeline_.Init(pipeline_config)) {
    return false;
  }
  camera::TrafficLightPreprocessorInitOptions preprocessor_options;
  preprocessor_options.camera_names = config.camera_names;
  preprocessor_options.sync_interval_seconds =
      static_cast<float>(config.sync_interval_seconds);
  if (!scene_gateway_.Init(config, preprocessor_options)) {
    return false;
  }

  data_provider_init_options_.image_height = config.image_height;
  data_provider_init_options_.image_width = config.image_width;
  data_provider_init_options_.device_id = GetTrafficGpuId(pipeline_config);
  if (data_provider_init_options_.device_id < 0) {
    AERROR << "Traffic light gpu_id is not configured in pipeline config.";
    return false;
  }
  data_provider_init_options_.do_undistortion = config.enable_undistortion;
  return InitCameraFrame();
}

bool TrafficLightSystem::CreateDebugDir() {
  if (access("/apollo/debug_vis", F_OK) == 0) {
    return true;
  }
  return mkdir("/apollo/debug_vis", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) ==
         0;
}

bool TrafficLightSystem::InitCameraFrame() {
  for (const auto& camera_name : config_.camera_names) {
    data_provider_init_options_.sensor_name = camera_name;
    std::shared_ptr<camera::DataProvider> data_provider(new camera::DataProvider);
    if (!data_provider->Init(data_provider_init_options_)) {
      return false;
    }
    data_providers_map_[camera_name] = data_provider;
  }
  return true;
}

void TrafficLightSystem::PushV2X(
    const apollo::v2x::IntersectionTrafficLightData& v2x_msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  v2x_msg_buffer_.push_back(v2x_msg);
}

bool TrafficLightSystem::ProcessImage(
    const std::shared_ptr<apollo::drivers::Image>& image,
    const std::string& camera_name,
    std::shared_ptr<apollo::perception::TrafficLightDetection>* out_msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double receive_img_timestamp = Clock::NowInSeconds();
  const double image_msg_ts = image->measurement_time() +
                              config_.image_timestamp_offset;

  scene_gateway_.RecordCameraHeartbeat(camera_name, image_msg_ts);
  if (!scene_gateway_.CheckCameraImageStatus(
          image_msg_ts, config_.check_image_status_interval_thresh,
          camera_name)) {
    return false;
  }
  if (!scene_gateway_.UpdateCameraSelection(image_msg_ts, frame_.get())) {
    AWARN << "Traffic light camera selection update failed.";
  }
  if (last_proc_image_ts_ > 0.0 &&
      receive_img_timestamp - last_proc_image_ts_ <
          config_.proc_interval_seconds) {
    return false;
  }
  if (!scene_gateway_.SyncInformation(image_msg_ts, camera_name)) {
    return false;
  }

  frame_->data_provider = data_providers_map_.at(camera_name).get();
  frame_->data_provider->FillImageData(
      config_.image_height, config_.image_width,
      reinterpret_cast<const uint8_t*>(image->data().data()),
      image->encoding());
  frame_->timestamp = image_msg_ts;

  if (std::fabs(image_msg_ts - receive_img_timestamp) >
      config_.image_sys_ts_diff_threshold) {
    AWARN << "Traffic light image timestamp drift detected.";
  }

  if (!scene_gateway_.VerifyLightsProjection(image_msg_ts, camera_name,
                                             frame_.get())) {
    AWARN << "Traffic light projection verification failed.";
  }

  pipeline::DataFrame data_frame;
  data_frame.camera_frame = frame_.get();
  if (!pipeline_.Process(&data_frame)) {
    return false;
  }
  SyncV2XTrafficLights(frame_.get());
  last_proc_image_ts_ = Clock::NowInSeconds();
  return BuildOutputMessage(frame_.get(), camera_name, out_msg);
}

void TrafficLightSystem::SyncV2XTrafficLights(camera::CameraFrame* frame) {
  const double camera_frame_timestamp = frame->timestamp;
  for (auto& light : frame->traffic_lights) {
    for (auto itr = v2x_msg_buffer_.rbegin(); itr != v2x_msg_buffer_.rend();
         ++itr) {
      double v2x_timestamp = itr->header().timestamp_sec();
      if (std::fabs(camera_frame_timestamp - v2x_timestamp) >=
          config_.v2x_sync_interval_seconds) {
        continue;
      }
      const auto& v2x_lights = itr->road_traffic_light(0);
      for (int i = 0; i < v2x_lights.single_traffic_light_size(); ++i) {
        const auto& v2x_light = v2x_lights.single_traffic_light(i);
        if (light->id != v2x_light.id()) {
          continue;
        }
        switch (v2x_light.color()) {
          case apollo::v2x::SingleTrafficLight::RED:
            light->status.color = base::TLColor::TL_RED;
            break;
          case apollo::v2x::SingleTrafficLight::YELLOW:
            light->status.color = base::TLColor::TL_YELLOW;
            break;
          case apollo::v2x::SingleTrafficLight::GREEN:
            light->status.color = base::TLColor::TL_GREEN;
            break;
          case apollo::v2x::SingleTrafficLight::BLACK:
            light->status.color = base::TLColor::TL_BLACK;
            break;
          case apollo::v2x::SingleTrafficLight::FLASH_GREEN:
            light->status.color = base::TLColor::TL_GREEN;
            light->status.blink = true;
            break;
          default:
            light->status.color = base::TLColor::TL_UNKNOWN_COLOR;
            break;
        }
      }
      break;
    }
  }
}

bool TrafficLightSystem::BuildOutputMessage(
    camera::CameraFrame* frame, const std::string& camera_name,
    std::shared_ptr<apollo::perception::TrafficLightDetection>* out_msg) {
  const std::map<std::string, TLCamID> camera_id_map = {
      {"front_24mm", TrafficLightDetection::CAMERA_FRONT_LONG},
      {"front_12mm", TrafficLightDetection::CAMERA_FRONT_NARROW},
      {"front_6mm", TrafficLightDetection::CAMERA_FRONT_SHORT},
      {"front_fisheye", TrafficLightDetection::CAMERA_FRONT_WIDE}};

  out_msg->reset(new apollo::perception::TrafficLightDetection);
  auto* header = (*out_msg)->mutable_header();
  header->set_timestamp_sec(Clock::NowInSeconds());
  header->set_camera_timestamp(static_cast<uint64_t>(frame->timestamp * 1e9));
  if (camera_id_map.find(camera_name) == camera_id_map.end()) {
    return false;
  }
  (*out_msg)->set_camera_id(camera_id_map.at(camera_name));

  auto decision = domain::AggregateTrafficLightDecision(&frame->traffic_lights);
  if (decision.contain_lights) {
    for (const auto& light : frame->traffic_lights) {
      auto* light_result = (*out_msg)->add_traffic_light();
      light_result->set_id(light->id);
      light_result->set_confidence(decision.dominant_confidence);
      light_result->set_color(
          static_cast<apollo::perception::TrafficLight_Color>(
              decision.dominant_color));
      light_result->set_blink(light->status.blink);
    }
    (*out_msg)->set_contain_lights(true);
  }
  if (!BuildDebugMessage(frame, camera_name, out_msg, decision)) {
    return false;
  }
  return true;
}

void TrafficLightSystem::TransRect2Box(
    const base::RectI& rect, apollo::perception::TrafficLightBox* box) const {
  box->set_x(rect.x);
  box->set_y(rect.y);
  box->set_width(rect.width);
  box->set_height(rect.height);
}

bool TrafficLightSystem::BuildDebugMessage(
    const camera::CameraFrame* frame,
  const std::string& camera_name,
    std::shared_ptr<apollo::perception::TrafficLightDetection>* out_msg,
    const domain::TrafficLightDecision& decision) {
  auto* light_debug = (*out_msg)->mutable_traffic_light_debug();
  const auto& lights = frame->traffic_lights;
  light_debug->set_signal_num(static_cast<int>(lights.size()));

  if (!lights.empty() && !lights[0]->region.debug_roi.empty()) {
    const auto& debug_roi = lights[0]->region.debug_roi;
    TransRect2Box(debug_roi[0], light_debug->mutable_cropbox());
    for (auto iter = debug_roi.begin() + 1; iter != debug_roi.end(); ++iter) {
      TransRect2Box(*iter, light_debug->add_box());
      TransRect2Box(*iter, light_debug->add_debug_roi());
    }
  }
  for (const auto& light : lights) {
    auto* box = light_debug->add_box();
    TransRect2Box(light->region.detection_roi, box);
    box->set_color(static_cast<TrafficLight_Color>(light->status.color));
    box->set_selected(true);
    TransRect2Box(light->region.projection_roi, light_debug->add_box());
    TransRect2Box(light->region.projection_roi,
                  light_debug->add_projected_roi());
    if (!light->region.debug_roi.empty()) {
      TransRect2Box(light->region.debug_roi[0], light_debug->add_crop_roi());
    }
    auto* rectified_roi = light_debug->add_rectified_roi();
    TransRect2Box(light->region.detection_roi, rectified_roi);
    rectified_roi->set_color(
        static_cast<TrafficLight_Color>(light->status.color));
    rectified_roi->set_selected(true);
  }
  light_debug->set_distance_to_stop_line(
      scene_gateway_.ComputeStoplineDistance(frame->timestamp, camera_name));
  if (FLAGS_start_visualizer) {
    Visualize(*frame, lights, decision);
  }
  return true;
}

void TrafficLightSystem::Visualize(
    const camera::CameraFrame& frame,
    const std::vector<base::TrafficLightPtr>& lights,
    const domain::TrafficLightDecision& decision) const {
  if (lights.empty()) {
    return;
  }
  cv::Mat output_image(config_.image_height, config_.image_width, CV_8UC3,
                       cv::Scalar(0, 0, 0));
  base::Image8U out_image(config_.image_height, config_.image_width,
                          base::Color::RGB);
  camera::DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  frame.data_provider->GetImage(image_options, &out_image);
  memcpy(output_image.data, out_image.cpu_data(),
         out_image.total() * sizeof(uint8_t));

  for (const auto& light : lights) {
    if (!light->region.debug_roi.empty()) {
      const auto& crop_roi = light->region.debug_roi[0];
      cv::rectangle(output_image,
                    cv::Rect(crop_roi.x, crop_roi.y, crop_roi.width,
                             crop_roi.height),
                    cv::Scalar(255, 255, 255), 2);
    }
    const auto& projection_roi = light->region.projection_roi;
    cv::rectangle(output_image,
                  cv::Rect(projection_roi.x, projection_roi.y,
                           projection_roi.width, projection_roi.height),
                  cv::Scalar(255, 0, 0), 3);
    const auto& rectified_roi = light->region.detection_roi;
    auto info_it = kTlInfos.find(light->status.color);
    cv::Scalar tl_color = info_it == kTlInfos.end()
                              ? cv::Scalar(255, 255, 255)
                              : info_it->second.tl_color;
    cv::rectangle(output_image,
                  cv::Rect(rectified_roi.x, rectified_roi.y,
                           rectified_roi.width, rectified_roi.height),
                  tl_color, 2);
  }

  auto info_it = kTlInfos.find(decision.dominant_color);
  cv::Scalar tl_color = info_it == kTlInfos.end()
                            ? cv::Scalar(255, 255, 255)
                            : info_it->second.tl_color;
  std::string tl_string = info_it == kTlInfos.end()
                              ? "UNKNOWN traffic light"
                              : info_it->second.tl_string_ex;
  const double total = std::max(1.0, decision.vote_stats.red +
                                         decision.vote_stats.green +
                                         decision.vote_stats.yellow +
                                         decision.vote_stats.unknown);
  cv::putText(output_image, tl_string, cv::Point(10, 90),
              cv::FONT_HERSHEY_DUPLEX, 2.0, tl_color, 3);
  char str[100];
  snprintf(str, sizeof(str), "Red lights:%.2f", decision.vote_stats.red / total);
  cv::putText(output_image, str, cv::Point(10, 150), cv::FONT_HERSHEY_DUPLEX,
              1.5, cv::Scalar(0, 0, 255), 3);
  snprintf(str, sizeof(str), "Green lights:%.2f",
           decision.vote_stats.green / total);
  cv::putText(output_image, str, cv::Point(10, 200), cv::FONT_HERSHEY_DUPLEX,
              1.5, cv::Scalar(0, 255, 0), 3);
  snprintf(str, sizeof(str), "Yellow lights:%.2f",
           decision.vote_stats.yellow / total);
  cv::putText(output_image, str, cv::Point(10, 250), cv::FONT_HERSHEY_DUPLEX,
              1.5, cv::Scalar(0, 255, 255), 3);
  snprintf(str, sizeof(str), "Unknown lights:%.2f",
           decision.vote_stats.unknown / total);
  cv::putText(output_image, str, cv::Point(10, 300), cv::FONT_HERSHEY_DUPLEX,
              1.5, cv::Scalar(255, 255, 255), 3);
  cv::resize(output_image, output_image, cv::Size(), 0.5, 0.5);
  cv::imwrite(absl::StrCat("/apollo/debug_vis/", std::to_string(frame.timestamp),
                           ".jpg"),
              output_image);
}

}  // namespace application
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
