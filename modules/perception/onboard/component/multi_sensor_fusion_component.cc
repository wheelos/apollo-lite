/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/onboard/component/multi_sensor_fusion_component.h"

#include <cmath>
#include <memory>

#include "cyber/common/file.h"
#include "cyber/time/clock.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/onboard/common_flags/common_flags.h"
#include "modules/perception/onboard/msg_serializer/msg_serializer.h"

namespace apollo {
namespace perception {
namespace onboard {

MultiSensorFusionComponent::~MultiSensorFusionComponent() {
  if (timer_) timer_->Stop();
}

bool MultiSensorFusionComponent::Init() {
  FusionComponentConfig config;
  if (!GetProtoConfig(&config)) return false;
  if (config.publish_period_ms() == 0 ||
      !std::isfinite(config.reorder_window()) || config.reorder_window() < 0) {
    AERROR << "Invalid fusion publication period or reorder window.";
    return false;
  }
  fusion_name_ = config.fusion_name();
  fusion_method_ = config.fusion_method();
  fusion_main_sensor_ = config.fusion_main_sensor();
  const auto path = cyber::common::GetAbsolutePath(
      config.sensor_fusion_conf_dir(), config.sensor_fusion_conf_file());
  if (!cyber::common::GetProtoFromFile(path, &multi_sensor_fusion_config_)) {
    AERROR << "Failed to read fusion pipeline: " << path;
    return false;
  }
  const pipeline::StageType expected_stages[] = {
      pipeline::StageType::ALL_LATEST_FUSION,
      pipeline::StageType::PROBABILISTIC_FUSION,
      pipeline::StageType::COLLECT_FUSED_OBJECT};
  if (multi_sensor_fusion_config_.stage_type_size() != 3 ||
      multi_sensor_fusion_config_.stage_config_size() != 3) {
    AERROR << "Periodic fusion requires scheduler, fusion and collection.";
    return false;
  }
  for (int index = 0; index < 3; ++index) {
    if (multi_sensor_fusion_config_.stage_type(index) != expected_stages[index] ||
        multi_sensor_fusion_config_.stage_config(index).stage_type() !=
            expected_stages[index] ||
        !multi_sensor_fusion_config_.stage_config(index).enabled()) {
      AERROR << "Invalid fusion pipeline order or disabled required stage.";
      return false;
    }
  }
  fusion::AllLatestFusionConfig scheduler_config;
  bool scheduler_found = false;
  for (const auto& stage : multi_sensor_fusion_config_.stage_config()) {
    if (stage.stage_type() != pipeline::StageType::ALL_LATEST_FUSION) continue;
    scheduler_found =
        stage.enabled() &&
        stage.all_latest_fusion_config().main_sensor() == fusion_main_sensor_;
    scheduler_config = stage.all_latest_fusion_config();
    if (config.reorder_window() >=
        stage.all_latest_fusion_config().max_prediction_age()) {
      AERROR << "Reorder window must be smaller than maximum prediction age.";
      return false;
    }
  }
  if (!scheduler_found || !InitAlgorithmPlugin()) {
    AERROR << "Fusion requires an enabled scheduler with matching main sensor.";
    return false;
  }
  const auto& fusion_config =
      multi_sensor_fusion_config_.stage_config(1).probabilistic_fusion_config();
  if (scheduler_config.use_lidar() != fusion_config.use_lidar() ||
      scheduler_config.use_radar() != fusion_config.use_radar() ||
      scheduler_config.use_camera() != fusion_config.use_camera() ||
      std::fabs(scheduler_config.max_prediction_age() -
                fusion_config.max_prediction_age()) > 1e-9) {
    AERROR << "Fusion scheduler and estimator configuration must agree.";
    return false;
  }
  auto source_validator = [scheduler_config](const std::string& source) {
    auto* manager = common::SensorManager::Instance();
    base::SensorInfo info;
    if (!manager->GetSensorInfo(source, &info)) return false;
    return (manager->IsLidar(source) && scheduler_config.use_lidar()) ||
           (manager->IsRadar(source) && scheduler_config.use_radar()) ||
           (manager->IsCamera(source) && scheduler_config.use_camera());
  };
  if (!source_validator(fusion_main_sensor_)) {
    AERROR << "Fusion main sensor must be registered and enabled.";
    return false;
  }
  if (!runtime_.Init(fusion_main_sensor_, config.reorder_window(),
                     [this](fusion::FusionFrame* frame) {
                       pipeline::DataFrame data;
                       data.fusion_frame = frame;
                       return fusion_->Process(&data);
                     }, source_validator)) {
    return false;
  }
  writer_ = node_->CreateWriter<PerceptionObstacles>(
      config.output_obstacles_channel_name());
  inner_writer_ = node_->CreateWriter<SensorFrameMessage>(
      config.output_viz_fused_content_channel_name());
  if (!writer_ || !inner_writer_) {
    AERROR << "Failed to create fusion writers.";
    return false;
  }
  if (config.object_in_roi_check()) {
    AWARN
        << "Fusion ROI filtering remains unsupported; no map-based filtering.";
  }
  timer_ = std::make_unique<cyber::Timer>(
      config.publish_period_ms(), [this]() { PublishTick(); }, false);
  timer_->Start();
  return true;
}

bool MultiSensorFusionComponent::InitAlgorithmPlugin() {
  fusion_.reset(
      fusion::BaseMultiSensorFusionRegisterer::GetInstanceByName(fusion_name_));
  if (!fusion_ || !fusion_->Init(multi_sensor_fusion_config_)) {
    AERROR << "Failed to initialize fusion pipeline: " << fusion_name_;
    return false;
  }
  return true;
}

bool MultiSensorFusionComponent::Proc(
    const std::shared_ptr<SensorFrameMessage>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!message) {
    AERROR << "Null fusion input message.";
    return false;
  }
  if (message->process_stage_ == ProcessStage::SENSOR_FUSION) return true;
  if (message->error_code_ != apollo::common::ErrorCode::OK) {
    return runtime_.ReportSourceError(message->sensor_id_, message->timestamp_);
  }
  if (!message->frame_ ||
      message->sensor_id_ != message->frame_->sensor_info.name) {
    AERROR << "Missing fusion frame or inconsistent source identity.";
    return false;
  }
  auto frame = std::make_shared<base::Frame>(*message->frame_);
  frame->timestamp = message->timestamp_;
  if (!runtime_.Add(frame)) return false;
  if (message->sensor_id_ == fusion_main_sensor_) {
    pending_lidar_timestamps_[frame->timestamp] = message->lidar_timestamp_;
  }
  return true;
}

void MultiSensorFusionComponent::PublishTick() {
  std::lock_guard<std::mutex> lock(mutex_);
  fusion::FusionFrame frame;
  const bool success = runtime_.Tick(cyber::Clock::NowInSeconds(), &frame);
  if (success) {
    for (const auto& processed : frame.sensor_frames) {
      if (processed->GetSensorId() != fusion_main_sensor_) continue;
      const auto metadata = pending_lidar_timestamps_.find(
          processed->GetTimestamp());
      if (metadata != pending_lidar_timestamps_.end()) {
        lidar_timestamp_ = metadata->second;
        pending_lidar_timestamps_.erase(pending_lidar_timestamps_.begin(),
            pending_lidar_timestamps_.upper_bound(processed->GetTimestamp()));
      }
    }
  }
  const auto error = success && !frame.degraded
                         ? apollo::common::ErrorCode::OK
                         : apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
  auto output = std::make_shared<PerceptionObstacles>();
  const double timestamp =
      frame.frame ? frame.frame->timestamp : cyber::Clock::NowInSeconds();
  if (!success) frame.fused_objects.clear();
  if (!MsgSerializer::SerializeMsg(timestamp, lidar_timestamp_, ++sequence_,
                                   frame.fused_objects, error, output.get())) {
    AERROR << "Failed to serialize periodic fusion output.";
    return;
  }
  if (!writer_->Write(output)) {
    AERROR << "Failed to publish periodic fusion output.";
  }
  if (FLAGS_obs_enable_visualization && frame.frame && frame.has_publish_pose) {
    auto visualization = std::make_shared<SensorFrameMessage>();
    visualization->timestamp_ = timestamp;
    visualization->sensor_id_ = fusion_main_sensor_;
    visualization->seq_num_ = sequence_;
    visualization->process_stage_ = ProcessStage::SENSOR_FUSION;
    visualization->error_code_ = error;
    visualization->frame_ = frame.frame;
    visualization->frame_->objects = frame.fused_objects;
    if (!inner_writer_->Write(visualization)) {
      AERROR << "Failed to publish fusion visualization.";
    }
  }
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
