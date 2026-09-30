/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/fusion/lib/data_fusion/all_latest_fusion/all_latest_fusion.h"

#include <cmath>

#include "modules/common/util/string_util.h"
#include "modules/perception/fusion/base/sensor_data_manager.h"
#include "modules/perception/fusion/base/observation_validation.h"
#include "modules/perception/pipeline/data_frame.h"

namespace apollo {
namespace perception {
namespace fusion {

bool AllLatestFusion::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }

  all_latest_fusion_config_ = stage_config.all_latest_fusion_config();

  main_sensor_ = all_latest_fusion_config_.main_sensor();
  use_lidar_ = all_latest_fusion_config_.use_lidar();
  use_radar_ = all_latest_fusion_config_.use_radar();
  use_camera_ = all_latest_fusion_config_.use_camera();
  if (main_sensor_.empty() || all_latest_fusion_config_.queue_capacity() == 0 ||
      !std::isfinite(all_latest_fusion_config_.main_sensor_timeout()) ||
      all_latest_fusion_config_.main_sensor_timeout() <= 0 ||
      !std::isfinite(all_latest_fusion_config_.max_prediction_age()) ||
      all_latest_fusion_config_.max_prediction_age() <= 0) {
    AERROR << "Invalid fusion scheduling configuration.";
    return false;
  }
  scheduler_ = FrameScheduler(all_latest_fusion_config_.queue_capacity());
  return SensorDataManager::Instance()->Init();
}

bool AllLatestFusion::Process(DataFrame* data_frame) {
  if (data_frame == nullptr) {
    return false;
  }

  FusionFrame* fusion_frame = data_frame->fusion_frame;
  if (fusion_frame == nullptr) return false;

  base::FrameConstPtr sensor_frame = fusion_frame->frame;
  if (sensor_frame == nullptr) return false;

  SensorDataManager* sensor_data_manager = SensorDataManager::Instance();
  fusion_frame->ready = false;
  fusion_frame->admitted = false;
  fusion_frame->sensor_frames.clear();
  if (!fusion_frame->publish_tick) {
    if (!sensor_data_manager->IsKnownSensor(sensor_frame)) {
      AERROR << "Unregistered fusion source or inconsistent source type.";
      return false;
    }
    if (!use_lidar_ && sensor_data_manager->IsLidar(sensor_frame)) {
      return true;
    }
    if (!use_radar_ && sensor_data_manager->IsRadar(sensor_frame)) {
      return true;
    }
    if (!use_camera_ && sensor_data_manager->IsCamera(sensor_frame)) {
      return true;
    }
    for (const auto& object : sensor_frame->objects) {
      if (!object ||
          (sensor_data_manager->IsCamera(sensor_frame) &&
           !ValidCameraObservation(*object)) ||
          (sensor_data_manager->IsRadar(sensor_frame) &&
           (!std::isfinite(object->radar_supplement.range) ||
            object->radar_supplement.range < 0))) {
        AERROR << "Invalid modality-specific fusion observation.";
        return false;
      }
    }
    fusion_frame->admitted = scheduler_.Add(sensor_frame);
    return fusion_frame->admitted;
  }

  std::vector<base::FrameConstPtr> frames;
  if (!scheduler_.Drain(sensor_frame->timestamp, &frames)) {
    return false;
  }
  for (const auto& frame : frames) {
    fusion_frame->sensor_frames.emplace_back(new SensorFrame(frame));
  }
  fusion_frame->ready = true;
  fusion_frame->degraded = !scheduler_.HasFreshSensor(
      main_sensor_, sensor_frame->timestamp,
      all_latest_fusion_config_.main_sensor_timeout());
  fusion_frame->max_prediction_age =
      all_latest_fusion_config_.max_prediction_age();
  return true;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
