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
#include "modules/perception/fusion/base/sensor_data_manager.h"

#include <algorithm>
#include <utility>

#include "cyber/common/log.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"

namespace apollo {
namespace perception {
namespace fusion {

SensorDataManager::SensorDataManager() { CHECK_EQ(this->Init(), true); }

bool SensorDataManager::Init() {
  if (inited_) {
    return true;
  }
  sensor_manager_ = common::SensorManager::Instance();
  inited_ = true;
  return true;
}

void SensorDataManager::Reset() {
  inited_ = false;
  sensor_manager_ = nullptr;
  sensors_.clear();
}

bool SensorDataManager::AddSensorMeasurements(
    const base::FrameConstPtr& frame_ptr) {
  if (!IsKnownSensor(frame_ptr)) {
    AERROR << "Unknown sensor or mismatched sensor type.";
    return false;
  }
  const base::SensorInfo& sensor_info = frame_ptr->sensor_info;
  std::string sensor_id = sensor_info.name;
  const auto it = sensors_.find(sensor_id);
  SensorPtr sensor_ptr = nullptr;
  if (it == sensors_.end()) {
    if (!sensor_manager_->IsSensorExist(sensor_id)) {
      AERROR << "Failed to find sensor " << sensor_id << " in sensor manager.";
      return false;
    }
    sensor_ptr.reset(new Sensor(sensor_info));
    sensors_.emplace(sensor_id, sensor_ptr);
  } else {
    sensor_ptr = it->second;
  }

  sensor_ptr->AddFrame(frame_ptr);
  return true;
}

bool SensorDataManager::AddSensorFrame(const SensorFramePtr& frame) {
  if (!frame) {
    AERROR << "Cannot register a null sensor frame.";
    return false;
  }
  auto header = std::make_shared<base::Frame>();
  header->timestamp = frame->GetTimestamp();
  header->sensor_info.name = frame->GetSensorId();
  header->sensor_info.type = frame->GetSensorType();
  if (!frame->GetPose(&header->sensor2world_pose)) {
    AERROR << "Missing processed sensor pose.";
    return false;
  }
  return AddSensorMeasurements(header);
}

bool SensorDataManager::IsLidar(const base::FrameConstPtr& frame_ptr) {
  base::SensorType type = frame_ptr->sensor_info.type;
  return sensor_manager_->IsLidar(type);
}

bool SensorDataManager::IsRadar(const base::FrameConstPtr& frame_ptr) {
  base::SensorType type = frame_ptr->sensor_info.type;
  return sensor_manager_->IsRadar(type);
}

bool SensorDataManager::IsCamera(const base::FrameConstPtr& frame_ptr) {
  base::SensorType type = frame_ptr->sensor_info.type;
  return sensor_manager_->IsCamera(type);
}

bool SensorDataManager::IsKnownSensor(
    const base::FrameConstPtr& frame_ptr) const {
  base::SensorInfo info;
  return sensor_manager_ && frame_ptr &&
         sensor_manager_->GetSensorInfo(frame_ptr->sensor_info.name, &info) &&
         info.type == frame_ptr->sensor_info.type;
}

void SensorDataManager::GetLatestSensorFrames(
    double timestamp, const std::string& sensor_id,
    std::vector<SensorFramePtr>* frames) const {
  if (frames == nullptr) {
    AERROR << "Nullptr error.";
    return;
  }
  const auto it = sensors_.find(sensor_id);
  if (it == sensors_.end()) {
    return;
  }
  return it->second->QueryLatestFrames(timestamp, frames);
}

void SensorDataManager::GetLatestFrames(
    double timestamp, std::vector<SensorFramePtr>* frames) const {
  if (frames == nullptr) {
    AERROR << "Nullptr error.";
    return;
  }

  frames->clear();
  for (auto it = sensors_.begin(); it != sensors_.end(); ++it) {
    SensorFramePtr frame = it->second->QueryLatestFrame(timestamp);
    if (frame != nullptr) {
      frames->push_back(frame);
    }
  }

  if (frames->empty()) {
    return;
  }

  std::sort(frames->begin(), frames->end(),
            [](const SensorFramePtr& p1, const SensorFramePtr& p2) {
              return p1->GetTimestamp() < p2->GetTimestamp();
            });
}

bool SensorDataManager::GetPose(const std::string& sensor_id, double timestamp,
                                Eigen::Affine3d* pose) const {
  if (pose == nullptr) {
    AERROR << "Nullptr error.";
    return false;
  }

  const auto it = sensors_.find(sensor_id);
  if (it == sensors_.end()) {
    AERROR << "Failed to find sensor " << sensor_id << " for get pose.";
    return false;
  }

  return it->second->GetPose(timestamp, pose);
}

base::BaseCameraModelPtr SensorDataManager::GetCameraIntrinsic(
    const std::string& sensor_id) const {
  return sensor_manager_->GetUndistortCameraModel(sensor_id);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
