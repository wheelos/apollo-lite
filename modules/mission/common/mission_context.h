// Copyright 2025 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-12-13
//  Author: daohu527

#pragma once

#include <memory>
#include <mutex>
#include <unordered_map>

#include "modules/common_msgs/basic_msgs/geometry.pb.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

#include "cyber/common/macros.h"
#include "cyber/cyber.h"

namespace apollo {
namespace mission {

class MissionContext {
 public:
  void SetRoutingWriter(
      const std::shared_ptr<cyber::Writer<routing::RoutingRequest>>& writer) {
    routing_writer_ = writer;
  }

  // Data storage (written by Component)
  void UpdateChassis(const std::shared_ptr<canbus::Chassis>& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    chassis_ = msg;
  }

  void UpdateLocalization(
      const std::shared_ptr<localization::LocalizationEstimate>& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    localization_ = msg;
  }

  void SaveWaypoint(const std::string& name, const common::PointENU& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    waypoints_[name] = pose;
  }

  // Data read (read by BT Nodes)
  std::shared_ptr<canbus::Chassis> GetChassis() {
    std::lock_guard<std::mutex> lock(mutex_);
    return chassis_;
  }

  std::shared_ptr<localization::LocalizationEstimate> GetLocalization() {
    std::lock_guard<std::mutex> lock(mutex_);
    return localization_;
  }

  bool GetWaypoint(const std::string& name, common::PointENU* out_pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (waypoints_.find(name) == waypoints_.end()) {
      return false;
    }
    *out_pose = waypoints_[name];
    return true;
  }

  void SendRoutingRequest(const common::PointENU& end_pose) {
    if (routing_writer_ == nullptr) {
      return;
    }
    routing::RoutingRequest request;
    auto* waypoint = request.add_waypoint();
    *waypoint->mutable_pose() = end_pose;
    routing_writer_->Write(request);
  }

  void SetCurrentMissionId(const std::string& id) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_mission_id_ = id;
  }

  std::string GetCurrentMissionId() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_mission_id_;
  }

 private:
  mutable std::mutex mutex_;

  std::string current_mission_id_;

  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> routing_writer_;
  std::shared_ptr<canbus::Chassis> chassis_;
  std::shared_ptr<localization::LocalizationEstimate> localization_;
  std::unordered_map<std::string, common::PointENU> waypoints_;

  DECLARE_SINGLETON(MissionContext)
};

}  // namespace mission
}  // namespace apollo
