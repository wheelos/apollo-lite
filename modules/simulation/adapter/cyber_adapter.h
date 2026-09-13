// Copyright 2026 WheelOS. All Rights Reserved.
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

//  Created Date: 2026-09-10
//  Author: daohu527

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <functional>
#include <utility>

#include "wheelos_msgs/chassis_msgs/chassis.pb.h"
#include "wheelos_msgs/chassis_msgs/chassis_detail.pb.h"
#include "wheelos_msgs/control_msgs/control_cmd.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"

#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/simulation/common/simulation_gflags.h"
#include "modules/simulation/core/simulation_types.h"

namespace apollo {
namespace simulation {

/**
 * @class CyberAdapter
 * @brief Handles Apollo Cyber I/O and converts wheelos_msgs to/from simulation
 * types.
 */
class CyberAdapter {
 public:
  using ChassisDetailFiller =
      std::function<void(const VehicleState&, apollo::canbus::ChassisDetail*)>;

  CyberAdapter() = default;
  ~CyberAdapter() = default;

  bool Init(const std::shared_ptr<cyber::Node>& node);
  void SetMaxSteerAngle(double max_steer_angle_rad) {
    max_steer_angle_rad_ = max_steer_angle_rad;
  }
  // Vehicle-specific protocols may populate ChassisDetail.Any without
  // coupling the generic Chassis message to a concrete vehicle model.
  void SetChassisDetailFiller(ChassisDetailFiller filler) {
    chassis_detail_filler_ = std::move(filler);
  }

  // Polls the latest command thread-safely. The return value is true only
  // when a new command arrived since the previous poll.
  bool PollCommand(VehicleCommand* cmd);

  // Publishes simulated Chassis and LocalizationEstimate to Cyber
  void PublishFeedback(const VehicleState& state);

  // Converter helpers
  static void ToChassis(const VehicleState& state,
                        apollo::canbus::Chassis* chassis,
                        double timestamp_sec = 0.0);
  static void ToLocalization(
      const VehicleState& state,
      apollo::localization::LocalizationEstimate* localization,
      double timestamp_sec = 0.0);
  static void FromControlCommand(const apollo::control::ControlCommand& msg,
                                 VehicleCommand* cmd,
                                 double max_steer_angle_rad = 0.50);

 private:
  void OnControlCommand(
      const std::shared_ptr<apollo::control::ControlCommand>& msg);

 private:
  std::shared_ptr<cyber::Node> node_;

  // Cyber Reader & Writers
  std::shared_ptr<cyber::Reader<apollo::control::ControlCommand>>
      control_reader_;
  std::shared_ptr<cyber::Writer<apollo::canbus::Chassis>> chassis_writer_;
  std::shared_ptr<cyber::Writer<apollo::canbus::ChassisDetail>>
      chassis_detail_writer_;
  std::shared_ptr<cyber::Writer<apollo::localization::LocalizationEstimate>>
      localization_writer_;

  std::mutex mutex_;
  VehicleCommand latest_command_{};
  bool has_received_cmd_{false};

  // Calibration parameters
  double max_steer_angle_rad_{0.50};  // ~28.6 degrees
  uint64_t msg_seq_num_{0};
  ChassisDetailFiller chassis_detail_filler_;
};

}  // namespace simulation
}  // namespace apollo
