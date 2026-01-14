// Copyright 2025 WheelOS. All Rights Reserved.
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

//  Created Date: 2026-01-03
//  Author: daohu527

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "boost/container/static_vector.hpp"

#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/local_view.pb.h"
#include "modules/control/proto/safety_status.pb.h"

#include "modules/common/util/debouncer.h"
#include "modules/control/safety/safety_defines.h"

namespace apollo {
namespace control {

struct SafetyResult {
  bool must_bypass = false;
  bool need_freeze = false;
};

class SafetyManager {
 public:
  SafetyManager() = default;
  virtual ~SafetyManager() = default;

  bool Init(const ControlConf& conf);

  // Phase 1: Pre-Computation Check
  // Returns true if critical failure detected (Skip Compute)
  SafetyResult PreCheck(const LocalView& view);

  // Phase 2: Post-Computation Check
  // Validates the calculated command
  SafetyResult PostCheck(const ControlCommand& cmd,
                         const ControlCommand& prev_cmd);

  // Phase 3: Policy Application
  // Overrides the command based on current SafetyState
  void ApplySafetyPolicy(ControlCommand* cmd);

  // Manual Reset Interface
  void TryReset(const PadMessage& pad_msg);

  SafetyState GetState() const { return current_state_; }

 private:
  void CheckPlanningTrajectory(const LocalView& view, SafetyResult* result);
  void CheckKinematics(const LocalView& view, SafetyResult* result);
  void CheckControlOutputDynamic(const ControlCommand& cmd,
                                 const ControlCommand& prev_cmd,
                                 SafetyResult* result);
  void Arbitrate();

  void ExecuteSoftStop(ControlCommand* cmd);
  void ExecuteHardEstop(ControlCommand* cmd);
  void ExecuteWarningPolicy(ControlCommand* cmd);

  void ReportFault(uint32_t id, FaultLevel level, FaultSource source);

 private:
  mutable std::mutex mutex_;

  ControlConf conf_;
  SafetyState current_state_ = SafetyState::kNormal;

  // Fixed-size container for deterministic memory usage
  boost::container::static_vector<FaultEvent, 16> active_faults_;

  // Debouncers for signal stability
  std::unique_ptr<CounterDebouncer> trajectory_loss_debouncer_;
  std::unique_ptr<CounterDebouncer> output_fault_debouncer_;
};

}  // namespace control
}  // namespace apollo
