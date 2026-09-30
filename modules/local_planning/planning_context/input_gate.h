// Copyright 2026 WheelOS All Rights Reserved.
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

#pragma once

#include "modules/common/status/status.h"
#include "modules/local_planning/planning_context/input_contract.h"

namespace apollo {
namespace local_planning {

class InputGate {
 public:
  explicit InputGate(const InputPolicy& policy);

  void Disarm();
  common::Status BeginEpoch(const OdometryInput& odometry, double now);
  common::Status Admit(const CycleInput& input);

 private:
  common::Status ValidatePolicy() const;
  common::Status ValidateStamp(const SourceStamp& stamp, double now,
                               double max_age) const;
  common::Status ValidateOdometry(const OdometryInput& input, double now) const;

  InputPolicy policy_;
  OdomEpoch epoch_;
  bool armed_ = false;
  bool has_epoch_ = false;
  bool has_cycle_ = false;
  double epoch_start_time_ = 0.0;
  OdometryInput initial_odometry_;
  CycleInput last_input_;
};

}  // namespace local_planning
}  // namespace apollo
