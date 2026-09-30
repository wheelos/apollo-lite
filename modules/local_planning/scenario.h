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

#include <memory>
#include <string>
#include <vector>

#include "modules/common/status/status.h"
#include "modules/local_planning/planning_context/input_gate.h"

namespace apollo {
namespace local_planning {

class LaneFollowPlanner;

class Task {
 public:
  virtual ~Task() = default;
  virtual std::string Name() const = 0;
  virtual common::Status Execute(const CycleInput& input) = 0;
  virtual void Reset() = 0;
};

class Stage {
 public:
  explicit Stage(std::vector<std::unique_ptr<Task>> tasks);
  common::Status Validate() const;
  common::Status Execute(const CycleInput& input);
  void Reset();

 private:
  std::vector<std::unique_ptr<Task>> tasks_;
};

enum class ScenarioState { IDLE, TASKS_COMPLETED, INVALID };

// A single-stage foundation. Completion never denotes an executable trajectory.
class Scenario {
 public:
  Scenario(const InputPolicy& policy, std::vector<std::unique_ptr<Task>> tasks);
  common::Status BeginEpoch(const OdometryInput& odometry, double now);
  common::Status Process(const CycleInput& input);
  void Invalidate();
  ScenarioState state() const { return state_; }
  bool input_admitted() const { return input_admitted_; }

 private:
  friend class LaneFollowPlanner;
  common::Status ProcessAdmitted(const CycleInput& input);
  void Disarm();

  InputGate gate_;
  Stage stage_;
  ScenarioState state_ = ScenarioState::IDLE;
  bool input_admitted_ = false;
};

}  // namespace local_planning
}  // namespace apollo
