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

//  Created Date: 2025-12-07
//  Author: daohu527

#include "modules/planning/scenarios/park/mission_idle/mission_idle_stage.h"

#include "cyber/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace mission_idle {

Stage::StageStatus MissionIdleStage::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  CHECK_NOTNULL(frame);

  // 1. Check if a valid routing response exists.
  // If routing is ready, finish this scenario to allow transition to
  // LaneFollow.
  const auto& routing = frame->local_view().routing;
  if (routing && !routing->road().empty()) {
    AINFO << "Routing received. Exiting Mission Idle Scenario.";
    return FinishScenario();
  }

  // 2. Log entry (throttle logs to avoid spamming)
  ADEBUG << "Mission Idle: Waiting for routing...";

  // 3. Generate a "Hold Position" trajectory.
  // Since we are idling, we don't necessarily need complex tasks.
  // We just need to ensure the control module receives a "stop" command.
  GenerateStopTrajectory(planning_init_point, frame);

  // 4. Return RUNNING to keep the scenario active.
  return Stage::RUNNING;
}

void MissionIdleStage::GenerateStopTrajectory(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  if (frame->mutable_reference_line_info()->empty()) {
    ADEBUG << "Mission Idle: no reference line available for stop trajectory.";
    return;
  }

  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    reference_line_info.SetDrivable(false);
  }

  DiscretizedTrajectory trajectory;
  constexpr double kStopTrajectoryStepSec = 0.1;
  constexpr std::size_t kStopTrajectoryPointCount = 8;
  for (std::size_t index = 0; index < kStopTrajectoryPointCount; ++index) {
    auto point = planning_init_point;
    point.set_relative_time(static_cast<double>(index) *
                            kStopTrajectoryStepSec);
    point.set_v(0.0);
    point.set_a(0.0);
    trajectory.AppendTrajectoryPoint(point);
  }

  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  reference_line_info.SetTrajectory(trajectory);
  reference_line_info.SetCost(0.0);
  reference_line_info.SetDrivable(true);
}

}  // namespace mission_idle
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
