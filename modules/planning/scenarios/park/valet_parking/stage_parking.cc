/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/
#include "modules/planning/scenarios/park/valet_parking/stage_parking.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace valet_parking {

Stage::StageStatus StageParking::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  // Open space planning doesn't use planning_init_point from upstream because
  // of different stitching strategy
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  if (NextStage() == StageType::NO_STAGE) {
    frame->mutable_open_space_info()->set_destination_reached(true);
    AINFO << "Valet parking destination already reached.";
    return StageStatus::FINISHED;
  }
  if (frame->open_space_info().destination_reached()) {
    AINFO << "Valet parking destination reached before task execution.";
    return FinishScenario();
  }
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    if (frame->open_space_info().destination_reached()) {
      AINFO << "Valet parking destination reached during task execution.";
      return FinishScenario();
    }
    AERROR << "StageParking planning error";
    return StageStatus::ERROR;
  }
  if (frame->open_space_info().destination_reached()) {
    AINFO << "Valet parking destination reached.";
    return FinishScenario();
  }
  return StageStatus::RUNNING;
}

Stage::StageStatus StageParking::FinishStage() { return Stage::FINISHED; }

}  // namespace valet_parking
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
