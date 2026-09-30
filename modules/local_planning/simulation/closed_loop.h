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

#include <string>
#include <vector>

#include "modules/local_planning/lane_follow_planner.h"

namespace apollo {
namespace local_planning {

// Explicit simulation fixture parameters; not production defaults.
InputPolicy SimulationInputPolicy();
LaneFollowConfig SimulationPlannerConfig();

struct SimulationOptions {
  std::string backend = "kinematic";
  std::string scenario = "straight";
  std::string model_path = "modules/simulation/model/ackermann_vehicle.xml";
};

struct SimulationSample {
  double time = 0.0;
  double truth_x = 0.0;
  double truth_y = 0.0;
  double odom_x = 0.0;
  double odom_y = 0.0;
  double speed = 0.0;
  double lateral_error = 0.0;
  bool trajectory_valid = false;
};

struct SimulationReport {
  bool passed = false;
  int rejected_cycles = 0;
  int stitched_cycles = 0;
  int lane_updates = 0;
  double progress = 0.0;
  double max_lateral_error = 0.0;
  double final_speed = 0.0;
  std::string first_rejection;
  std::vector<SimulationSample> samples;
  std::vector<Point2> centerline;
};

common::Status RunClosedLoop(const SimulationOptions& options,
                             SimulationReport* report);
common::Status WriteSimulationArtifacts(const std::string& prefix,
                                        const SimulationReport& report);

}  // namespace local_planning
}  // namespace apollo
