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

#include <optional>

#include "modules/local_planning/planning_context/environment_geometry.h"
#include "modules/local_planning/planning_context/input_gate.h"
#include "modules/local_planning/planning_context/planning_geometry.h"

namespace apollo {
namespace local_planning {

struct PlanningInputConfig {
  VehicleEnvelope vehicle;
  double reference_rear = 0.0;
  double reference_front = 0.0;
  double continuity_tolerance = 0.0;
  double minimum_prediction_horizon = 0.0;
};

struct PlanningStartState {
  SourceStamp odometry_stamp;
  double measured_at = 0.0;
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed_mps = 0.0;
};

struct DynamicOccupancyBinding {
  SourceStamp prediction_stamp;
  uint64_t graph_sequence = 0;
  world_model::SceneMode scope = world_model::SceneMode::INVALID;
  double coverage_start = 0.0;
  double coverage_end = 0.0;
};

struct PlanningInputConditions {
  CycleInput cycle;
  PlanningGeometry geometry;
  PlanningStartState measured_start;
  DynamicOccupancyBinding dynamic_occupancy;
  VehicleEnvelope vehicle;
  double valid_until = 0.0;
  bool executable_conditions = false;
};

class PlanningInputBuilder {
 public:
  PlanningInputBuilder(const InputPolicy& policy,
                       const PlanningInputConfig& config);

  common::Status BeginEpoch(const OdometryInput& odometry, double now);
  void Disarm();
  common::Status Build(const world_model::LocalScene& scene,
                       const OdometryInput& odometry,
                       const PredictionInput& prediction,
                       const LocalOccupancy& occupancy, double now);
  void Invalidate();

  const PlanningInputConditions* conditions() const;

 private:
  common::Status ValidateConfig() const;
  common::Status ValidateOccupancy(const PredictionInput& prediction,
                                   const LocalOccupancy& occupancy,
                                   world_model::SceneMode mode,
                                   double now) const;
  void ClearCycleState();

  InputGate gate_;
  PlanningInputConfig config_;
  std::optional<world_model::LocalScene> accepted_scene_;
  std::optional<ReferenceGeometry> previous_reference_;
  std::optional<world_model::SceneMode> previous_mode_;
  std::optional<PlanningInputConditions> conditions_;
  ContinuityReason continuity_reset_reason_ =
      ContinuityReason::NO_PREVIOUS_REFERENCE;
};

}  // namespace local_planning
}  // namespace apollo
