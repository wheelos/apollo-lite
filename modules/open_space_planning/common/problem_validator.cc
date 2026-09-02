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

#include "modules/open_space_planning/common/problem_validator.h"

#include <cmath>

namespace apollo {
namespace open_space_planning {

Status ProblemValidator::Validate(const PlanningProblem& problem) {
  if (problem.grid_map == nullptr) {
    return {StatusCode::kInvalidInput, "grid map is missing"};
  }
  const auto& map = *problem.grid_map;
  if (map.frame_id.empty()) {
    return {StatusCode::kInvalidInput, "grid map frame is empty"};
  }
  if (!std::isfinite(map.resolution) || map.resolution <= 0.0) {
    return {StatusCode::kInvalidInput, "grid map resolution is invalid"};
  }
  if (map.width == 0 || map.height == 0) {
    return {StatusCode::kInvalidInput, "grid map dimensions are empty"};
  }
  if (map.cell_state.size() != map.cell_count()) {
    return {StatusCode::kInvalidInput, "grid map cell-state size mismatch"};
  }
  if (!map.traversal_cost.empty() &&
      map.traversal_cost.size() != map.cell_count()) {
    return {StatusCode::kInvalidInput, "grid map cost size mismatch"};
  }
  if (!map.semantic_flags.empty() &&
      map.semantic_flags.size() != map.cell_count()) {
    return {StatusCode::kInvalidInput, "grid map semantics size mismatch"};
  }
  if (problem.vehicle.wheel_base <= 0.0 ||
      problem.vehicle.maximum_curvature <= 0.0) {
    return {StatusCode::kInvalidInput, "vehicle model is invalid"};
  }
  return Status::Ok();
}

}  // namespace open_space_planning
}  // namespace apollo
