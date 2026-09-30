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

#include "modules/local_planning/planning_context/planning_geometry.h"

#include <cmath>
#include <utility>

#include "modules/local_planning/planning_context/environment_geometry.h"

namespace apollo {
namespace local_planning {
namespace {

common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "planning geometry: " + reason);
}

}  // namespace

common::Status BuildPlanningGeometry(
    const world_model::LocalScene& scene, const OdometryInput& ego, double now,
    double rear, double front, double continuity_tolerance,
    const ReferenceGeometry* previous_reference, PlanningGeometry* output) {
  if (output == nullptr) return Error("output required");
  *output = {};
  const auto& source = scene.source;
  if (scene.source.health != world_model::SceneHealth::HEALTHY ||
      scene.mode == world_model::SceneMode::INVALID ||
      scene.reference != world_model::VehicleReference::REAR_AXLE ||
      !scene.capabilities.drivable_region || !std::isfinite(now) ||
      !scene.invalid_reason.empty() || source.frame_id.empty() ||
      source.clock_id.empty() || source.session.empty() ||
      source.generation == 0 || source.sequence == 0 ||
      !std::isfinite(source.measurement_time) ||
      !std::isfinite(source.publication_time) ||
      !std::isfinite(source.valid_until) || source.measurement_time < 0.0 ||
      source.measurement_time > source.publication_time ||
      now < source.publication_time || now >= source.valid_until ||
      source.frame_id != ego.stamp.frame_id ||
      source.clock_id != ego.stamp.clock_id ||
      source.session != ego.stamp.epoch.producer_session ||
      source.generation != ego.stamp.epoch.generation ||
      ego.stamp.health != InputHealth::HEALTHY || ego.stamp.sequence == 0 ||
      ego.stamp.epoch.generation == 0 ||
      ego.stamp.epoch.producer_session.empty() ||
      !std::isfinite(ego.stamp.measurement_time) ||
      !std::isfinite(ego.stamp.publication_time) ||
      !std::isfinite(ego.stamp.valid_until) ||
      ego.stamp.measurement_time < 0.0 ||
      ego.stamp.measurement_time > ego.stamp.publication_time ||
      ego.stamp.publication_time > now || now >= ego.stamp.valid_until ||
      std::abs(now - ego.stamp.measurement_time) > 1e-6 ||
      !std::isfinite(ego.x) || !std::isfinite(ego.y) ||
      !std::isfinite(ego.heading) || !std::isfinite(ego.speed_mps)) {
    return Error("invalid scene lifecycle or drivable-region capability");
  }
  auto status = ValidateEnvironment(scene.environment, scene.source, now);
  if (!status.ok()) return status;
  output->source = scene.source;
  output->mode = scene.mode;
  output->capabilities = scene.capabilities;
  output->environment = scene.environment;
  output->mode_reason = scene.mode_reason;
  if (scene.mode == world_model::SceneMode::AREA) {
    if (scene.capabilities.lane_follow || !scene.boundaries.empty() ||
        !scene.lanes.empty() || !scene.edges.empty() || !scene.rules.empty() ||
        scene.navigation || scene.capabilities.lane_change ||
        scene.capabilities.intersection) {
      *output = {};
      return Error("AREA mode contains lane-only executable semantics");
    }
    return common::Status::OK();
  }
  if (scene.mode != world_model::SceneMode::LANE ||
      !scene.capabilities.lane_follow) {
    *output = {};
    return Error("unsupported scene mode");
  }
  ReferenceGeometry reference;
  status =
      BuildReferenceGeometry(scene, ego, now, rear, front, continuity_tolerance,
                             previous_reference, &reference);
  if (!status.ok()) {
    *output = {};
    return status;
  }
  output->reference = std::move(reference);
  return common::Status::OK();
}

}  // namespace local_planning
}  // namespace apollo
