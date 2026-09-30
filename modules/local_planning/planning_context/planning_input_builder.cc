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

#include "modules/local_planning/planning_context/planning_input_builder.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <utility>

namespace apollo {
namespace local_planning {
namespace {

common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "planning input: " + reason);
}

SourceStamp GraphStamp(const world_model::SceneSource& source) {
  return {source.frame_id,
          source.clock_id,
          {source.session, source.generation},
          source.sequence,
          source.measurement_time,
          source.publication_time,
          source.valid_until,
          source.health == world_model::SceneHealth::HEALTHY
              ? InputHealth::HEALTHY
              : InputHealth::INVALID};
}

bool SameGraphVersion(const world_model::SceneSource& a,
                      const world_model::SceneSource& b) {
  return a.session == b.session && a.generation == b.generation &&
         a.sequence == b.sequence;
}

}  // namespace

PlanningInputBuilder::PlanningInputBuilder(const InputPolicy& policy,
                                           const PlanningInputConfig& config)
    : gate_(policy), config_(config) {}

common::Status PlanningInputBuilder::ValidateConfig() const {
  const auto& v = config_.vehicle;
  if (!std::isfinite(v.front) || v.front <= 0.0 || !std::isfinite(v.rear) ||
      v.rear < 0.0 || !std::isfinite(v.half_width) || v.half_width <= 0.0 ||
      !std::isfinite(v.clearance) || v.clearance < 0.0 ||
      !std::isfinite(config_.reference_rear) ||
      config_.reference_rear < v.rear ||
      !std::isfinite(config_.reference_front) ||
      config_.reference_front <= 0.0 ||
      !std::isfinite(config_.continuity_tolerance) ||
      config_.continuity_tolerance < 0.0 ||
      !std::isfinite(config_.minimum_prediction_horizon) ||
      config_.minimum_prediction_horizon <= 0.0) {
    return Error(
        "invalid vehicle envelope, geometry extent or prediction "
        "coverage requirement");
  }
  return common::Status::OK();
}

void PlanningInputBuilder::ClearCycleState() {
  previous_reference_.reset();
  previous_mode_.reset();
  conditions_.reset();
  continuity_reset_reason_ = ContinuityReason::NO_PREVIOUS_REFERENCE;
}

void PlanningInputBuilder::Invalidate() { ClearCycleState(); }

void PlanningInputBuilder::Disarm() {
  ClearCycleState();
  gate_.Disarm();
}

common::Status PlanningInputBuilder::BeginEpoch(const OdometryInput& odometry,
                                                double now) {
  const bool had_reference_history =
      previous_reference_.has_value() || previous_mode_.has_value();
  ClearCycleState();
  auto status = ValidateConfig();
  if (!status.ok()) {
    gate_.Disarm();
    return status;
  }
  status = gate_.BeginEpoch(odometry, now);
  if (status.ok() && had_reference_history) {
    continuity_reset_reason_ = ContinuityReason::ODOM_EPOCH_CHANGED;
  }
  return status;
}

common::Status PlanningInputBuilder::ValidateOccupancy(
    const PredictionInput& prediction, const LocalOccupancy& occupancy,
    world_model::SceneMode mode, double now) const {
  const bool coverage_declared =
      mode == world_model::SceneMode::LANE
          ? occupancy.corridor_fully_observed
          : mode == world_model::SceneMode::AREA &&
                occupancy.drivable_region_fully_observed;
  const double coverage_end =
      prediction.stamp.measurement_time + occupancy.horizon;
  const double required_end = now + config_.minimum_prediction_horizon;
  if (!coverage_declared || !std::isfinite(occupancy.horizon) ||
      occupancy.horizon <= 0.0 || !std::isfinite(coverage_end) ||
      !std::isfinite(required_end) || coverage_end < required_end ||
      occupancy.obstacles.size() > 1000) {
    return Error(
        "dynamic occupancy is missing or does not cover the required "
        "planning horizon");
  }
  std::unordered_set<uint64_t> ids;
  for (const auto& obstacle : occupancy.obstacles) {
    if (obstacle.id == 0 || !ids.insert(obstacle.id).second ||
        !std::isfinite(obstacle.position.x) ||
        !std::isfinite(obstacle.position.y) ||
        !std::isfinite(obstacle.velocity.x) ||
        !std::isfinite(obstacle.velocity.y) ||
        !std::isfinite(obstacle.radius) || obstacle.radius <= 0.0) {
      return Error("invalid dynamic occupancy element");
    }
  }
  return common::Status::OK();
}

common::Status PlanningInputBuilder::Build(const world_model::LocalScene& scene,
                                           const OdometryInput& odometry,
                                           const PredictionInput& prediction,
                                           const LocalOccupancy& occupancy,
                                           double now) {
  conditions_.reset();
  auto fail = [this](const common::Status& status) {
    previous_reference_.reset();
    previous_mode_.reset();
    continuity_reset_reason_ = ContinuityReason::NO_PREVIOUS_REFERENCE;
    return status;
  };

  auto status = ValidateConfig();
  if (!status.ok()) return fail(status);
  if (!std::isfinite(now) ||
      std::abs(now - odometry.stamp.measurement_time) > 1e-6) {
    return fail(Error("cycle requires current measured ODOM"));
  }

  CycleInput cycle;
  cycle.planning_time = now;
  cycle.odometry = odometry;
  cycle.graph = GraphStamp(scene.source);
  cycle.prediction = prediction;
  cycle.environment = scene.environment;
  cycle.occupancy = occupancy;

  // Admit source data, never a reference that changes with ego position.
  // Full-scene immutability is checked separately from the derived corridor.
  if (accepted_scene_ &&
      SameGraphVersion(accepted_scene_->source, scene.source) &&
      !(scene == *accepted_scene_)) {
    return fail(Error("accepted full-scene graph version mutated"));
  }
  status = gate_.Admit(cycle);
  if (!status.ok()) return fail(status);
  accepted_scene_ = scene;

  status = ValidateOccupancy(prediction, occupancy, scene.mode, now);
  if (!status.ok()) return fail(status);
  status = ValidateEnvironment(scene.environment, scene.source, now);
  if (!status.ok()) return fail(status);

  const ReferenceGeometry* previous = nullptr;
  if (scene.mode == world_model::SceneMode::LANE && previous_mode_ &&
      *previous_mode_ == world_model::SceneMode::LANE && previous_reference_) {
    previous = &*previous_reference_;
  }
  PlanningGeometry geometry;
  status = BuildPlanningGeometry(
      scene, odometry, now, config_.reference_rear, config_.reference_front,
      config_.continuity_tolerance, previous, &geometry);
  if (!status.ok()) return fail(status);
  if (geometry.mode == world_model::SceneMode::LANE && geometry.reference &&
      !previous) {
    if (continuity_reset_reason_ == ContinuityReason::ODOM_EPOCH_CHANGED) {
      geometry.reference->continuity.reason =
          ContinuityReason::ODOM_EPOCH_CHANGED;
      geometry.reference->continuity.detail = "ODOM epoch transition";
    } else if (previous_mode_ == world_model::SceneMode::AREA) {
      geometry.reference->continuity.reason = ContinuityReason::MODE_CHANGED;
      geometry.reference->continuity.detail =
          "lane continuity reset after AREA mode";
    }
  }

  const EnvironmentPose ego_pose{odometry.x, odometry.y, odometry.heading};
  status = CheckEnvironmentSweep(scene.environment, config_.vehicle, ego_pose,
                                 ego_pose);
  if (!status.ok()) return fail(status);
  if (geometry.mode == world_model::SceneMode::LANE) {
    if (!geometry.reference)
      return fail(Error("LANE geometry lacks reference"));
    status = CheckCorridorFootprint(geometry.reference->corridor,
                                    config_.vehicle, ego_pose);
    if (!status.ok()) return fail(status);
    cycle.corridor = geometry.reference->corridor;
  }

  PlanningInputConditions result;
  result.cycle = std::move(cycle);
  result.geometry = std::move(geometry);
  result.measured_start = {odometry.stamp,   now,
                           odometry.x,       odometry.y,
                           odometry.heading, odometry.speed_mps};
  result.dynamic_occupancy = {
      prediction.stamp, prediction.graph_sequence, scene.mode,
      prediction.stamp.measurement_time,
      prediction.stamp.measurement_time + occupancy.horizon};
  result.vehicle = config_.vehicle;
  result.valid_until = std::min(
      {odometry.stamp.valid_until, scene.source.valid_until,
       prediction.stamp.valid_until, scene.environment.source.valid_until});
  result.executable_conditions = true;
  conditions_ = std::move(result);
  previous_mode_ = scene.mode;
  continuity_reset_reason_ = ContinuityReason::NO_PREVIOUS_REFERENCE;
  if (scene.mode == world_model::SceneMode::LANE) {
    previous_reference_ = conditions_->geometry.reference;
  } else {
    previous_reference_.reset();
  }
  return common::Status::OK();
}

const PlanningInputConditions* PlanningInputBuilder::conditions() const {
  return conditions_ ? &*conditions_ : nullptr;
}

}  // namespace local_planning
}  // namespace apollo
