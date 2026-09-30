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

#include "modules/local_planning/lane_follow_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>

#include "modules/local_planning/planning_context/environment_geometry.h"
#include "modules/local_planning/planning_context/trajectory_geometry.h"
#include "modules/local_planning/trajectory_continuity/trajectory_continuity.h"

namespace apollo {
namespace local_planning {
namespace {
using common::math::Vec2d;

common::Status Error(const std::string& message) {
  return common::Status(common::ErrorCode::PLANNING_ERROR, message);
}

double Angle(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

bool Finite(const Point2& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::abs(p.x) < 1e6 &&
         std::abs(p.y) < 1e6;
}

common::Status CheckConfig(const LaneFollowConfig& c) {
  for (double value : {c.wheelbase,
                       c.front_extent,
                       c.rear_extent,
                       c.half_width,
                       c.cruise_speed,
                       c.acceleration,
                       c.braking,
                       c.max_curvature,
                       c.max_lateral_acceleration,
                       c.lookahead,
                       c.horizon,
                       c.step,
                       c.execution_lifetime,
                       c.clearance,
                       c.max_position_error,
                       c.max_jerk,
                       c.max_curvature_rate,
                       c.stitch_position_tolerance,
                       c.stitch_heading_tolerance,
                       c.stitch_speed_tolerance}) {
    if (!std::isfinite(value) || value <= 0.0) {
      return Error("lane-follow configuration must be finite and positive");
    }
  }
  if (c.step > 0.1 || c.horizon < c.step * 2.0 || c.horizon / c.step > 1000.0 ||
      c.execution_lifetime > c.horizon || c.cruise_speed > 3.0 ||
      c.max_curvature > 0.2 ||
      std::abs(c.horizon / c.step - std::round(c.horizon / c.step)) > 1e-6) {
    return Error("configuration outside bounded low-speed simulation domain");
  }
  return common::Status::OK();
}

PlanningInputConfig InputBuilderConfig(const LaneFollowConfig& config) {
  PlanningInputConfig result;
  result.vehicle = {config.front_extent, config.rear_extent, config.half_width,
                    config.clearance};
  result.reference_rear = config.rear_extent + config.lookahead;
  result.reference_front = config.cruise_speed * config.horizon +
                           config.front_extent + config.lookahead;
  result.continuity_tolerance = config.max_position_error;
  result.minimum_prediction_horizon = config.horizon + config.step;
  return result;
}

common::Status CheckScene(const CycleInput& input, const LaneFollowConfig& c) {
  const auto& lane = input.corridor;
  if ((lane.speed_limit &&
       (!std::isfinite(*lane.speed_limit) || *lane.speed_limit < 0.0)) ||
      (lane.stop_station &&
       (!std::isfinite(*lane.stop_station) || *lane.stop_station < 0.0))) {
    return Error("invalid local road restriction");
  }
  if (lane.speed_limit && input.odometry.speed_mps > *lane.speed_limit + 1e-6)
    return Error("measured speed exceeds observed local speed restriction");
  if (!lane.confirmed_forward || lane.lane_id == 0 ||
      lane.centerline.size() < 2 || lane.centerline.size() > 2000 ||
      !std::isfinite(lane.half_width) || lane.half_width <= c.half_width ||
      !std::isfinite(lane.position_error_bound) ||
      lane.position_error_bound < 0.0 ||
      lane.position_error_bound > c.max_position_error) {
    return Error("invalid or unsupported corridor evidence");
  }
  double last_heading = 0.0;
  double total_turn = 0.0;
  for (size_t i = 0; i < lane.centerline.size(); ++i) {
    if (!Finite(lane.centerline[i])) {
      return Error("nonfinite corridor geometry");
    }
    if (i == 0) {
      continue;
    }
    const auto& a = lane.centerline[i - 1];
    const auto& b = lane.centerline[i];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const double heading = std::atan2(b.y - a.y, b.x - a.x);
    if (length < 0.05 || length > 2.0) {
      return Error("corridor sampling outside supported spacing");
    }
    if (i > 1) {
      const double turn = std::abs(Angle(heading - last_heading));
      total_turn += turn;
      if (turn / length > c.max_curvature || total_turn > 1.5) {
        return Error(
            "sharp or ambiguous corridor is outside lane-follow domain");
      }
    }
    last_heading = heading;
  }
  if (!input.occupancy.corridor_fully_observed ||
      !std::isfinite(input.occupancy.horizon) ||
      input.occupancy.horizon < input.planning_time -
                                    input.prediction.stamp.measurement_time +
                                    c.horizon + c.step ||
      input.occupancy.obstacles.size() > 1000) {
    return Error("missing obstacle coverage or prediction horizon");
  }
  auto environment_status = ValidateEnvironment(
      input.environment,
      input.graph.sequence == 0
          ? world_model::SceneSource{}
          : world_model::SceneSource{input.graph.frame_id, input.graph.clock_id,
                                     input.graph.epoch.producer_session,
                                     input.graph.epoch.generation,
                                     input.graph.sequence,
                                     input.graph.measurement_time,
                                     input.graph.publication_time,
                                     input.graph.valid_until,
                                     world_model::SceneHealth::HEALTHY},
      input.planning_time);
  if (!environment_status.ok()) return environment_status;
  if (input.environment.position_error > c.max_position_error)
    return Error("environment uncertainty exceeds planning limit");
  std::unordered_set<uint64_t> ids;
  for (const auto& object : input.occupancy.obstacles) {
    if (object.id == 0 || !ids.insert(object.id).second ||
        !Finite(object.position) || !Finite(object.velocity) ||
        !std::isfinite(object.radius) || object.radius <= 0.0 ||
        std::hypot(object.velocity.x, object.velocity.y) > 5.0) {
      return Error("invalid or unsupported obstacle prediction");
    }
  }
  if (input.odometry.speed_mps < -0.05 ||
      !Finite({input.odometry.x, input.odometry.y}) ||
      input.odometry.speed_mps > c.cruise_speed + 0.5 ||
      std::abs(input.planning_time - input.odometry.stamp.measurement_time) >
          1e-6) {
    return Error("requires current forward low-speed ODOM state");
  }
  return common::Status::OK();
}

// Three overlapping discs cover the entire configured rectangular footprint.
double CoverRadius(const LaneFollowConfig& c) {
  return std::hypot((c.front_extent + c.rear_extent) / 6.0, c.half_width);
}

double StopStation(const CycleInput& input, const LaneFollowConfig& c,
                   const CorridorPath& path, StopReason* reason) {
  double stop = path.length() - c.front_extent - c.clearance - CoverRadius(c);
  *reason = StopReason::CORRIDOR_END;
  if (input.corridor.stop_station &&
      *input.corridor.stop_station < path.length()) {
    stop = *input.corridor.stop_station - c.front_extent - c.clearance -
           CoverRadius(c);
    *reason = StopReason::OBSERVED_RULE;
  }
  const double age =
      input.planning_time - input.prediction.stamp.measurement_time;
  const auto ego = path.Project(Vec2d(input.odometry.x, input.odometry.y));
  for (const auto& object : input.occupancy.obstacles) {
    const Vec2d p(object.position.x + object.velocity.x * age,
                  object.position.y + object.velocity.y * age);
    const auto projection = path.Project(p);
    // Reserve the complete reachable disc, including lateral cut-ins.
    const double reach =
        std::hypot(object.velocity.x, object.velocity.y) * c.horizon;
    if (projection.distance <=
            input.corridor.half_width + object.radius + reach &&
        projection.s + object.radius + reach >= ego.s - c.rear_extent) {
      const double obstacle_stop = projection.s - object.radius - reach -
                                   c.front_extent - c.clearance -
                                   CoverRadius(c);
      if (obstacle_stop < stop) {
        stop = obstacle_stop;
        *reason = StopReason::OBSTACLE;
      }
    }
  }
  for (const auto& obstacle : input.environment.obstacles) {
    double obstacle_s = std::numeric_limits<double>::infinity();
    double obstacle_end_s = -std::numeric_limits<double>::infinity();
    double obstacle_l = std::numeric_limits<double>::infinity();
    for (const auto& point : obstacle.points) {
      const auto projection = path.Project(Vec2d(point.x, point.y));
      obstacle_s = std::min(obstacle_s, projection.s);
      obstacle_end_s = std::max(obstacle_end_s, projection.s);
      obstacle_l = std::min(obstacle_l, projection.distance);
    }
    if (obstacle_l <= input.corridor.half_width &&
        obstacle_end_s >= ego.s - c.rear_extent) {
      const double obstacle_stop = obstacle_s - c.front_extent - c.clearance -
                                   CoverRadius(c) -
                                   input.environment.position_error;
      if (obstacle_stop < stop) {
        stop = obstacle_stop;
        *reason = StopReason::OBSTACLE;
      }
    }
  }
  return stop;
}

}  // namespace

struct LaneFollowPlanner::Workspace {
  LocalTrajectory trajectory;
  TrajectoryContinuity continuity;
  double stop_s = 0.0;
  StopReason stop_reason = StopReason::CORRIDOR_END;
};

class LaneFollowPlanner::DecisionTask : public Task {
 public:
  DecisionTask(const LaneFollowConfig& config, Workspace* workspace)
      : c_(config), w_(workspace) {}
  std::string Name() const override { return "corridor_stop_decision"; }
  void Reset() override { w_->stop_s = 0.0; }
  common::Status Execute(const CycleInput& input) override {
    auto status = CheckConfig(c_);
    if (status.ok()) status = CheckScene(input, c_);
    if (!status.ok()) return status;
    w_->stop_s =
        StopStation(input, c_, CorridorPath(input.corridor), &w_->stop_reason);
    return common::Status::OK();
  }

 private:
  LaneFollowConfig c_;
  Workspace* w_;
};

class LaneFollowPlanner::GenerateTask : public Task {
 public:
  GenerateTask(const LaneFollowConfig& config, Workspace* workspace)
      : c_(config), w_(workspace) {}
  std::string Name() const override { return "lane_follow_rollout"; }
  void Reset() override {
    w_->trajectory = {};
    w_->continuity.Reset();
  }
  common::Status Execute(const CycleInput& input) override {
    const CorridorPath path(input.corridor);
    const double stop_s = w_->stop_s;
    TrajectoryPoint p;
    p.x = input.odometry.x;
    p.y = input.odometry.y;
    p.heading = input.odometry.heading;
    p.speed = std::max(0.0, input.odometry.speed_mps);
    auto actual_status = CheckLaneFootprint(input, c_, p);
    if (!actual_status.ok()) return actual_status;
    const double actual_remaining = stop_s - path.Project(Vec2d(p.x, p.y)).s;
    if (actual_remaining < 0.0 ||
        p.speed * p.speed / (2.0 * c_.braking) + p.speed * c_.step >
            actual_remaining) {
      return Error("actual ODOM state has insufficient distance to stop");
    }
    TrajectoryStart start;
    auto stitch_status = w_->continuity.Start(
        input,
        {c_.stitch_position_tolerance, c_.stitch_heading_tolerance,
         c_.stitch_speed_tolerance},
        &start);
    if (!stitch_status.ok()) return stitch_status;
    p = start.point;
    w_->trajectory.stitched =
        start.continuity == TrajectoryContinuityState::STITCHED;
    const auto projection = path.Project(Vec2d(p.x, p.y));
    if (std::abs(Angle(p.heading - projection.heading)) > 0.5) {
      return Error("ego heading inconsistent with forward corridor");
    }
    const int steps = static_cast<int>(std::round(c_.horizon / c_.step));
    for (int i = 0; i <= steps; ++i) {
      p.time = i * c_.step;
      const auto current = path.Project(Vec2d(p.x, p.y));
      const double remaining = stop_s - current.s;
      if (remaining < -1e-6 ||
          p.speed * p.speed / (2.0 * c_.braking) + p.speed * c_.step >
              remaining + 1e-6) {
        return Error("insufficient observed distance to stop: planning_time=" +
                     std::to_string(input.planning_time) +
                     " rollout_time=" + std::to_string(p.time) +
                     " remaining=" + std::to_string(remaining) +
                     " speed=" + std::to_string(p.speed) +
                     " acceleration=" + std::to_string(p.acceleration) +
                     " stop_s=" + std::to_string(stop_s) + " actual_speed=" +
                     std::to_string(input.odometry.speed_mps));
      }
      // The bumper stop boundary is longitudinal, not a lateral target.
      const Vec2d target = path.At(current.s + c_.lookahead);
      const double dx = target.x() - p.x;
      const double dy = target.y() - p.y;
      const double lateral =
          -std::sin(p.heading) * dx + std::cos(p.heading) * dy;
      const double distance_squared = dx * dx + dy * dy;
      const double requested_curvature =
          distance_squared > 0.01 ? 2.0 * lateral / distance_squared : 0.0;
      if (std::abs(requested_curvature) > c_.max_curvature) {
        return Error("required curvature exceeds configured limit");
      }
      const double control_dt =
          i == 0 && w_->trajectory.stitched ? 0.0 : c_.step;
      p.curvature = std::clamp(
          requested_curvature, p.curvature - c_.max_curvature_rate * control_dt,
          p.curvature + c_.max_curvature_rate * control_dt);
      const double desired =
          std::min({c_.cruise_speed,
                    input.corridor.speed_limit.value_or(c_.cruise_speed),
                    0.8 * std::max(0.0, remaining - c_.clearance),
                    std::sqrt(2.0 * c_.braking * 0.5 *
                              std::max(0.0, remaining - c_.clearance)),
                    std::sqrt(c_.max_lateral_acceleration /
                              std::max(1e-6, std::abs(p.curvature)))});
      const double requested_acceleration =
          std::clamp(1.5 * (desired - p.speed), -c_.braking, c_.acceleration);
      p.acceleration = std::clamp(requested_acceleration,
                                  p.acceleration - c_.max_jerk * control_dt,
                                  p.acceleration + c_.max_jerk * control_dt);
      w_->trajectory.points.push_back(p);
      const double next_speed =
          std::max(0.0, p.speed + p.acceleration * c_.step);
      const double distance = (p.speed + next_speed) * 0.5 * c_.step;
      const double midpoint_heading = p.heading + distance * p.curvature * 0.5;
      p.x += distance * std::cos(midpoint_heading);
      p.y += distance * std::sin(midpoint_heading);
      p.heading = Angle(p.heading + distance * p.curvature);
      p.speed = next_speed;
    }
    auto& trajectory = w_->trajectory;
    trajectory.stamp = input.odometry.stamp;
    trajectory.stamp.measurement_time = input.planning_time;
    trajectory.stamp.publication_time = input.planning_time;
    trajectory.stamp.valid_until =
        std::min({input.planning_time + c_.execution_lifetime,
                  input.odometry.stamp.valid_until, input.graph.valid_until,
                  input.prediction.stamp.valid_until});
    trajectory.graph_sequence = input.graph.sequence;
    trajectory.prediction_sequence = input.prediction.stamp.sequence;
    trajectory.lane_id = input.corridor.lane_id;
    trajectory.stop_reason = w_->stop_reason;
    const auto& first = trajectory.points.front();
    trajectory.behavior =
        first.speed < 0.02 && first.acceleration <= 0.0 ? BehaviorState::STOPPED
        : first.acceleration < -1e-6 ? BehaviorState::STOPPING
                                     : BehaviorState::FOLLOWING;
    return common::Status::OK();
  }

 private:
  LaneFollowConfig c_;
  Workspace* w_;
};

class LaneFollowPlanner::ValidateTask : public Task {
 public:
  ValidateTask(const LaneFollowConfig& config, Workspace* workspace)
      : c_(config), w_(workspace) {}
  std::string Name() const override {
    return "validate_lane_follow_trajectory";
  }
  void Reset() override {
    w_->trajectory = {};
    w_->continuity.Reset();
  }
  common::Status Execute(const CycleInput& input) override {
    auto status = w_->continuity.Accept(input, c_, w_->trajectory);
    if (!status.ok()) return status;
    w_->trajectory.valid = true;
    return common::Status::OK();
  }

 private:
  LaneFollowConfig c_;
  Workspace* w_;
};

LaneFollowPlanner::LaneFollowPlanner(const InputPolicy& policy,
                                     const LaneFollowConfig& config)
    : config_(config),
      workspace_(std::make_unique<Workspace>()),
      input_builder_(std::make_unique<PlanningInputBuilder>(
          policy, InputBuilderConfig(config))) {
  std::vector<std::unique_ptr<Task>> tasks;
  tasks.push_back(std::make_unique<DecisionTask>(config, workspace_.get()));
  tasks.push_back(std::make_unique<GenerateTask>(config, workspace_.get()));
  tasks.push_back(std::make_unique<ValidateTask>(config, workspace_.get()));
  scenario_ = std::make_unique<Scenario>(policy, std::move(tasks));
}

LaneFollowPlanner::~LaneFollowPlanner() = default;

common::Status LaneFollowPlanner::BeginEpoch(const OdometryInput& odometry,
                                             double now) {
  workspace_->trajectory = {};
  workspace_->continuity.Reset();
  entry_mode_ = EntryMode::NONE;
  auto status = CheckConfig(config_);
  if (!status.ok()) {
    scenario_->Disarm();
    input_builder_->Disarm();
    behavior_ = BehaviorState::INVALID;
    invalid_reason_ = status.error_message();
    return status;
  }
  auto scenario_status = scenario_->BeginEpoch(odometry, now);
  auto builder_status = input_builder_->BeginEpoch(odometry, now);
  status = scenario_status.ok() ? builder_status : scenario_status;
  if (!status.ok()) {
    scenario_->Disarm();
    input_builder_->Disarm();
  }
  behavior_ = status.ok() ? BehaviorState::IDLE : BehaviorState::INVALID;
  invalid_reason_ = status.ok() ? "" : status.error_message();
  return status;
}

common::Status LaneFollowPlanner::Plan(const CycleInput& input) {
  if (entry_mode_ == EntryMode::SCENE) {
    workspace_->trajectory = {};
    workspace_->continuity.Reset();
    scenario_->Invalidate();
    input_builder_->Invalidate();
    behavior_ = BehaviorState::INVALID;
    invalid_reason_ = "Plan and PlanScene cannot be mixed within one epoch";
    return Error(invalid_reason_);
  }
  entry_mode_ = EntryMode::DIRECT;
  input_builder_->Invalidate();
  return ExecuteCycle(input, false);
}

common::Status LaneFollowPlanner::ExecuteCycle(const CycleInput& input,
                                               bool admitted) {
  workspace_->trajectory = {};
  auto status =
      admitted ? scenario_->ProcessAdmitted(input) : scenario_->Process(input);
  if (!status.ok()) input_builder_->Invalidate();
  behavior_ =
      status.ok() ? workspace_->trajectory.behavior : BehaviorState::INVALID;
  invalid_reason_ = status.ok() ? "" : status.error_message();
  return status;
}

common::Status LaneFollowPlanner::PlanScene(
    const world_model::LocalScene& scene, const OdometryInput& odometry,
    const PredictionInput& prediction, const LocalOccupancy& occupancy,
    double now) {
  if (entry_mode_ == EntryMode::DIRECT) {
    workspace_->trajectory = {};
    workspace_->continuity.Reset();
    scenario_->Invalidate();
    input_builder_->Invalidate();
    behavior_ = BehaviorState::INVALID;
    invalid_reason_ = "Plan and PlanScene cannot be mixed within one epoch";
    return Error(invalid_reason_);
  }
  entry_mode_ = EntryMode::SCENE;
  auto status =
      input_builder_->Build(scene, odometry, prediction, occupancy, now);
  if (!status.ok()) {
    scenario_->Invalidate();
    behavior_ = BehaviorState::INVALID;
    invalid_reason_ = status.error_message();
    return status;
  }
  const auto* conditions = input_builder_->conditions();
  if (conditions == nullptr) {
    scenario_->Invalidate();
    behavior_ = BehaviorState::INVALID;
    invalid_reason_ = "planning input builder returned no conditions";
    return Error(invalid_reason_);
  }
  if (conditions->geometry.mode == world_model::SceneMode::AREA) {
    scenario_->Invalidate();
    behavior_ = BehaviorState::INVALID;
    invalid_reason_ =
        "lane-follow trajectory unavailable in AREA mode; drivable geometry "
        "remains valid";
    return Error(invalid_reason_);
  }
  status = ExecuteCycle(conditions->cycle, true);
  return status;
}

const LocalTrajectory& LaneFollowPlanner::trajectory() const {
  return workspace_->trajectory;
}

const ReferenceGeometry* LaneFollowPlanner::reference_geometry() const {
  const auto* conditions = input_builder_->conditions();
  return conditions && conditions->geometry.reference
             ? &*conditions->geometry.reference
             : nullptr;
}

const PlanningGeometry* LaneFollowPlanner::planning_geometry() const {
  const auto* conditions = input_builder_->conditions();
  return conditions ? &conditions->geometry : nullptr;
}

}  // namespace local_planning
}  // namespace apollo
