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

#include "modules/open_space_planning/open_space_planning_component.h"

#include <memory>
#include <string>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/open_space_planning/route/skeleton_corridor_route_planner.h"
#include "modules/open_space_planning/safety/default_fallback_planner.h"
#include "modules/open_space_planning/safety/default_trajectory_validator.h"
#include "modules/open_space_planning/trajectory/open_space_trajectory_planner.h"

namespace apollo {
namespace open_space_planning {
namespace {

constexpr char kGridMapTopic[] = "/apollo/open_space/grid_map";
constexpr char kVehicleModelTopic[] = "/apollo/open_space/vehicle_model";
constexpr char kRoutingRequestTopic[] = "/apollo/routing_request";
constexpr char kPlanningResultTopic[] = "/apollo/open_space/planning_result";

Pose2d ToPose(double x, double y, double heading) { return {x, y, heading}; }

}  // namespace

bool OpenSpacePlanningComponent::Init() {
  planner_ = std::make_unique<OpenSpacePlanner>(
      OpenSpacePlannerConfig{},
      std::make_unique<SkeletonCorridorRoutePlanner>(),
      std::make_unique<OpenSpaceTrajectoryPlanner>(),
      std::make_unique<DefaultTrajectoryValidator>(),
      std::make_unique<DefaultFallbackPlanner>());

  grid_map_reader_ = node_->CreateReader<proto::GridMap>(
      kGridMapTopic, [this](const std::shared_ptr<proto::GridMap>& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        grid_map_.CopyFrom(*message);
        has_grid_map_ = true;
      });
  vehicle_model_reader_ = node_->CreateReader<proto::VehicleModel>(
      kVehicleModelTopic,
      [this](const std::shared_ptr<proto::VehicleModel>& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        vehicle_model_.CopyFrom(*message);
        has_vehicle_model_ = true;
      });
  routing_reader_ = node_->CreateReader<routing::RoutingRequest>(
      kRoutingRequestTopic,
      [this](const std::shared_ptr<routing::RoutingRequest>& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        routing_request_.CopyFrom(*message);
        has_routing_request_ = true;
      });
  result_writer_ =
      node_->CreateWriter<proto::OpenSpacePlanningResult>(kPlanningResultTopic);
  trajectory_writer_ = node_->CreateWriter<planning::ADCTrajectory>(
      FLAGS_planning_trajectory_topic);
  return grid_map_reader_ != nullptr && vehicle_model_reader_ != nullptr &&
         routing_reader_ != nullptr && result_writer_ != nullptr &&
         trajectory_writer_ != nullptr;
}

bool OpenSpacePlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>& prediction,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>& localization) {
  if (prediction == nullptr || chassis == nullptr || localization == nullptr) {
    AERROR << "open-space planning received a null input";
    return false;
  }

  PlanningProblem problem;
  if (!BuildProblem(*prediction, *chassis, *localization, &problem)) {
    return false;
  }

  PlanningResult result;
  const Status status = planner_->Plan(problem, &result);
  Publish(result, status);
  return status.ok();
}

bool OpenSpacePlanningComponent::BuildProblem(
    const prediction::PredictionObstacles& prediction,
    const canbus::Chassis& chassis,
    const localization::LocalizationEstimate& localization,
    PlanningProblem* problem) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_grid_map_ || !has_vehicle_model_ || !has_routing_request_ ||
      routing_request_.waypoint_size() == 0 || !localization.has_pose()) {
    AWARN << "open-space planning inputs are not ready";
    return false;
  }

  auto grid = std::make_shared<GridMap>();
  grid->frame_id = grid_map_.frame_id();
  grid->timestamp_sec = grid_map_.timestamp_sec();
  grid->origin = ToPose(grid_map_.origin_x(), grid_map_.origin_y(),
                        grid_map_.origin_heading());
  grid->resolution = grid_map_.resolution();
  grid->width = grid_map_.width();
  grid->height = grid_map_.height();
  grid->revision = grid_map_.revision();
  grid->cell_state.reserve(grid_map_.cell_state_size());
  for (const auto state : grid_map_.cell_state()) {
    grid->cell_state.push_back(static_cast<CellState>(state));
  }
  for (const auto cost : grid_map_.traversal_cost()) {
    grid->traversal_cost.push_back(static_cast<std::uint8_t>(cost));
  }
  for (const auto flags : grid_map_.semantic_flags()) {
    grid->semantic_flags.push_back(flags);
  }

  common::VehicleStateProvider vehicle_state_provider;
  if (!vehicle_state_provider.Update(localization, chassis).ok()) {
    AERROR << "failed to construct vehicle state";
    return false;
  }

  problem->grid_map = std::move(grid);
  problem->start.pose =
      ToPose(vehicle_state_provider.x(), vehicle_state_provider.y(),
             vehicle_state_provider.heading());
  problem->start.longitudinal_velocity =
      vehicle_state_provider.linear_velocity();
  problem->start.longitudinal_acceleration =
      vehicle_state_provider.linear_acceleration();
  problem->start.steering_angle = vehicle_state_provider.steering_percentage();
  problem->start.gear =
      vehicle_state_provider.gear() == canbus::Chassis::GEAR_REVERSE
          ? Gear::kReverse
          : Gear::kDrive;
  problem->start.timestamp_sec = vehicle_state_provider.timestamp();
  problem->vehicle.wheel_base = vehicle_model_.wheel_base();
  problem->vehicle.front_edge_to_center = vehicle_model_.front_edge_to_center();
  problem->vehicle.back_edge_to_center = vehicle_model_.back_edge_to_center();
  problem->vehicle.left_edge_to_center = vehicle_model_.left_edge_to_center();
  problem->vehicle.right_edge_to_center = vehicle_model_.right_edge_to_center();
  problem->vehicle.maximum_steering_angle =
      vehicle_model_.maximum_steering_angle();
  problem->vehicle.maximum_curvature = vehicle_model_.maximum_curvature();

  const auto& waypoint =
      routing_request_.waypoint(routing_request_.waypoint_size() - 1);
  if (!waypoint.has_pose()) {
    AERROR << "routing request final waypoint has no pose";
    return false;
  }
  problem->goal.pose =
      ToPose(waypoint.pose().x(), waypoint.pose().y(), waypoint.heading());
  problem->goal.allow_reverse = true;
  problem->planning_timestamp_sec = prediction.has_header()
                                        ? prediction.header().timestamp_sec()
                                        : vehicle_state_provider.timestamp();

  for (const auto& obstacle : prediction.prediction_obstacle()) {
    if (!obstacle.has_perception_obstacle()) {
      continue;
    }
    const auto& perception = obstacle.perception_obstacle();
    DynamicObstacle converted;
    converted.id = std::to_string(perception.id());
    for (const auto& point : perception.polygon_point()) {
      converted.footprint.push_back(ToPose(point.x(), point.y(), 0.0));
    }
    for (const auto& prediction_trajectory : obstacle.trajectory()) {
      for (const auto& point : prediction_trajectory.trajectory_point()) {
        if (!point.has_path_point()) {
          continue;
        }
        const auto& path_point = point.path_point();
        converted.prediction.push_back(
            {point.relative_time(),
             ToPose(path_point.x(), path_point.y(), path_point.theta()),
             point.v()});
      }
    }
    problem->dynamic_obstacles.push_back(std::move(converted));
  }
  return true;
}

void OpenSpacePlanningComponent::Publish(const PlanningResult& result,
                                         const Status& status) {
  proto::OpenSpacePlanningResult output;
  output.set_outcome(result.outcome == PlanningOutcome::kFallbackTrajectory
                         ? proto::OpenSpacePlanningResult::FALLBACK_TRAJECTORY
                         : proto::OpenSpacePlanningResult::TRAJECTORY);
  output.set_route_candidate_id(result.active_route.id);
  output.set_route_topology_id(result.active_route.topology_id);
  for (const auto& point : result.active_route.skeleton) {
    auto* route_point = output.add_route_point();
    route_point->set_x(point.pose.x);
    route_point->set_y(point.pose.y);
    route_point->set_heading(point.pose.heading);
    route_point->set_s(point.s);
    route_point->set_curvature(point.curvature);
    route_point->set_curvature_derivative(point.curvature_derivative);
    route_point->set_gear(static_cast<int>(point.gear));
  }
  for (const auto& point : result.trajectory.points) {
    auto* trajectory_point = output.add_trajectory_point();
    auto* path_point = trajectory_point->mutable_path_point();
    path_point->set_x(point.pose.x);
    path_point->set_y(point.pose.y);
    path_point->set_heading(point.pose.heading);
    path_point->set_s(point.s);
    path_point->set_curvature(point.curvature);
    path_point->set_curvature_derivative(point.curvature_derivative);
    path_point->set_gear(static_cast<int>(point.gear));
    trajectory_point->set_velocity(point.velocity);
    trajectory_point->set_acceleration(point.acceleration);
    trajectory_point->set_relative_time(point.relative_time);
  }
  output.set_validation_safe(result.validation.safe);
  for (const auto& issue : result.validation.issues) {
    auto* output_issue = output.add_validation_issue();
    output_issue->set_rule(issue.rule);
    output_issue->set_detail(issue.detail);
    output_issue->set_trajectory_point_index(issue.trajectory_point_index);
  }
  output.set_status_code(static_cast<int>(status.code()));
  output.set_status_message(status.message());
  result_writer_->Write(output);

  planning::ADCTrajectory trajectory;
  trajectory.set_trajectory_type(result.outcome ==
                                         PlanningOutcome::kFallbackTrajectory
                                     ? planning::ADCTrajectory::PATH_FALLBACK
                                     : planning::ADCTrajectory::NORMAL);
  for (const auto& point : result.trajectory.points) {
    auto* output_point = trajectory.add_trajectory_point();
    auto* path_point = output_point->mutable_path_point();
    path_point->set_x(point.pose.x);
    path_point->set_y(point.pose.y);
    path_point->set_theta(point.pose.heading);
    path_point->set_s(point.s);
    path_point->set_kappa(point.curvature);
    path_point->set_dkappa(point.curvature_derivative);
    output_point->set_v(point.velocity);
    output_point->set_a(point.acceleration);
    output_point->set_relative_time(point.relative_time);
  }
  trajectory.set_total_path_time(
      result.trajectory.points.empty()
          ? 0.0
          : result.trajectory.points.back().relative_time);
  trajectory.set_total_path_length(result.trajectory.points.empty()
                                       ? 0.0
                                       : result.trajectory.points.back().s);
  trajectory_writer_->Write(trajectory);
}

}  // namespace open_space_planning
}  // namespace apollo
