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

#include "modules/local_planning/simulation/closed_loop.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

#include "modules/simulation/core/simulation_engine.h"
#include "modules/world_model/local_map/local_scene_builder.h"

namespace apollo {
namespace local_planning {
namespace {
constexpr double kDt = 0.02;
constexpr double kCurveRadius = 60.0;
constexpr double kRoadEnd = 45.0;
constexpr double kHalfWidth = 2.5;
constexpr double kObstacleX = 25.0;

common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR, reason);
}

Point2 WorldToObservationToOdom(const Point2& world,
                                const simulation::VehicleState& truth,
                                const OdometryInput& odom) {
  const double dx = world.x - truth.x;
  const double dy = world.y - truth.y;
  const double body_x = std::cos(truth.yaw) * dx + std::sin(truth.yaw) * dy;
  const double body_y = -std::sin(truth.yaw) * dx + std::cos(truth.yaw) * dy;
  return {odom.x + std::cos(odom.heading) * body_x -
              std::sin(odom.heading) * body_y,
          odom.y + std::sin(odom.heading) * body_x +
              std::cos(odom.heading) * body_y};
}

// Only body motion feeds this integrator. No simulator pose enters ODOM.
void IntegrateMotion(double vx, double vy, double yaw_rate,
                     OdometryInput* odom) {
  const double middle_heading = odom->heading + yaw_rate * kDt * 0.5;
  odom->x +=
      (vx * std::cos(middle_heading) - vy * std::sin(middle_heading)) * kDt;
  odom->y +=
      (vx * std::sin(middle_heading) + vy * std::cos(middle_heading)) * kDt;
  odom->heading = std::atan2(std::sin(odom->heading + yaw_rate * kDt),
                             std::cos(odom->heading + yaw_rate * kDt));
}

SourceStamp Stamp(double now, uint64_t sequence) {
  SourceStamp stamp;
  stamp.frame_id = "odom";
  stamp.clock_id = "simulation";
  stamp.epoch = {"local-planning-simulation", 1};
  stamp.sequence = sequence;
  stamp.measurement_time = now;
  stamp.publication_time = now;
  stamp.valid_until = now + 0.4;
  stamp.health = InputHealth::HEALTHY;
  return stamp;
}

world_model::SceneSource SceneSource(const SourceStamp& s,
                                     const std::string& frame) {
  return {frame,
          s.clock_id,
          s.epoch.producer_session,
          s.epoch.generation,
          s.sequence,
          s.measurement_time,
          s.publication_time,
          s.valid_until,
          s.health == InputHealth::HEALTHY ? world_model::SceneHealth::HEALTHY
                                           : world_model::SceneHealth::INVALID};
}

common::Status Track(const LocalTrajectory& trajectory, const CycleInput& input,
                     double now, const OdometryInput& odom,
                     const LaneFollowConfig& config, double* speed_integral,
                     simulation::VehicleCommand* command) {
  if (!trajectory.valid || trajectory.points.size() < 2 ||
      trajectory.stamp.frame_id != odom.stamp.frame_id ||
      trajectory.stamp.clock_id != odom.stamp.clock_id ||
      !(trajectory.stamp.epoch == odom.stamp.epoch) ||
      trajectory.graph_sequence != input.graph.sequence ||
      trajectory.prediction_sequence != input.prediction.stamp.sequence ||
      now < trajectory.stamp.measurement_time ||
      now >= trajectory.stamp.valid_until) {
    return Error(
        "simulation tracker rejected invalid trajectory tags/deadline");
  }
  const double elapsed = now - trajectory.stamp.measurement_time;
  const auto& points = trajectory.points;
  const auto found = std::upper_bound(
      points.begin(), points.end(), elapsed,
      [](double time, const TrajectoryPoint& p) { return time < p.time; });
  const auto& p = found == points.begin() ? points.front() : *(found - 1);
  const double dt = elapsed - p.time;
  const double target_speed = p.speed + p.acceleration * dt;
  const double speed_error = target_speed - odom.speed_mps;
  const double station_error = (p.x - odom.x) * std::cos(p.heading) +
                               (p.y - odom.y) * std::sin(p.heading) +
                               p.speed * dt + 0.5 * p.acceleration * dt * dt;
  *speed_integral = std::clamp(*speed_integral + speed_error * kDt,
                               -config.acceleration, config.acceleration);
  command->front_steering_rad = std::atan(config.wheelbase * p.curvature);
  command->target_acceleration_mps2 =
      std::clamp(p.acceleration + 1.5 * speed_error + 0.8 * station_error +
                     *speed_integral,
                 -config.braking, config.acceleration + 0.15);
  if (target_speed < 0.02 && p.acceleration <= 0.0) {
    *speed_integral = 0.0;
    command->target_acceleration_mps2 = 0.0;
    command->brake = 0.4;
  }
  return common::Status::OK();
}
}  // namespace

InputPolicy SimulationInputPolicy() {
  return {"odom", "simulation", 0.1, 0.6, 0.3, 0.02};
}

LaneFollowConfig SimulationPlannerConfig() {
  LaneFollowConfig c;
  c.wheelbase = 2.8448;
  c.front_extent = 3.8;
  c.rear_extent = 1.0;
  c.half_width = 1.0;
  c.cruise_speed = 2.0;
  c.acceleration = 0.7;
  c.braking = 1.2;
  c.max_curvature = 0.15;
  c.max_lateral_acceleration = 1.0;
  c.lookahead = 4.0;
  c.horizon = 6.0;
  c.step = 0.1;
  c.execution_lifetime = 0.2;
  c.clearance = 0.3;
  c.max_position_error = 0.1;
  c.max_jerk = 2.0;
  c.max_curvature_rate = 0.1;
  c.stitch_position_tolerance = 0.15;
  c.stitch_heading_tolerance = 0.1;
  c.stitch_speed_tolerance = 0.25;
  return c;
}

common::Status RunClosedLoop(const SimulationOptions& options,
                             SimulationReport* report) {
  if (report == nullptr) {
    return Error("simulation report is required");
  }
  *report = {};
  const std::set<std::string> scenarios{
      "straight",  "curve",   "obstacle",  "moving_obstacle",
      "stale",     "epoch",   "odom_loss", "temporal_noise",
      "occlusion", "delayed", "odom_bias"};
  if (scenarios.count(options.scenario) == 0 ||
      (options.backend != "kinematic" && options.backend != "mujoco")) {
    return Error("unknown simulation scenario or backend");
  }
  const bool curved =
      options.scenario == "curve" || options.scenario == "temporal_noise" ||
      options.scenario == "occlusion" || options.scenario == "delayed" ||
      options.scenario == "odom_bias";
  const bool obstacle =
      options.scenario == "obstacle" || options.scenario == "moving_obstacle";
  const bool fault = options.scenario == "stale" ||
                     options.scenario == "epoch" ||
                     options.scenario == "odom_loss";
  const auto config = SimulationPlannerConfig();
  simulation::SimulationEngine engine;
  engine.SetVehicleGeometry(config.wheelbase, 1.58, 0.335);
  engine.SetMaxSteerAngle(0.6108652382);
  engine.SetControlMode("throttle");
  if (!engine.SetPhysicsDt(0.002) || !engine.SetCommandTimeout(0.2) ||
      !engine.Init(options.backend, options.model_path)) {
    return Error("simulation engine initialization failed");
  }
  simulation::VehicleState truth;
  if (!engine.GetVehicleState(&truth)) {
    return Error("missing initial simulation state");
  }
  for (double s = -8.0; s <= kRoadEnd; s += 0.5) {
    report->centerline.push_back(
        curved ? Point2{kCurveRadius * std::sin(s / kCurveRadius),
                        kCurveRadius * (1.0 - std::cos(s / kCurveRadius))}
               : Point2{s, 0.0});
  }
  OdometryInput odom;
  odom.stamp = Stamp(1.0, 1);
  LaneFollowPlanner planner(SimulationInputPolicy(), config);
  auto status = planner.BeginEpoch(odom, 1.0);
  if (!status.ok()) {
    return status;
  }
  world_model::LocalSceneBuilder lane_map(
      {0.5, 2.0, 0.1, 0.6, 1.0, 0.22, 0.3, 2.0, 3.0, 6.0, 0.1, 0.3}, "odom",
      "base_link", "simulation");
  status = lane_map.BeginEpoch(SceneSource(odom.stamp, "odom"),
                               {1.0, 0, 0, 0, 0.01}, 1.0, true);
  if (!status.ok()) return status;
  world_model::LaneObservation pending;
  CycleInput input;
  bool supervisor_braking = false;
  double speed_integral = 0.0;
  simulation::VehicleCommand last_command;
  std::deque<std::string> diagnostics;
  for (int tick = 0; tick < 1500; ++tick) {
    const double now = 1.0 + tick * kDt;
    odom.stamp = Stamp(now, tick + 1);
    // Small negative speed is physics settling, not forward motion evidence.
    odom.speed_mps = truth.linear_velocity_mps;
    if (tick > 0) {
      status =
          lane_map.AddOdometry(SceneSource(odom.stamp, "odom"),
                               {now, odom.x, odom.y, odom.heading, 0.01}, now);
      if (!status.ok()) return status;
    }
    if (tick % 5 == 0) {
      input = {};
      input.planning_time = now;
      input.odometry = odom;
      input.graph = Stamp(now, tick / 5 + 1);
      input.prediction.stamp = input.graph;
      input.prediction.graph_sequence = input.graph.sequence;
      world_model::LaneObservation observation;
      observation.session = odom.stamp.epoch.producer_session;
      observation.generation = odom.stamp.epoch.generation;
      observation.sequence = tick / 5 + 1;
      observation.measurement_time = now;
      observation.position_error = 0.02;
      observation.forward_confirmed = true;
      const double station =
          curved ? kCurveRadius * std::atan2(truth.x, kCurveRadius - truth.y)
                 : truth.x;
      const double noise = options.scenario == "temporal_noise"
                               ? 0.015 * std::sin(tick * 0.7)
                               : 0.0;
      // Synthetic sensor emits only paired base-link boundaries in finite view.
      const double visible_start = std::max(-8.0, station - 8.0);
      const double visible_end = std::min(kRoadEnd, station + 25.0);
      const int intervals =
          static_cast<int>(std::ceil((visible_end - visible_start) / 0.5));
      // Include both visible endpoints regardless of the moving sample phase.
      for (int i = 0; i <= intervals; ++i) {
        const double s =
            visible_start + (visible_end - visible_start) * i / intervals;
        const double heading = curved ? s / kCurveRadius : 0.0;
        const Point2 center =
            curved ? Point2{kCurveRadius * std::sin(heading),
                            kCurveRadius * (1.0 - std::cos(heading))}
                   : Point2{s, 0.0};
        OdometryInput sensor_origin;
        const auto left = WorldToObservationToOdom(
            {center.x - std::sin(heading) * (kHalfWidth + noise),
             center.y + std::cos(heading) * (kHalfWidth + noise)},
            truth, sensor_origin);
        const auto right = WorldToObservationToOdom(
            {center.x + std::sin(heading) * (kHalfWidth - noise),
             center.y - std::cos(heading) * (kHalfWidth - noise)},
            truth, sensor_origin);
        observation.left.push_back({left.x, left.y});
        observation.right.push_back({right.x, right.y});
      }
      const bool missing =
          options.scenario == "occlusion" && tick >= 250 && tick < 265;
      if (!missing) {
        if (options.scenario == "delayed" && tick > 0) {
          if (tick > 5) {
            status = lane_map.Observe(
                SceneSource(Stamp(pending.measurement_time, pending.sequence),
                            "base_link"),
                pending, now);
            ++report->lane_updates;
          }
        } else {
          status = lane_map.Observe(
              SceneSource(Stamp(now, observation.sequence), "base_link"),
              observation, now);
          ++report->lane_updates;
        }
        if (!status.ok()) return status;
      }
      pending = observation;
      world_model::EnvironmentObservation environment;
      environment.session = odom.stamp.epoch.producer_session;
      environment.generation = odom.stamp.epoch.generation;
      environment.sequence = observation.sequence;
      environment.measurement_time = now;
      environment.position_error = 0.02;
      environment.free_space = {{-10, -12}, {30, -12}, {30, 12}, {-10, 12}};
      world_model::ObservedPolyline left_curb, right_curb;
      left_curb.id = 1;
      right_curb.id = 2;
      left_curb.points = observation.left;
      right_curb.points = observation.right;
      environment.curbs = {left_curb, right_curb};
      if (options.scenario == "obstacle") {
        const Point2 center =
            WorldToObservationToOdom({kObstacleX, 0.0}, truth, {});
        environment.obstacles = {{1,
                                  {{center.x - 0.5, center.y - 0.5},
                                   {center.x + 0.5, center.y - 0.5},
                                   {center.x + 0.5, center.y + 0.5},
                                   {center.x - 0.5, center.y + 0.5}}}};
      }
      if (!missing) {
        status = lane_map.ObserveEnvironment(
            SceneSource(Stamp(now, observation.sequence), "base_link"),
            environment, now);
        if (!status.ok()) return status;
      }
      world_model::LocalScene scene;
      status = lane_map.Build(now, &scene);
      if (!status.ok()) return status;
      input.graph.sequence = scene.source.sequence;
      input.graph.measurement_time = scene.source.measurement_time;
      input.graph.valid_until = scene.source.valid_until;
      input.prediction.graph_sequence = input.graph.sequence;
      input.occupancy.corridor_fully_observed = true;
      input.occupancy.horizon = 6.2;
      if (options.scenario == "moving_obstacle") {
        PredictedDisc disc;
        disc.id = 1;
        const bool moving = options.scenario == "moving_obstacle";
        const double world_y =
            moving ? std::max(0.0, 4.0 - tick * kDt * 0.25) : 0.0;
        disc.position =
            WorldToObservationToOdom({kObstacleX, world_y}, truth, odom);
        const double vy = moving && world_y > 0.0 ? -0.25 : 0.0;
        const double rotation = odom.heading - truth.yaw;
        disc.velocity = {-std::sin(rotation) * vy, std::cos(rotation) * vy};
        disc.radius = 1.0;
        input.occupancy.obstacles.push_back(disc);
      }
      if (fault && tick >= 250) {
        if (options.scenario == "stale") {
          scene.source.measurement_time -= 1.0;
        } else if (options.scenario == "epoch") {
          ++scene.source.generation;
        } else {
          input.odometry.stamp.health = InputHealth::INVALID;
        }
      }
      status = planner.PlanScene(scene, input.odometry, input.prediction,
                                 input.occupancy, now);
      std::ostringstream diagnostic;
      diagnostic << "t=" << now << " odom_x=" << odom.x
                 << " actual_v=" << odom.speed_mps
                 << " actual_a=" << truth.linear_acceleration_mps2
                 << " end_x=" << scene.boundaries.front().points.back().x
                 << " previous_command_a="
                 << last_command.target_acceleration_mps2;
      if (status.ok()) {
        const auto& trajectory = planner.trajectory();
        diagnostic << " stitched=" << trajectory.stitched
                   << " seed_x=" << trajectory.points.front().x
                   << " seed_v=" << trajectory.points.front().speed
                   << " seed_a=" << trajectory.points.front().acceleration;
      }
      diagnostics.push_back(diagnostic.str());
      if (diagnostics.size() > 15) diagnostics.pop_front();
      if (status.ok() && planner.trajectory().stitched)
        ++report->stitched_cycles;
      if (!status.ok()) {
        ++report->rejected_cycles;
        if (report->first_rejection.empty()) {
          report->first_rejection = status.error_message();
        }
        supervisor_braking = true;
        if (!fault || tick < 250) {
          std::string history;
          for (const auto& line : diagnostics) history += "\n" + line;
          return Error("unexpected planner rejection: " +
                       status.error_message() + history);
        }
      } else if (fault && tick >= 250) {
        return Error("injected fault was admitted by planner");
      }
    }
    simulation::VehicleCommand command;
    command.timestamp_sec = now;
    command.sequence_num = tick + 1;
    if (supervisor_braking) {
      command.emergency_stop = true;
    } else {
      status = Track(planner.trajectory(), input, now, odom, config,
                     &speed_integral, &command);
      if (!status.ok()) {
        return status;
      }
    }
    const auto before = truth;
    last_command = command;
    if (!engine.Step(command, kDt) || !engine.GetVehicleState(&truth)) {
      return Error("simulation stepping failed");
    }
    IntegrateMotion(
        (before.linear_velocity_mps + truth.linear_velocity_mps) * 0.5,
        (before.lateral_velocity_mps + truth.lateral_velocity_mps) * 0.5,
        (before.angular_velocity_yaw_radps + truth.angular_velocity_yaw_radps) *
                0.5 +
            (options.scenario == "odom_bias" ? 0.0002 : 0.0),
        &odom);
    const double error =
        curved ? std::abs(std::hypot(truth.x, truth.y - kCurveRadius) -
                          kCurveRadius)
               : std::abs(truth.y);
    report->max_lateral_error = std::max(report->max_lateral_error, error);
    report->progress =
        curved ? kCurveRadius * std::atan2(truth.x, kCurveRadius - truth.y)
               : truth.x;
    report->final_speed = std::abs(truth.linear_velocity_mps);
    if (!std::isfinite(error) || !std::isfinite(report->progress) ||
        !std::isfinite(report->final_speed) || truth.is_collision ||
        error > 0.5 || report->progress + config.front_extent > kRoadEnd ||
        (obstacle &&
         report->progress + config.front_extent + 1.0 >= kObstacleX)) {
      return Error(
          "closed-loop collision, corridor or finite-state check failed");
    }
    report->samples.push_back({now + kDt, truth.x, truth.y, odom.x, odom.y,
                               truth.linear_velocity_mps, error,
                               planner.trajectory().valid});
  }
  if (report->progress < (fault      ? 2.0
                          : obstacle ? 10.0
                                     : 25.0) ||
      report->final_speed > 0.1 || (fault && report->rejected_cycles == 0)) {
    return Error("closed-loop progress/stop/fault acceptance threshold failed");
  }
  report->passed = true;
  return common::Status::OK();
}

common::Status WriteSimulationArtifacts(const std::string& prefix,
                                        const SimulationReport& report) {
  if (prefix.empty() || report.samples.empty()) {
    return Error("artifact prefix and simulation samples are required");
  }
  std::ofstream csv(prefix + ".csv");
  if (!csv) {
    return Error("cannot open CSV artifact");
  }
  csv << "time,truth_x,truth_y,odom_x,odom_y,speed,lateral_error,trajectory_"
         "valid\n";
  csv << std::setprecision(12);
  for (const auto& p : report.samples) {
    csv << p.time << ',' << p.truth_x << ',' << p.truth_y << ',' << p.odom_x
        << ',' << p.odom_y << ',' << p.speed << ',' << p.lateral_error << ','
        << p.trajectory_valid << '\n';
  }
  csv.close();
  if (!csv) {
    return Error("failed to write CSV artifact");
  }
  std::ofstream svg(prefix + ".svg");
  if (!svg) {
    return Error("cannot open SVG artifact");
  }
  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1100\" "
         "height=\"420\""
         " viewBox=\"-10 -16 65 25\">\n"
         "<rect x=\"-10\" y=\"-16\" width=\"65\" height=\"25\" "
         "fill=\"white\"/>\n"
         "<g transform=\"scale(1,-1)\">\n";
  for (const auto& style :
       {std::string("stroke=\"#ddd\" stroke-width=\"5\""),
        std::string("stroke=\"#555\" stroke-width=\"0.08\"")}) {
    svg << "<polyline fill=\"none\" " << style << " points=\"";
    for (const auto& p : report.centerline) {
      svg << p.x << ',' << p.y << ' ';
    }
    svg << "\"/>\n";
  }
  svg << "<polyline fill=\"none\" stroke=\"#0675cc\" stroke-width=\"0.16\""
         " points=\"";
  for (const auto& p : report.samples) {
    svg << p.truth_x << ',' << p.truth_y << ' ';
  }
  svg << "\"/>\n<polyline fill=\"none\" stroke=\"#ef7300\" "
         "stroke-width=\"0.08\""
         " points=\"";
  for (const auto& p : report.samples) {
    svg << p.odom_x << ',' << p.odom_y << ' ';
  }
  svg << "\"/>\n</g><text x=\"-8\" y=\"7\" font-size=\"1\">"
         "Blue: physics trajectory; orange: integrated ODOM; gray: synthetic "
         "lane"
         "</text></svg>\n";
  svg.close();
  if (!svg) {
    return Error("failed to write SVG artifact");
  }
  return common::Status::OK();
}

}  // namespace local_planning
}  // namespace apollo
