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

#include "modules/open_space_planning/safety/default_trajectory_validator.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace open_space_planning {

DefaultTrajectoryValidator::DefaultTrajectoryValidator(
    TrajectoryValidatorConfig config)
    : config_(config) {}

ValidationReport DefaultTrajectoryValidator::Validate(
    const TrajectoryValidationRequest& request) const {
  ValidationReport report;
  report.safe = true;

  const auto& traj = request.trajectory;
  if (traj.points.empty()) {
    report.safe = false;
    report.issues.push_back(
        {"EmptyTrajectory", "Trajectory contains no points", 0});
    return report;
  }

  for (std::size_t i = 0; i < traj.points.size(); ++i) {
    const auto& pt = traj.points[i];

    // Speed check
    if (std::abs(pt.velocity) > config_.maximum_speed) {
      report.safe = false;
      report.issues.push_back({"SpeedLimitViolation",
                               "Point speed exceeds maximum allowed limit", i});
    }

    // Acceleration check
    if (pt.acceleration > config_.maximum_acceleration ||
        pt.acceleration < -config_.maximum_deceleration) {
      report.safe = false;
      report.issues.push_back(
          {"AccelerationLimitViolation",
           "Point acceleration exceeds allowed physical limits", i});
    }

    // Curvature check
    if (std::abs(pt.curvature) > config_.maximum_curvature) {
      report.safe = false;
      report.issues.push_back(
          {"CurvatureLimitViolation",
           "Point curvature exceeds maximum allowable limit", i});
    }

    // Monotonicity check
    if (i > 0) {
      const auto& prev = traj.points[i - 1];
      if (pt.relative_time < prev.relative_time) {
        report.safe = false;
        report.issues.push_back(
            {"NonMonotonicTime",
             "Trajectory relative time is not monotonically increasing", i});
      }
      if (pt.s < prev.s - 1e-4) {
        report.safe = false;
        report.issues.push_back(
            {"NonMonotonicStation",
             "Trajectory station s is not monotonically increasing", i});
      }
    }
  }

  if (traj.points.back().relative_time > config_.maximum_time_horizon) {
    report.safe = false;
    report.issues.push_back({"TimeHorizonExceeded",
                             "Trajectory duration exceeds maximum time horizon",
                             traj.points.size() - 1});
  }

  // Dynamic obstacles collision check
  const auto& obstacles = request.problem.dynamic_obstacles;
  for (std::size_t i = 0; i < traj.points.size(); ++i) {
    const auto& pt = traj.points[i];
    const double t = pt.relative_time;

    for (const auto& obs : obstacles) {
      if (obs.prediction.empty()) {
        continue;
      }
      Pose2d obs_pose = obs.prediction.front().pose;
      if (t <= obs.prediction.front().relative_time) {
        obs_pose = obs.prediction.front().pose;
      } else if (t >= obs.prediction.back().relative_time) {
        obs_pose = obs.prediction.back().pose;
      } else {
        for (std::size_t k = 0; k + 1 < obs.prediction.size(); ++k) {
          if (t >= obs.prediction[k].relative_time &&
              t <= obs.prediction[k + 1].relative_time) {
            const double dt = obs.prediction[k + 1].relative_time -
                              obs.prediction[k].relative_time;
            const double r =
                (dt > 1e-6) ? (t - obs.prediction[k].relative_time) / dt : 0.0;
            obs_pose.x = (1.0 - r) * obs.prediction[k].pose.x +
                         r * obs.prediction[k + 1].pose.x;
            obs_pose.y = (1.0 - r) * obs.prediction[k].pose.y +
                         r * obs.prediction[k + 1].pose.y;
            break;
          }
        }
      }

      double obs_radius = 0.5;
      if (!obs.footprint.empty()) {
        double cx = 0.0;
        double cy = 0.0;
        for (const auto& pt_obs : obs.footprint) {
          cx += pt_obs.x;
          cy += pt_obs.y;
        }
        cx /= static_cast<double>(obs.footprint.size());
        cy /= static_cast<double>(obs.footprint.size());
        obs_radius = 0.0;
        for (const auto& pt_obs : obs.footprint) {
          obs_radius =
              std::max(obs_radius, std::hypot(pt_obs.x - cx, pt_obs.y - cy));
        }
      }

      const double dist =
          std::hypot(pt.pose.x - obs_pose.x, pt.pose.y - obs_pose.y);
      if (dist < (obs_radius + config_.minimum_obstacle_clearance)) {
        report.safe = false;
        report.issues.push_back(
            {"DynamicObstacleCollision",
             "Trajectory point collides with dynamic obstacle: " + obs.id, i});
      }
    }
  }

  return report;
}

}  // namespace open_space_planning
}  // namespace apollo
