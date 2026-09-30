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

#include "modules/local_planning/planning_context/reference_continuity.h"

#include <algorithm>
#include <cmath>

#include "modules/common/math/line_segment2d.h"
#include "modules/local_planning/planning_context/trajectory_geometry.h"

namespace apollo {
namespace local_planning {
namespace {
using common::math::Vec2d;

bool SameEnvironmentGeometry(const world_model::DrivableEnvironment& a,
                             const world_model::DrivableEnvironment& b) {
  return a.position_error == b.position_error && a.free_space == b.free_space &&
         a.curbs == b.curbs && a.obstacles == b.obstacles;
}

void Explain(ContinuityReason reason, const std::string& detail,
             ReferenceContinuityReport* report) {
  report->reason = reason;
  report->detail = detail;
}

}  // namespace

common::Status ApplyReferenceContinuity(
    const world_model::LocalScene& scene, double now,
    const ReferenceContinuityConfig& config,
    const PreviousReferenceGeometry* previous, LocalCorridor* current,
    ReferenceContinuityReport* report) {
  if (current == nullptr || report == nullptr) {
    return common::Status(common::ErrorCode::PLANNING_ERROR,
                          "reference continuity: outputs required");
  }
  *report = {};
  if (!std::isfinite(now) || !std::isfinite(config.maximum_displacement) ||
      config.maximum_displacement < 0.0) {
    return common::Status(common::ErrorCode::PLANNING_ERROR,
                          "reference continuity: invalid time or tolerance");
  }
  if (previous == nullptr) {
    Explain(ContinuityReason::NO_PREVIOUS_REFERENCE,
            "no accepted prior reference", report);
    return common::Status::OK();
  }
  const auto& a = previous->source;
  const auto& b = scene.source;
  if (a.frame_id != b.frame_id || a.clock_id != b.clock_id) {
    Explain(ContinuityReason::FRAME_OR_CLOCK_CHANGED,
            "reference frame or clock changed", report);
    return common::Status::OK();
  }
  if (a.session != b.session || a.generation != b.generation) {
    Explain(ContinuityReason::ODOM_EPOCH_CHANGED, "ODOM epoch changed", report);
    return common::Status::OK();
  }
  if (now >= a.valid_until) {
    Explain(ContinuityReason::PREVIOUS_REFERENCE_EXPIRED,
            "prior scene deadline elapsed", report);
    return common::Status::OK();
  }
  if (previous->corridor.lane_id != current->lane_id) {
    Explain(ContinuityReason::LANE_IDENTITY_CHANGED, "lane identity changed",
            report);
    return common::Status::OK();
  }
  if (previous->corridor.centerline.size() < 2 ||
      !SameEnvironmentGeometry(previous->environment, scene.environment) ||
      current->half_width < previous->observed_half_width - 1e-9 ||
      current->position_error_bound >
          previous->corridor.position_error_bound + 1e-9 ||
      current->stop_station != previous->corridor.stop_station ||
      current->speed_limit != previous->corridor.speed_limit) {
    Explain(ContinuityReason::RESTRICTION_CHANGED,
            "a restriction or environment changed; reuse is disabled", report);
    return common::Status::OK();
  }

  const CorridorPath old_path(previous->corridor);
  // Replanning against one immutable scene must not repeatedly project and
  // deform the same reference. Only the ego projection/window should change.
  if (a == b && current->left_boundary == previous->corridor.left_boundary &&
      current->right_boundary == previous->corridor.right_boundary &&
      current->centerline.size() == previous->corridor.centerline.size()) {
    double displacement = 0.0;
    for (size_t i = 0; i < current->centerline.size(); ++i) {
      displacement = std::max(
          displacement,
          std::hypot(
              current->centerline[i].x - previous->corridor.centerline[i].x,
              current->centerline[i].y - previous->corridor.centerline[i].y));
    }
    if (displacement <= config.maximum_displacement) {
      current->centerline = previous->corridor.centerline;
      current->half_width = previous->corridor.half_width;
      report->reused_points = current->centerline.size();
      report->first_reused_station = 0.0;
      report->last_reused_station = old_path.length();
      report->maximum_displacement = displacement;
      Explain(ContinuityReason::REUSED_OVERLAP,
              "retained reference for the same immutable scene", report);
      return common::Status::OK();
    }
  }
  const Vec2d first(previous->corridor.centerline.front().x,
                    previous->corridor.centerline.front().y);
  const Vec2d last(previous->corridor.centerline.back().x,
                   previous->corridor.centerline.back().y);
  double first_station = 0.0;
  double last_station = 0.0;
  for (auto& point : current->centerline) {
    const auto projection = old_path.Project(Vec2d(point.x, point.y));
    const auto old = old_path.At(projection.s);
    if (projection.s <= 0.0 || projection.s >= old_path.length() ||
        projection.distance > config.maximum_displacement ||
        old.DistanceTo(first) < 1e-9 || old.DistanceTo(last) < 1e-9) {
      continue;
    }
    if (report->reused_points == 0) first_station = projection.s;
    last_station = projection.s;
    report->maximum_displacement =
        std::max(report->maximum_displacement, projection.distance);
    point = {old.x(), old.y()};
    ++report->reused_points;
  }
  if (report->reused_points == 0) {
    Explain(ContinuityReason::NO_USABLE_OVERLAP,
            "no interior overlap was within the displacement bound", report);
    return common::Status::OK();
  }
  report->reason = ContinuityReason::REUSED_OVERLAP;
  report->first_reused_station = first_station;
  report->last_reused_station = last_station;
  report->detail = "reused bounded prior centerline overlap";
  current->half_width -= report->maximum_displacement;
  if (current->half_width <= current->position_error_bound) {
    return common::Status(common::ErrorCode::PLANNING_ERROR,
                          "reference continuity: no usable width after reuse");
  }
  return common::Status::OK();
}

}  // namespace local_planning
}  // namespace apollo
