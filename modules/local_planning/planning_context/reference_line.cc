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

#include "modules/local_planning/planning_context/reference_line.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "modules/local_planning/planning_context/boundary_geometry.h"
#include "modules/local_planning/planning_context/reference_continuity.h"
#include "modules/local_planning/planning_context/trajectory_geometry.h"

namespace apollo {
namespace local_planning {
namespace {
using common::math::Vec2d;

common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "reference geometry: " + reason);
}

double Cross(const Point2& a, const Point2& b, const Point2& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool OnSegment(const Point2& point, const Point2& a, const Point2& b) {
  return std::abs(Cross(a, b, point)) <= 1e-9 &&
         point.x >= std::min(a.x, b.x) - 1e-9 &&
         point.x <= std::max(a.x, b.x) + 1e-9 &&
         point.y >= std::min(a.y, b.y) - 1e-9 &&
         point.y <= std::max(a.y, b.y) + 1e-9;
}

bool Intersects(const Point2& a, const Point2& b, const Point2& c,
                const Point2& d) {
  const double ab_c = Cross(a, b, c);
  const double ab_d = Cross(a, b, d);
  const double cd_a = Cross(c, d, a);
  const double cd_b = Cross(c, d, b);
  if (((ab_c > 1e-9 && ab_d < -1e-9) || (ab_c < -1e-9 && ab_d > 1e-9)) &&
      ((cd_a > 1e-9 && cd_b < -1e-9) || (cd_a < -1e-9 && cd_b > 1e-9))) {
    return true;
  }
  return OnSegment(c, a, b) || OnSegment(d, a, b) || OnSegment(a, c, d) ||
         OnSegment(b, c, d);
}

}  // namespace

common::Status BuildReferenceGeometry(const world_model::LocalScene& scene,
                                      const OdometryInput& ego, double now,
                                      double rear, double front,
                                      double continuity_tolerance,
                                      const ReferenceGeometry* previous,
                                      ReferenceGeometry* output) {
  if (output == nullptr) return Error("output required");
  *output = {};
  if (scene.mode != world_model::SceneMode::LANE)
    return Error("lane reference requires LANE scene mode");
  if (!std::isfinite(rear) || !std::isfinite(front) ||
      !std::isfinite(continuity_tolerance) || rear < 0 || front <= 0 ||
      continuity_tolerance < 0)
    return Error("invalid reference extent or continuity tolerance");
  LocalCorridor corridor;
  auto status = SelectCorridor(scene, ego, now, &corridor);
  if (!status.ok()) return status;
  if (corridor.left_boundary.size() != corridor.centerline.size() ||
      corridor.right_boundary.size() != corridor.centerline.size())
    return Error("paired boundaries required");

  const double observed_half_width = corridor.half_width;
  ReferenceContinuityConfig continuity_config;
  continuity_config.maximum_displacement = continuity_tolerance;
  PreviousReferenceGeometry previous_geometry;
  if (previous != nullptr) {
    previous_geometry = {previous->source, previous->corridor,
                         previous->environment, previous->observed_half_width};
  }
  status = ApplyReferenceContinuity(
      scene, now, continuity_config,
      previous == nullptr ? nullptr : &previous_geometry, &corridor,
      &output->continuity);
  if (!status.ok()) return status;
  const bool continuous =
      output->continuity.reason == ContinuityReason::REUSED_OVERLAP;

  for (size_t i = 1; i < corridor.centerline.size(); ++i) {
    const auto& a = corridor.centerline[i - 1];
    const auto& b = corridor.centerline[i];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (!std::isfinite(length) || length < 0.05 || length > 2.0 ||
        (i > 1 && dx * (a.x - corridor.centerline[i - 2].x) +
                          dy * (a.y - corridor.centerline[i - 2].y) <
                      -1e-9)) {
      return Error(
          "continuity produced duplicate, oversized or reversed "
          "reference samples");
    }
    for (size_t j = 1; j + 1 < i; ++j) {
      if (Intersects(corridor.centerline[i - 1], corridor.centerline[i],
                     corridor.centerline[j - 1], corridor.centerline[j])) {
        return Error("reference centerline self-intersects");
      }
    }
  }

  const CorridorPath path(corridor);
  const auto projection = path.Project(Vec2d(ego.x, ego.y));
  const auto nearest = path.At(projection.s);
  const double dx = ego.x - nearest.x(), dy = ego.y - nearest.y();
  const double lateral =
      -std::sin(projection.heading) * dx + std::cos(projection.heading) * dy;
  const double start = std::max(0.0, projection.s - rear);
  const double end = std::min(path.length(), projection.s + front);
  if (end - start < 0.05 ||
      projection.distance >=
          corridor.half_width - corridor.position_error_bound)
    return Error("ego outside usable observed reference extent");
  output->source = scene.source;
  output->corridor = std::move(corridor);
  output->environment = scene.environment;
  output->ego_s = projection.s;
  output->ego_l = lateral;
  output->start_s = start;
  output->end_s = end;
  output->observed_half_width = observed_half_width;
  output->continuous = continuous;
  MeasuredBoundaryLines boundaries;
  status =
      BuildMeasuredBoundaryLines(output->corridor, start, end, &boundaries);
  if (!status.ok()) {
    *output = {};
    return status;
  }
  output->stations = std::move(boundaries.stations);
  output->line = std::move(boundaries.center);
  output->left = std::move(boundaries.left);
  output->right = std::move(boundaries.right);
  return common::Status::OK();
}

}  // namespace local_planning
}  // namespace apollo
