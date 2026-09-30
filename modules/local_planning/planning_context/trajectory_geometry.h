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

#include <algorithm>
#include <limits>
#include <vector>

#include "modules/common/math/line_segment2d.h"
#include "modules/local_planning/planning_context/scene.h"

namespace apollo {
namespace local_planning {

struct Projection {
  double s = 0.0;
  double distance = std::numeric_limits<double>::infinity();
  double heading = 0.0;
};

// Callers validate corridor sampling before construction.
class CorridorPath {
 public:
  explicit CorridorPath(const LocalCorridor& corridor) {
    for (size_t i = 1; i < corridor.centerline.size(); ++i) {
      const auto& a = corridor.centerline[i - 1];
      const auto& b = corridor.centerline[i];
      segments_.emplace_back(common::math::Vec2d(a.x, a.y),
                             common::math::Vec2d(b.x, b.y));
      length_ += segments_.back().length();
    }
  }
  Projection Project(const common::math::Vec2d& p) const {
    Projection result;
    double s = 0.0;
    for (const auto& segment : segments_) {
      common::math::Vec2d nearest;
      const double distance = segment.DistanceTo(p, &nearest);
      if (distance < result.distance) {
        result = {s + nearest.DistanceTo(segment.start()), distance,
                  segment.heading()};
      }
      s += segment.length();
    }
    return result;
  }
  common::math::Vec2d At(double s) const {
    for (const auto& segment : segments_) {
      if (s <= segment.length())
        return segment.start() + segment.unit_direction() * std::max(0.0, s);
      s -= segment.length();
    }
    return segments_.back().end();
  }
  double length() const { return length_; }

 private:
  std::vector<common::math::LineSegment2d> segments_;
  double length_ = 0.0;
};

}  // namespace local_planning
}  // namespace apollo
