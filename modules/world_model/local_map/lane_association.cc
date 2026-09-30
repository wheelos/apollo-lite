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

#include "modules/world_model/local_map/lane_association.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace apollo {
namespace world_model {
namespace {

common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "lane association: " + reason);
}

double Distance(const LanePoint& a, const LanePoint& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

LanePoint Mix(const LanePoint& a, const LanePoint& b, double weight) {
  return {a.x * (1.0 - weight) + b.x * weight,
          a.y * (1.0 - weight) + b.y * weight};
}

bool Finite(const LanePoint& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool ValidSample(const TrackedLaneSample& sample, double now) {
  return Finite(sample.left) && Finite(sample.right) &&
         std::isfinite(sample.last_observed) && sample.last_observed >= 0.0 &&
         sample.last_observed <= now && std::isfinite(sample.position_error) &&
         sample.position_error >= 0.0;
}

}  // namespace

common::Status AssociateLaneSamples(
    const std::map<int, TrackedLaneSample>& history,
    const std::map<int, TrackedLaneSample>& candidates,
    const TemporalLanePolicy& policy, double now,
    LaneAssociationResult* result) {
  if (result == nullptr) return Error("output required");
  *result = {};
  if (!std::isfinite(now) || candidates.empty() ||
      !std::isfinite(policy.grid_spacing) || policy.grid_spacing <= 0.0 ||
      !std::isfinite(policy.sample_lifetime) || policy.sample_lifetime <= 0.0 ||
      !std::isfinite(policy.association_distance) ||
      policy.association_distance <= 0.0 ||
      !std::isfinite(policy.min_overlap) || policy.min_overlap < 0.0 ||
      !std::isfinite(policy.fresh_weight) || policy.fresh_weight <= 0.0 ||
      policy.fresh_weight > 1.0) {
    return Error("invalid input or policy");
  }

  for (const auto& entry : candidates) {
    const auto& candidate = entry.second;
    if (!ValidSample(candidate, now)) return Error("invalid candidate sample");
    auto fused = candidate;
    const auto old = history.find(entry.first);
    if (old != history.end()) {
      if (!ValidSample(old->second, now))
        return Error("invalid history sample");
      if (now - old->second.last_observed < policy.sample_lifetime) {
        if (Distance(candidate.left, old->second.left) >
                policy.association_distance ||
            Distance(candidate.right, old->second.right) >
                policy.association_distance) {
          return Error("ambiguous lane association");
        }
        ++result->overlap_samples;
        fused.left = Mix(old->second.left, candidate.left, policy.fresh_weight);
        fused.right =
            Mix(old->second.right, candidate.right, policy.fresh_weight);

        const double nx = candidate.left.x - candidate.right.x;
        const double ny = candidate.left.y - candidate.right.y;
        if ((fused.left.x - candidate.left.x) * nx +
                (fused.left.y - candidate.left.y) * ny >
            0.0) {
          fused.left = candidate.left;
        }
        if ((fused.right.x - candidate.right.x) * nx +
                (fused.right.y - candidate.right.y) * ny <
            0.0) {
          fused.right = candidate.right;
        }
        fused.position_error =
            std::max(candidate.position_error, old->second.position_error);
      }
    }
    result->samples.emplace(entry.first, fused);
  }

  if (!history.empty() &&
      result->overlap_samples * policy.grid_spacing < policy.min_overlap) {
    *result = {};
    return Error("insufficient overlap to associate lane history");
  }
  return common::Status::OK();
}

}  // namespace world_model
}  // namespace apollo
