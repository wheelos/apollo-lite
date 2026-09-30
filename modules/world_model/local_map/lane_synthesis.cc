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

#include "modules/world_model/local_map/lane_synthesis.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace apollo {
namespace world_model {
namespace {

common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "lane synthesis: " + reason);
}

}  // namespace

common::Status SynthesizeObservedLane(const SceneSource& source,
                                      const TemporalLaneSnapshot& snapshot,
                                      const TemporalLanePolicy& policy,
                                      uint64_t left_id, uint64_t right_id,
                                      LaneLayer* layer) {
  if (layer == nullptr) return Error("output required");
  *layer = {};
  if (source.frame_id.empty() || source.clock_id.empty() ||
      source.session != snapshot.session ||
      source.generation != snapshot.generation || source.sequence == 0 ||
      source.health != SceneHealth::HEALTHY ||
      !std::isfinite(source.measurement_time) ||
      !std::isfinite(source.publication_time) ||
      !std::isfinite(source.valid_until) ||
      source.measurement_time > source.publication_time ||
      source.publication_time >= source.valid_until || snapshot.version == 0 ||
      snapshot.lane_id == 0 || snapshot.samples.size() < 3 ||
      !std::isfinite(snapshot.measurement_time) ||
      !std::isfinite(snapshot.valid_until) ||
      snapshot.measurement_time > source.publication_time ||
      snapshot.valid_until < source.valid_until || left_id == 0 ||
      right_id == 0 || left_id == right_id ||
      !std::isfinite(policy.sample_lifetime) || policy.sample_lifetime <= 0.0) {
    return Error("invalid source, track identity or lifetime");
  }

  LaneBoundary left, right;
  left.id = left_id;
  right.id = right_id;
  LaneSegment lane;
  lane.id = snapshot.lane_id;
  lane.left_boundary = left_id;
  lane.right_boundary = right_id;
  lane.forward = EvidenceState::CONFIRMED;
  for (const auto& sample : snapshot.samples) {
    if (!std::isfinite(sample.left.x) || !std::isfinite(sample.left.y) ||
        !std::isfinite(sample.right.x) || !std::isfinite(sample.right.y) ||
        !std::isfinite(sample.last_observed) || sample.last_observed < 0.0 ||
        sample.last_observed > source.publication_time ||
        !std::isfinite(sample.position_error) || sample.position_error < 0.0) {
      return Error("invalid tracked boundary sample");
    }
    left.points.push_back(sample.left);
    right.points.push_back(sample.right);
    left.observation_times.push_back(sample.last_observed);
    right.observation_times.push_back(sample.last_observed);
    left.valid_until.push_back(sample.last_observed + policy.sample_lifetime);
    right.valid_until.push_back(sample.last_observed + policy.sample_lifetime);
    lane.position_error = std::max(lane.position_error, sample.position_error);
  }
  layer->source = source;
  layer->source.sequence = snapshot.version;
  layer->source.measurement_time = snapshot.measurement_time;
  layer->source.valid_until =
      std::min(source.valid_until, snapshot.valid_until);
  layer->boundaries = {std::move(left), std::move(right)};
  layer->lanes.push_back(lane);
  return common::Status::OK();
}

}  // namespace world_model
}  // namespace apollo
