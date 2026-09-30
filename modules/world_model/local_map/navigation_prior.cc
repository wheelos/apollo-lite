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

#include "modules/world_model/local_map/navigation_prior.h"

#include <cmath>

namespace apollo {
namespace world_model {
namespace {

common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "navigation prior: " + reason);
}

}  // namespace

common::Status FuseNavigationPrior(const NavigationPrior& prior, double now,
                                   LocalScene* scene) {
  if (scene == nullptr || scene->source.health != SceneHealth::HEALTHY ||
      scene->navigation || !std::isfinite(now) ||
      !std::isfinite(prior.source.measurement_time) ||
      !std::isfinite(prior.source.publication_time) ||
      !std::isfinite(prior.source.valid_until) ||
      prior.source.frame_id != scene->source.frame_id ||
      prior.source.clock_id != scene->source.clock_id ||
      prior.source.session != scene->source.session ||
      prior.source.generation != scene->source.generation ||
      prior.source.sequence == 0 ||
      prior.source.health != SceneHealth::HEALTHY ||
      prior.source.measurement_time < 0 ||
      prior.source.measurement_time > prior.source.publication_time ||
      prior.source.publication_time > now || now >= prior.source.valid_until ||
      prior.from_lane == 0 || prior.to_lane == 0 ||
      prior.from_lane == prior.to_lane) {
    return Error("invalid provenance, lifetime or route identity");
  }
  int from = 0, to = 0, matching = 0;
  for (const auto& lane : scene->lanes) {
    from += lane.id == prior.from_lane;
    to += lane.id == prior.to_lane;
  }
  for (const auto& edge : scene->edges) {
    matching += edge.from == prior.from_lane && edge.to == prior.to_lane &&
                edge.type == EdgeType::SUCCESSOR;
  }
  if (from != 1 || to != 1 || matching != 1)
    return Error("ambiguous or unobserved route connection");
  scene->navigation = prior;
  return common::Status::OK();
}

}  // namespace world_model
}  // namespace apollo
