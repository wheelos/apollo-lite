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

#include "modules/world_model/local_map/scene_assembly.h"

#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_set>

#include "modules/world_model/local_map/drivable_environment.h"
#include "modules/world_model/local_map/navigation_prior.h"

namespace apollo {
namespace world_model {
namespace {

common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "scene assembly: " + reason);
}

bool SameEpochFrameClock(const SceneSource& a, const SceneSource& b) {
  return a.frame_id == b.frame_id && a.clock_id == b.clock_id &&
         a.session == b.session && a.generation == b.generation;
}

bool ValidEvidence(EvidenceState state) {
  return state == EvidenceState::UNKNOWN ||
         state == EvidenceState::HYPOTHESIS ||
         state == EvidenceState::CONFIRMED || state == EvidenceState::INVALID;
}

common::Status ValidateLaneLayer(const SceneSource& source,
                                 const LaneLayer& layer) {
  if (!SameEpochFrameClock(source, layer.source) ||
      layer.source.health != SceneHealth::HEALTHY ||
      layer.source.sequence == 0 || layer.lanes.empty() ||
      layer.boundaries.empty() || layer.lanes.size() > 200 ||
      layer.boundaries.size() > 400 || layer.edges.size() > 1000 ||
      layer.rules.size() > 1000 ||
      !std::isfinite(layer.source.measurement_time) ||
      !std::isfinite(layer.source.publication_time) ||
      !std::isfinite(layer.source.valid_until) ||
      layer.source.measurement_time < 0.0 ||
      layer.source.measurement_time > layer.source.publication_time ||
      layer.source.measurement_time > source.measurement_time ||
      layer.source.publication_time > source.publication_time ||
      layer.source.valid_until < source.valid_until) {
    return Error("lane layer provenance does not fit scene lifetime");
  }
  std::unordered_set<uint64_t> boundary_ids;
  for (const auto& boundary : layer.boundaries) {
    if (boundary.id == 0 || boundary.points.size() < 2 ||
        boundary.points.size() > 2000 || !ValidEvidence(boundary.crossing) ||
        boundary.observation_times.size() != boundary.points.size() ||
        boundary.valid_until.size() != boundary.points.size() ||
        !boundary_ids.insert(boundary.id).second) {
      return Error("invalid or duplicate lane boundary");
    }
    for (size_t i = 0; i < boundary.points.size(); ++i) {
      if (!std::isfinite(boundary.points[i].x) ||
          !std::isfinite(boundary.points[i].y) ||
          !std::isfinite(boundary.observation_times[i]) ||
          !std::isfinite(boundary.valid_until[i]) ||
          boundary.observation_times[i] < 0.0 ||
          boundary.observation_times[i] > source.publication_time ||
          boundary.valid_until[i] <= boundary.observation_times[i] ||
          boundary.valid_until[i] < source.valid_until) {
        return Error("invalid lane boundary freshness");
      }
    }
  }

  std::unordered_set<uint64_t> lane_ids;
  for (const auto& lane : layer.lanes) {
    if (lane.id == 0 || !boundary_ids.count(lane.left_boundary) ||
        !boundary_ids.count(lane.right_boundary) ||
        lane.left_boundary == lane.right_boundary ||
        !ValidEvidence(lane.forward) || !std::isfinite(lane.position_error) ||
        lane.position_error < 0.0 || !lane_ids.insert(lane.id).second) {
      return Error("invalid or duplicate lane identity/reference");
    }
    const LaneBoundary* left = nullptr;
    const LaneBoundary* right = nullptr;
    for (const auto& boundary : layer.boundaries) {
      if (boundary.id == lane.left_boundary) left = &boundary;
      if (boundary.id == lane.right_boundary) right = &boundary;
    }
    if (left->points.size() != right->points.size())
      return Error("unpaired lane boundary geometry");
    for (size_t i = 0; i < left->points.size(); ++i) {
      const double width = std::hypot(left->points[i].x - right->points[i].x,
                                      left->points[i].y - right->points[i].y);
      if (!std::isfinite(width) || width < 0.05)
        return Error("degenerate lane boundary width");
      if (i == 0) continue;
      const double dx = (left->points[i].x + right->points[i].x -
                         left->points[i - 1].x - right->points[i - 1].x);
      const double dy = (left->points[i].y + right->points[i].y -
                         left->points[i - 1].y - right->points[i - 1].y);
      const double cross = dx * (left->points[i].y - right->points[i].y) -
                           dy * (left->points[i].x - right->points[i].x);
      if (std::hypot(dx, dy) < 0.1 || cross <= 0.0)
        return Error("reversed or degenerate paired lane");
    }
  }
  std::unordered_set<uint64_t> rule_ids;
  for (const auto& rule : layer.rules) {
    if (rule.id == 0 || !lane_ids.count(rule.lane_id) ||
        !ValidEvidence(rule.evidence) ||
        (rule.type != RuleType::STOP && rule.type != RuleType::SPEED_LIMIT) ||
        !std::isfinite(rule.station) || rule.station < 0.0 ||
        !std::isfinite(rule.speed_mps) || rule.speed_mps < 0.0 ||
        !rule_ids.insert(rule.id).second) {
      return Error("invalid or duplicate rule reference");
    }
  }
  std::unordered_set<std::string> edges;
  for (const auto& edge : layer.edges) {
    if (!lane_ids.count(edge.from) || !lane_ids.count(edge.to) ||
        edge.from == edge.to || !ValidEvidence(edge.connection) ||
        !ValidEvidence(edge.permission) ||
        (edge.type != EdgeType::SUCCESSOR &&
         edge.type != EdgeType::ADJACENT_LEFT &&
         edge.type != EdgeType::ADJACENT_RIGHT &&
         edge.type != EdgeType::MERGE && edge.type != EdgeType::SPLIT) ||
        !edges
             .insert(std::to_string(edge.from) + ":" + std::to_string(edge.to) +
                     ":" + std::to_string(static_cast<int>(edge.type)))
             .second) {
      return Error("invalid topology edge reference");
    }
  }
  return common::Status::OK();
}

}  // namespace

common::Status AssembleLocalScene(const SceneSource& source,
                                  const DrivableEnvironment& environment,
                                  const LaneLayer* lanes,
                                  const std::string& area_reason,
                                  const NavigationPrior* navigation, double now,
                                  LocalScene* scene) {
  if (scene == nullptr) return Error("output required");
  *scene = {};
  if (source.frame_id.empty() || source.clock_id.empty() ||
      source.session.empty() || source.generation == 0 ||
      source.sequence == 0 || source.health != SceneHealth::HEALTHY ||
      !std::isfinite(source.measurement_time) ||
      !std::isfinite(source.publication_time) ||
      !std::isfinite(source.valid_until) || !std::isfinite(now) ||
      source.measurement_time < 0.0 ||
      source.measurement_time > source.publication_time ||
      source.publication_time > now || now >= source.valid_until) {
    return Error("invalid scene source or lifetime");
  }
  if (!SameEpochFrameClock(source, environment.source) ||
      environment.source.health != SceneHealth::HEALTHY ||
      environment.source.sequence == 0 ||
      !std::isfinite(environment.source.measurement_time) ||
      !std::isfinite(environment.source.publication_time) ||
      !std::isfinite(environment.source.valid_until) ||
      environment.source.measurement_time > source.measurement_time ||
      environment.source.publication_time > source.publication_time ||
      environment.source.valid_until < source.valid_until ||
      !std::isfinite(environment.position_error) ||
      environment.position_error < 0.0) {
    return Error("environment layer provenance or geometry is invalid");
  }
  auto status = ValidateDrivableEnvironment(environment);
  if (!status.ok()) return status;

  if (lanes != nullptr) {
    status = ValidateLaneLayer(source, *lanes);
    if (!status.ok()) return status;
  }
  scene->source = source;
  scene->reference = VehicleReference::REAR_AXLE;
  scene->environment = environment;
  scene->capabilities.drivable_region = true;
  if (lanes != nullptr) {
    scene->mode = SceneMode::LANE;
    scene->capabilities.lane_follow = true;
    scene->boundaries = lanes->boundaries;
    scene->lanes = lanes->lanes;
    scene->edges = lanes->edges;
    scene->rules = lanes->rules;
  } else {
    scene->mode = SceneMode::AREA;
    scene->mode_reason = area_reason;
    if (navigation != nullptr) {
      scene->mode_reason +=
          "; navigation prior not applicable without observed lanes";
    }
  }
  if (navigation != nullptr && lanes != nullptr) {
    auto status = FuseNavigationPrior(*navigation, now, scene);
    if (!status.ok()) {
      *scene = {};
      return status;
    }
  }
  return common::Status::OK();
}

}  // namespace world_model
}  // namespace apollo
