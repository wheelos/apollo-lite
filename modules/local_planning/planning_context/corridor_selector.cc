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

#include "modules/local_planning/planning_context/corridor_selector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "modules/common/math/line_segment2d.h"

namespace apollo {
namespace local_planning {
namespace {
using world_model::EvidenceState;
common::Status Error(const std::string& message) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "corridor selection: " + message);
}
double Distance(const Point2& a, const Point2& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
bool Finite(const world_model::LanePoint& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::abs(p.x) < 1e6 &&
         std::abs(p.y) < 1e6;
}
double Length(const LocalCorridor& corridor) {
  double length = 0.0;
  for (size_t i = 1; i < corridor.centerline.size(); ++i)
    length += Distance(corridor.centerline[i - 1], corridor.centerline[i]);
  return length;
}
bool ValidEvidence(EvidenceState state) {
  return state == EvidenceState::UNKNOWN ||
         state == EvidenceState::HYPOTHESIS ||
         state == EvidenceState::CONFIRMED || state == EvidenceState::INVALID;
}
}  // namespace

common::Status SelectCorridor(const world_model::LocalScene& scene,
                              const OdometryInput& ego, double now,
                              LocalCorridor* out) {
  if (out == nullptr) return Error("output required");
  *out = {};
  const auto& s = scene.source;
  if (!std::isfinite(now) || !std::isfinite(s.measurement_time) ||
      !std::isfinite(s.publication_time) || !std::isfinite(s.valid_until) ||
      s.measurement_time < 0.0 || s.measurement_time > s.publication_time ||
      s.publication_time > now || now >= s.valid_until ||
      s.health != world_model::SceneHealth::HEALTHY ||
      !scene.invalid_reason.empty() || s.frame_id.empty() ||
      s.clock_id.empty() || s.session.empty() || s.generation == 0 ||
      s.sequence == 0 || s.frame_id != ego.stamp.frame_id ||
      s.clock_id != ego.stamp.clock_id ||
      s.session != ego.stamp.epoch.producer_session ||
      s.generation != ego.stamp.epoch.generation ||
      scene.reference != world_model::VehicleReference::REAR_AXLE ||
      scene.mode != world_model::SceneMode::LANE ||
      !scene.capabilities.drivable_region || !scene.capabilities.lane_follow ||
      scene.lanes.empty() || scene.lanes.size() > 100 ||
      scene.boundaries.size() > 200 || scene.edges.size() > 1000 ||
      scene.rules.size() > 1000 || !std::isfinite(ego.x) ||
      !std::isfinite(ego.y) || !std::isfinite(ego.heading)) {
    return Error("invalid scene provenance, integrity or capability");
  }
  std::unordered_map<uint64_t, const world_model::LaneBoundary*> boundaries;
  for (const auto& b : scene.boundaries) {
    if (b.id == 0 || !boundaries.emplace(b.id, &b).second ||
        !ValidEvidence(b.crossing) || b.points.size() < 2 ||
        b.points.size() > 2000 ||
        b.observation_times.size() != b.points.size() ||
        b.valid_until.size() != b.points.size()) {
      return Error("invalid or duplicate boundary identity");
    }
    for (size_t i = 0; i < b.points.size(); ++i) {
      if (!Finite(b.points[i]) || !std::isfinite(b.observation_times[i]) ||
          b.observation_times[i] < 0.0 ||
          b.observation_times[i] > s.measurement_time ||
          !std::isfinite(b.valid_until[i]) ||
          b.valid_until[i] < s.valid_until) {
        return Error("invalid boundary geometry or evidence time");
      }
    }
  }
  std::unordered_map<uint64_t, LocalCorridor> lanes;
  uint64_t selected = 0;
  for (const auto& lane : scene.lanes) {
    if (lane.id == 0 || lanes.count(lane.id) || !ValidEvidence(lane.forward) ||
        lane.left_boundary == lane.right_boundary ||
        !boundaries.count(lane.left_boundary) ||
        !boundaries.count(lane.right_boundary) ||
        !std::isfinite(lane.position_error) || lane.position_error < 0.0) {
      return Error("invalid lane or boundary reference");
    }
    const auto& left = *boundaries.at(lane.left_boundary);
    const auto& right = *boundaries.at(lane.right_boundary);
    if (left.points.size() != right.points.size())
      return Error("unpaired lane sampling");
    LocalCorridor corridor;
    corridor.lane_id = lane.id;
    corridor.confirmed_forward = lane.forward == EvidenceState::CONFIRMED;
    corridor.position_error_bound = lane.position_error;
    corridor.half_width = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < left.points.size(); ++i) {
      const auto& a = left.points[i];
      const auto& b = right.points[i];
      const double width = std::hypot(a.x - b.x, a.y - b.y);
      if (width < 0.1 || width > 20.0) return Error("invalid lane width");
      corridor.half_width = std::min(corridor.half_width, width * 0.5);
      corridor.left_boundary.push_back({a.x, a.y});
      corridor.right_boundary.push_back({b.x, b.y});
      corridor.centerline.push_back({(a.x + b.x) * 0.5, (a.y + b.y) * 0.5});
      if (i > 0) {
        const auto& previous = corridor.centerline[i - 1];
        const auto& current = corridor.centerline[i];
        const double dx = current.x - previous.x;
        const double dy = current.y - previous.y;
        if (std::hypot(dx, dy) < 0.05 || std::hypot(dx, dy) > 2.0 ||
            dx * (a.y - b.y) - dy * (a.x - b.x) <= 0.0)
          return Error("reversed or disconnected paired boundaries");
      }
    }
    double nearest = std::numeric_limits<double>::infinity();
    double heading = 0.0;
    for (size_t i = 1; i < corridor.centerline.size(); ++i) {
      const auto& a = corridor.centerline[i - 1];
      const auto& b = corridor.centerline[i];
      common::math::LineSegment2d segment({a.x, a.y}, {b.x, b.y});
      const double distance = segment.DistanceTo({ego.x, ego.y});
      if (distance < nearest) {
        nearest = distance;
        heading = segment.heading();
      }
    }
    const double angle = std::atan2(std::sin(heading - ego.heading),
                                    std::cos(heading - ego.heading));
    const auto& start = corridor.centerline.front();
    const auto& second = corridor.centerline[1];
    const auto& end = corridor.centerline.back();
    const auto& before = corridor.centerline[corridor.centerline.size() - 2];
    const bool within_ends = (ego.x - start.x) * (second.x - start.x) +
                                     (ego.y - start.y) * (second.y - start.y) >=
                                 0.0 &&
                             (ego.x - end.x) * (end.x - before.x) +
                                     (ego.y - end.y) * (end.y - before.y) <=
                                 0.0;
    if (within_ends && corridor.confirmed_forward &&
        nearest + corridor.position_error_bound < corridor.half_width &&
        std::abs(angle) < 0.5) {
      if (selected != 0) return Error("ambiguous ego lane");
      selected = lane.id;
    }
    lanes.emplace(lane.id, std::move(corridor));
  }
  for (const auto& edge : scene.edges) {
    const bool valid_type =
        edge.type == world_model::EdgeType::SUCCESSOR ||
        edge.type == world_model::EdgeType::ADJACENT_LEFT ||
        edge.type == world_model::EdgeType::ADJACENT_RIGHT ||
        edge.type == world_model::EdgeType::MERGE ||
        edge.type == world_model::EdgeType::SPLIT;
    if (!valid_type || !ValidEvidence(edge.connection) ||
        !ValidEvidence(edge.permission) || !lanes.count(edge.from) ||
        !lanes.count(edge.to) || edge.from == edge.to)
      return Error("dangling or self-referencing topology edge");
  }
  std::unordered_set<uint64_t> rule_ids;
  for (const auto& rule : scene.rules) {
    if (rule.id == 0 || !rule_ids.insert(rule.id).second ||
        !ValidEvidence(rule.evidence) ||
        (rule.type != world_model::RuleType::STOP &&
         rule.type != world_model::RuleType::SPEED_LIMIT) ||
        !lanes.count(rule.lane_id) || !std::isfinite(rule.station) ||
        rule.station < 0.0 || rule.station > Length(lanes.at(rule.lane_id)) ||
        !std::isfinite(rule.speed_mps) || rule.speed_mps < 0.0) {
      return Error("invalid rule identity, lane or station");
    }
  }
  if (selected == 0) return Error("no confirmed ego corridor");
  if (scene.navigation) {
    const auto& n = *scene.navigation;
    const auto& p = n.source;
    if (p.frame_id != s.frame_id || p.clock_id != s.clock_id ||
        p.session != s.session || p.generation != s.generation ||
        p.health != world_model::SceneHealth::HEALTHY || p.sequence == 0 ||
        !std::isfinite(p.measurement_time) ||
        !std::isfinite(p.publication_time) || !std::isfinite(p.valid_until) ||
        p.measurement_time < 0 || p.measurement_time > p.publication_time ||
        p.publication_time > now || now >= p.valid_until ||
        !lanes.count(n.from_lane) || !lanes.count(n.to_lane) ||
        n.from_lane == n.to_lane)
      return Error("invalid or expired navigation prior");
    int matches = 0;
    for (const auto& edge : scene.edges)
      matches += edge.type == world_model::EdgeType::SUCCESSOR &&
                 edge.from == n.from_lane && edge.to == n.to_lane;
    if (matches != 1)
      return Error("navigation references ambiguous or unobserved connection");
  }
  LocalCorridor result = lanes.at(selected);
  std::unordered_set<uint64_t> visited;
  double offset = 0.0;
  while (true) {
    if (!visited.insert(selected).second)
      return Error("cyclic successor graph");
    for (const auto& rule : scene.rules) {
      if (rule.lane_id != selected) continue;
      if (rule.type == world_model::RuleType::STOP &&
          rule.evidence != EvidenceState::INVALID) {
        const double stop = offset + rule.station;
        result.stop_station =
            result.stop_station ? std::min(*result.stop_station, stop) : stop;
      } else if (rule.type == world_model::RuleType::SPEED_LIMIT &&
                 rule.evidence == EvidenceState::CONFIRMED) {
        result.speed_limit = result.speed_limit
                                 ? std::min(*result.speed_limit, rule.speed_mps)
                                 : rule.speed_mps;
      } else {
        return Error("unsupported or uncertain applicable rule");
      }
    }
    const world_model::TopologyEdge* successor = nullptr;
    int exits = 0;
    for (const auto& edge : scene.edges) {
      if (edge.from != selected ||
          edge.type == world_model::EdgeType::ADJACENT_LEFT ||
          edge.type == world_model::EdgeType::ADJACENT_RIGHT)
        continue;
      ++exits;
      successor = &edge;
    }
    if (scene.navigation && scene.navigation->from_lane == selected) {
      successor = nullptr;
      for (const auto& edge : scene.edges) {
        if (edge.from == selected &&
            edge.type == world_model::EdgeType::SUCCESSOR &&
            edge.to == scene.navigation->to_lane)
          successor = &edge;
      }
      if (successor == nullptr) return Error("navigation connection missing");
      exits = 1;
    }
    if (exits != 1 || successor->type != world_model::EdgeType::SUCCESSOR ||
        successor->connection != EvidenceState::CONFIRMED ||
        successor->permission != EvidenceState::CONFIRMED)
      break;
    const auto& next = lanes.at(successor->to);
    if (!next.confirmed_forward) break;
    // A declared edge still needs a geometrically continuous join.
    if (Distance(result.centerline.back(), next.centerline.front()) > 1e-6)
      return Error("successor endpoint mismatch");
    offset = Length(result);
    result.centerline.insert(result.centerline.end(),
                             next.centerline.begin() + 1,
                             next.centerline.end());
    result.left_boundary.insert(result.left_boundary.end(),
                                next.left_boundary.begin() + 1,
                                next.left_boundary.end());
    result.right_boundary.insert(result.right_boundary.end(),
                                 next.right_boundary.begin() + 1,
                                 next.right_boundary.end());
    result.half_width = std::min(result.half_width, next.half_width);
    result.position_error_bound =
        std::max(result.position_error_bound, next.position_error_bound);
    if (result.centerline.size() > 2000)
      return Error("corridor extent exceeded");
    selected = successor->to;
  }
  *out = std::move(result);
  return common::Status::OK();
}

}  // namespace local_planning
}  // namespace apollo
