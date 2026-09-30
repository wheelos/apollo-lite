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

#include <cstdint>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "modules/world_model/local_map/lane_types.h"

namespace apollo {
namespace world_model {

enum class EvidenceState { UNKNOWN, HYPOTHESIS, CONFIRMED, INVALID };
enum class SceneHealth { UNKNOWN, HEALTHY, INVALID };
enum class SceneMode { INVALID, AREA, LANE };
enum class EdgeType { SUCCESSOR, ADJACENT_LEFT, ADJACENT_RIGHT, MERGE, SPLIT };
enum class RuleType { STOP, SPEED_LIMIT };
enum class VehicleReference { UNKNOWN, REAR_AXLE };

struct SceneSource {
  std::string frame_id;
  std::string clock_id;
  std::string session;
  uint64_t generation = 0;
  uint64_t sequence = 0;
  double measurement_time = 0.0;
  double publication_time = 0.0;
  double valid_until = 0.0;
  SceneHealth health = SceneHealth::UNKNOWN;
  bool operator==(const SceneSource& s) const {
    return std::tie(frame_id, clock_id, session, generation, sequence,
                    measurement_time, publication_time, valid_until, health) ==
           std::tie(s.frame_id, s.clock_id, s.session, s.generation, s.sequence,
                    s.measurement_time, s.publication_time, s.valid_until,
                    s.health);
  }
};

struct LaneBoundary {
  uint64_t id = 0;
  std::vector<LanePoint> points;
  EvidenceState crossing = EvidenceState::UNKNOWN;
  std::vector<double> observation_times;
  std::vector<double> valid_until;
  bool operator==(const LaneBoundary& b) const {
    return std::tie(id, points, crossing, observation_times, valid_until) ==
           std::tie(b.id, b.points, b.crossing, b.observation_times,
                    b.valid_until);
  }
};

struct LaneSegment {
  uint64_t id = 0;
  uint64_t left_boundary = 0;
  uint64_t right_boundary = 0;
  EvidenceState forward = EvidenceState::UNKNOWN;
  double position_error = 0.0;
  bool operator==(const LaneSegment& l) const {
    return std::tie(id, left_boundary, right_boundary, forward,
                    position_error) == std::tie(l.id, l.left_boundary,
                                                l.right_boundary, l.forward,
                                                l.position_error);
  }
};

struct TopologyEdge {
  uint64_t from = 0;
  uint64_t to = 0;
  EdgeType type = EdgeType::SUCCESSOR;
  EvidenceState connection = EvidenceState::UNKNOWN;
  EvidenceState permission = EvidenceState::UNKNOWN;
  bool operator==(const TopologyEdge& e) const {
    return std::tie(from, to, type, connection, permission) ==
           std::tie(e.from, e.to, e.type, e.connection, e.permission);
  }
};

struct LocalRule {
  uint64_t id = 0;
  uint64_t lane_id = 0;
  RuleType type = RuleType::STOP;
  EvidenceState evidence = EvidenceState::UNKNOWN;
  double station = 0.0;
  double speed_mps = 0.0;
  bool operator==(const LocalRule& r) const {
    return std::tie(id, lane_id, type, evidence, station, speed_mps) ==
           std::tie(r.id, r.lane_id, r.type, r.evidence, r.station,
                    r.speed_mps);
  }
};

struct SceneCapabilities {
  bool drivable_region = false;
  bool lane_follow = false;
  bool lane_change = false;
  bool intersection = false;
  bool operator==(const SceneCapabilities& c) const {
    return std::tie(drivable_region, lane_follow, lane_change, intersection) ==
           std::tie(c.drivable_region, c.lane_follow, c.lane_change,
                    c.intersection);
  }
};

// Advisory route choice among observed local lane IDs, never map geometry.
struct NavigationPrior {
  SceneSource source;
  uint64_t from_lane = 0;
  uint64_t to_lane = 0;
  bool operator==(const NavigationPrior& n) const {
    return std::tie(source, from_lane, to_lane) ==
           std::tie(n.source, n.from_lane, n.to_lane);
  }
};

struct LaneLayer {
  SceneSource source;
  std::vector<LaneBoundary> boundaries;
  std::vector<LaneSegment> lanes;
  std::vector<TopologyEdge> edges;
  std::vector<LocalRule> rules;
};

struct ObservedPolyline {
  uint64_t id = 0;
  std::vector<LanePoint> points;
  bool operator==(const ObservedPolyline& p) const {
    return std::tie(id, points) == std::tie(p.id, p.points);
  }
};

struct ObservedPolygon {
  uint64_t id = 0;
  std::vector<LanePoint> points;
  bool operator==(const ObservedPolygon& p) const {
    return std::tie(id, points) == std::tie(p.id, p.points);
  }
};

// One bounded observation packet in source.frame_id. Free space is affirmative
// measured coverage; geometry outside it remains unknown.
struct EnvironmentObservation {
  std::string session;
  uint64_t generation = 0;
  uint64_t sequence = 0;
  double measurement_time = 0.0;
  double position_error = 0.0;
  std::vector<LanePoint> free_space;
  std::vector<ObservedPolyline> curbs;
  std::vector<ObservedPolygon> obstacles;
};

// Vehicle-independent ODOM geometry. The planner owns vehicle expansion.
struct DrivableEnvironment {
  SceneSource source;
  double position_error = 0.0;
  std::vector<LanePoint> free_space;
  std::vector<ObservedPolyline> curbs;
  std::vector<ObservedPolygon> obstacles;
  bool operator==(const DrivableEnvironment& e) const {
    return std::tie(source, position_error, free_space, curbs, obstacles) ==
           std::tie(e.source, e.position_error, e.free_space, e.curbs,
                    e.obstacles);
  }
};

// Owned, immutable after publication. Geometry is in source.frame_id.
// Rules describe observations; permission remains distinct from connectivity.
struct LocalScene {
  SceneSource source;
  SceneMode mode = SceneMode::INVALID;
  VehicleReference reference = VehicleReference::UNKNOWN;
  SceneCapabilities capabilities;
  std::vector<LaneBoundary> boundaries;
  std::vector<LaneSegment> lanes;
  std::vector<TopologyEdge> edges;
  std::vector<LocalRule> rules;
  DrivableEnvironment environment;
  std::optional<NavigationPrior> navigation;
  std::string mode_reason;
  std::string invalid_reason;
  bool operator==(const LocalScene& s) const {
    return std::tie(source, mode, reference, capabilities, boundaries, lanes,
                    edges, rules, environment, navigation, mode_reason,
                    invalid_reason) ==
           std::tie(s.source, s.mode, s.reference, s.capabilities, s.boundaries,
                    s.lanes, s.edges, s.rules, s.environment, s.navigation,
                    s.mode_reason, s.invalid_reason);
  }
};

}  // namespace world_model
}  // namespace apollo
