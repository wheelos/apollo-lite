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

#include "modules/prediction/evaluator/vehicle/hivt_scene_feature_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cyber/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

namespace {

constexpr float kLocalRadius = 50.0F;

struct SceneLane {
  hdmap::LaneInfoConstPtr lane;
  double distance = std::numeric_limits<double>::max();
};

common::math::Vec2d WorldToScene(const common::math::Vec2d& point,
                                 double origin_x, double origin_y,
                                 double cos_heading, double sin_heading) {
  const double dx = point.x() - origin_x;
  const double dy = point.y() - origin_y;
  return {dx * cos_heading + dy * sin_heading,
          -dx * sin_heading + dy * cos_heading};
}

double SquaredDistance(const Obstacle* left, const Obstacle* right) {
  if (left == nullptr || right == nullptr ||
      !left->latest_feature().IsInitialized() ||
      !right->latest_feature().IsInitialized()) {
    return std::numeric_limits<double>::max();
  }
  const auto& left_position = left->latest_feature().position();
  const auto& right_position = right->latest_feature().position();
  const double dx = left_position.x() - right_position.x();
  const double dy = left_position.y() - right_position.y();
  return dx * dx + dy * dy;
}

bool HasHiVTHistory(const Obstacle* actor) {
  if (actor == nullptr || actor->history_size() < 2 ||
      !actor->latest_feature().IsInitialized() ||
      FLAGS_prediction_trajectory_time_resolution <= 0.0) {
    return false;
  }
  std::vector<bool> valid_steps(kHiVTHistoricalSteps, false);
  const double latest_timestamp = actor->latest_feature().timestamp();
  for (std::size_t index = 0;
       index < actor->history_size() &&
       index < static_cast<std::size_t>(kHiVTHistoricalSteps);
       ++index) {
    const Feature& feature = actor->feature(index);
    if (!feature.IsInitialized()) {
      continue;
    }
    const double elapsed = latest_timestamp - feature.timestamp();
    const int age = static_cast<int>(
        std::llround(elapsed / FLAGS_prediction_trajectory_time_resolution));
    if (age >= 0 && age < kHiVTHistoricalSteps) {
      valid_steps[age] = true;
    }
  }
  return valid_steps.front() &&
         std::count(valid_steps.begin(), valid_steps.end(), true) >= 2;
}

int TurnDirection(const hdmap::Lane& lane) {
  switch (lane.turn()) {
    case hdmap::Lane::LEFT_TURN:
      return 1;
    case hdmap::Lane::RIGHT_TURN:
      return 2;
    default:
      return 0;
  }
}

}  // namespace

std::vector<Obstacle*> HiVTSceneFeatureBuilder::SelectTargets(
    const Obstacle* ego, const std::vector<Obstacle*>& candidates) {
  std::vector<Obstacle*> selected;
  if (ego == nullptr) {
    return selected;
  }
  for (Obstacle* candidate : candidates) {
    if (candidate != nullptr && candidate->id() != ego->id() &&
        HasHiVTHistory(candidate)) {
      selected.push_back(candidate);
    }
  }
  std::sort(selected.begin(), selected.end(),
            [ego](const Obstacle* left, const Obstacle* right) {
              const double left_distance = SquaredDistance(ego, left);
              const double right_distance = SquaredDistance(ego, right);
              if (left_distance != right_distance) {
                return left_distance < right_distance;
              }
              return left->id() < right->id();
            });
  if (selected.size() > kHiVTMaxActors - 1) {
    selected.resize(kHiVTMaxActors - 1);
  }
  return selected;
}

std::vector<Obstacle*> HiVTSceneFeatureBuilder::SelectActors(
    Obstacle* ego, const std::vector<Obstacle*>& targets,
    const std::vector<Obstacle*>& candidates) {
  std::vector<Obstacle*> actors;
  if (ego == nullptr) {
    return actors;
  }
  actors.reserve(kHiVTMaxActors);
  std::unordered_set<int> selected_ids;
  const auto add_mandatory = [&actors, &selected_ids](Obstacle* actor) {
    if (actor != nullptr && selected_ids.insert(actor->id()).second) {
      actors.push_back(actor);
    }
  };
  add_mandatory(ego);
  for (Obstacle* target : targets) {
    add_mandatory(target);
  }
  if (actors.size() > kHiVTMaxActors) {
    AERROR << "HiVT selected targets exceed actor capacity";
    return {};
  }

  std::vector<Obstacle*> context;
  context.reserve(candidates.size());
  for (Obstacle* actor : candidates) {
    if (actor != nullptr && actor->id() != ego->id() && HasHiVTHistory(actor) &&
        selected_ids.count(actor->id()) == 0) {
      context.push_back(actor);
    }
  }
  const auto nearest_target_distance = [&targets, ego](const Obstacle* actor) {
    double distance = SquaredDistance(ego, actor);
    for (const Obstacle* target : targets) {
      distance = std::min(distance, SquaredDistance(target, actor));
    }
    return distance;
  };
  std::sort(
      context.begin(), context.end(),
      [&nearest_target_distance](const Obstacle* left, const Obstacle* right) {
        const double left_distance = nearest_target_distance(left);
        const double right_distance = nearest_target_distance(right);
        if (left_distance != right_distance) {
          return left_distance < right_distance;
        }
        return left->id() < right->id();
      });
  for (Obstacle* actor : context) {
    if (actors.size() == kHiVTMaxActors) {
      break;
    }
    add_mandatory(actor);
  }
  std::sort(actors.begin(), actors.end(),
            [](const Obstacle* left, const Obstacle* right) {
              return left->id() < right->id();
            });
  return actors;
}

bool HiVTSceneFeatureBuilder::Build(const Obstacle* ego,
                                    const std::vector<Obstacle*>& actors,
                                    HiVTSceneInput* input) const {
  if (input == nullptr || ego == nullptr || ego->history_size() < 2 ||
      actors.size() < 2 || actors.size() > kHiVTMaxActors ||
      std::abs(FLAGS_prediction_trajectory_time_resolution -
               kHiVTTimeStepSeconds) > 1e-6) {
    AERROR << "Invalid HiVT scene input: actors=" << actors.size();
    return false;
  }

  const Feature& ego_latest = ego->latest_feature();
  const Feature& ego_previous = ego->feature(1);
  if (!ego_latest.IsInitialized() || !ego_previous.IsInitialized()) {
    AERROR << "HiVT scene ego history is not initialized";
    return false;
  }
  input->origin_x = ego_latest.position().x();
  input->origin_y = ego_latest.position().y();
  const double ego_heading =
      std::atan2(ego_latest.position().y() - ego_previous.position().y(),
                 ego_latest.position().x() - ego_previous.position().x());
  input->origin_cos = std::cos(ego_heading);
  input->origin_sin = std::sin(ego_heading);

  input->actor_count = static_cast<int>(actors.size());
  input->actor_ids.clear();
  input->x.assign(actors.size() * kHiVTHistoricalSteps * 2, 0.0F);
  input->positions.assign(
      actors.size() * (kHiVTHistoricalSteps + kHiVTFutureSteps) * 2, 0.0F);
  input->padding_mask.assign(
      actors.size() * (kHiVTHistoricalSteps + kHiVTFutureSteps), 1);
  input->bos_mask.assign(actors.size() * kHiVTHistoricalSteps, 0);
  input->rotate_angles.resize(actors.size());

  std::vector<common::math::Vec2d> current_positions;
  std::vector<common::math::Vec2d> world_positions;
  current_positions.reserve(actors.size());
  world_positions.reserve(actors.size());
  std::unordered_set<int> actor_ids;
  for (std::size_t actor_index = 0; actor_index < actors.size();
       ++actor_index) {
    const Obstacle* actor = actors[actor_index];
    if (actor == nullptr || actor->history_size() == 0 ||
        !actor->latest_feature().IsInitialized() ||
        !actor_ids.insert(actor->id()).second) {
      AERROR << "HiVT scene contains an invalid actor at index " << actor_index;
      return false;
    }
    input->actor_ids.push_back(actor->id());
    const Feature& latest = actor->latest_feature();
    const common::math::Vec2d world_position(latest.position().x(),
                                             latest.position().y());
    world_positions.push_back(world_position);
    current_positions.push_back(WorldToScene(world_position, input->origin_x,
                                             input->origin_y, input->origin_cos,
                                             input->origin_sin));

    const double latest_timestamp = latest.timestamp();
    for (std::size_t history_index = 0;
         history_index < actor->history_size() &&
         history_index < static_cast<std::size_t>(kHiVTHistoricalSteps);
         ++history_index) {
      const Feature& feature = actor->feature(history_index);
      if (!feature.IsInitialized()) {
        continue;
      }
      const double elapsed = latest_timestamp - feature.timestamp();
      const int age = static_cast<int>(
          std::llround(elapsed / FLAGS_prediction_trajectory_time_resolution));
      if (age < 0 || age >= kHiVTHistoricalSteps) {
        continue;
      }
      const int time_index = kHiVTHistoricalSteps - 1 - age;
      const std::size_t mask_index =
          actor_index * (kHiVTHistoricalSteps + kHiVTFutureSteps) +
          static_cast<std::size_t>(time_index);
      if (input->padding_mask[mask_index] == 0) {
        continue;
      }
      input->padding_mask[mask_index] = 0;
      const common::math::Vec2d scene_position = WorldToScene(
          {feature.position().x(), feature.position().y()}, input->origin_x,
          input->origin_y, input->origin_cos, input->origin_sin);
      const std::size_t position_index =
          (actor_index * (kHiVTHistoricalSteps + kHiVTFutureSteps) +
           static_cast<std::size_t>(time_index)) *
          2;
      input->positions[position_index] = static_cast<float>(scene_position.x());
      input->positions[position_index + 1] =
          static_cast<float>(scene_position.y());
    }

    const std::size_t history_offset = actor_index * kHiVTHistoricalSteps;
    bool previous_valid = false;
    float previous_x = 0.0F;
    float previous_y = 0.0F;
    for (int time_index = 0; time_index < kHiVTHistoricalSteps; ++time_index) {
      const std::size_t mask_index =
          actor_index * (kHiVTHistoricalSteps + kHiVTFutureSteps) +
          static_cast<std::size_t>(time_index);
      if (input->padding_mask[mask_index] != 0) {
        previous_valid = false;
        continue;
      }
      if (!previous_valid) {
        input->bos_mask[history_offset + static_cast<std::size_t>(time_index)] =
            1;
      } else {
        const std::size_t position_index = mask_index * 2;
        const std::size_t motion_index =
            (history_offset + static_cast<std::size_t>(time_index)) * 2;
        const float x = input->positions[position_index];
        const float y = input->positions[position_index + 1];
        input->x[motion_index] = x - previous_x;
        input->x[motion_index + 1] = y - previous_y;
      }
      const std::size_t position_index = mask_index * 2;
      previous_x = input->positions[position_index];
      previous_y = input->positions[position_index + 1];
      previous_valid = true;
    }

    int latest_valid_step = -1;
    int previous_valid_step = -1;
    for (int time_index = kHiVTHistoricalSteps - 1; time_index >= 0;
         --time_index) {
      const std::size_t mask_index =
          actor_index * (kHiVTHistoricalSteps + kHiVTFutureSteps) +
          static_cast<std::size_t>(time_index);
      if (input->padding_mask[mask_index] == 0) {
        if (latest_valid_step < 0) {
          latest_valid_step = time_index;
        } else {
          previous_valid_step = time_index;
          break;
        }
      }
    }
    if (latest_valid_step != kHiVTHistoricalSteps - 1 ||
        previous_valid_step < 0) {
      AERROR << "HiVT actor needs two valid history samples and a current "
                "sample: "
             << actor->id();
      return false;
    }
    const std::size_t latest_position_index =
        (actor_index * (kHiVTHistoricalSteps + kHiVTFutureSteps) +
         static_cast<std::size_t>(latest_valid_step)) *
        2;
    const std::size_t previous_position_index =
        (actor_index * (kHiVTHistoricalSteps + kHiVTFutureSteps) +
         static_cast<std::size_t>(previous_valid_step)) *
        2;
    input->rotate_angles[actor_index] = static_cast<float>(
        std::atan2(input->positions[latest_position_index + 1] -
                       input->positions[previous_position_index + 1],
                   input->positions[latest_position_index] -
                       input->positions[previous_position_index]));
    for (int future_step = kHiVTHistoricalSteps;
         future_step < kHiVTHistoricalSteps + kHiVTFutureSteps; ++future_step) {
      input->padding_mask[actor_index *
                              (kHiVTHistoricalSteps + kHiVTFutureSteps) +
                          static_cast<std::size_t>(future_step)] = 0;
    }
  }

  input->edge_index.clear();
  for (int source = 0; source < input->actor_count; ++source) {
    for (int target = 0; target < input->actor_count; ++target) {
      if (source != target) {
        input->edge_index.push_back(source);
      }
    }
  }
  for (int source = 0; source < input->actor_count; ++source) {
    for (int target = 0; target < input->actor_count; ++target) {
      if (source != target) {
        input->edge_index.push_back(target);
      }
    }
  }
  input->actor_edge_count = input->actor_count * (input->actor_count - 1);

  std::unordered_map<std::string, SceneLane> nearby_lanes;
  for (std::size_t actor_index = 0; actor_index < world_positions.size();
       ++actor_index) {
    const auto& position = world_positions[actor_index];
    common::PointENU map_position;
    map_position.set_x(position.x());
    map_position.set_y(position.y());
    std::vector<hdmap::LaneInfoConstPtr> lanes;
    if (hdmap::HDMapUtil::BaseMap().GetLanes(map_position, kLocalRadius,
                                             &lanes) != 0) {
      AERROR << "Failed to query nearby lanes for HiVT scene";
      return false;
    }
    for (const auto& lane : lanes) {
      if (lane == nullptr) {
        continue;
      }
      const double distance = lane->DistanceTo(position);
      auto [it, inserted] =
          nearby_lanes.emplace(lane->id().id(), SceneLane{lane, distance});
      if (!inserted) {
        it->second.distance = std::min(it->second.distance, distance);
      }
    }
  }

  std::vector<SceneLane> selected_lanes;
  selected_lanes.reserve(nearby_lanes.size());
  for (const auto& entry : nearby_lanes) {
    selected_lanes.push_back(entry.second);
  }
  std::sort(selected_lanes.begin(), selected_lanes.end(),
            [](const SceneLane& left, const SceneLane& right) {
              if (left.distance != right.distance) {
                return left.distance < right.distance;
              }
              return left.lane->id().id() < right.lane->id().id();
            });
  const std::size_t nearby_lane_count = selected_lanes.size();
  if (nearby_lane_count > kHiVTMaxLanePolylines) {
    selected_lanes.resize(kHiVTMaxLanePolylines);
    AWARN << "HiVT scene keeps the nearest " << kHiVTMaxLanePolylines << " of "
          << nearby_lane_count << " nearby lanes";
  }

  input->lane_vectors.clear();
  input->is_intersections.clear();
  input->turn_directions.clear();
  input->traffic_controls.clear();
  std::vector<int64_t> lane_rows;
  std::vector<int64_t> actor_cols;
  input->lane_actor_vectors.clear();

  int capacity_skipped_lanes = 0;
  for (const auto& selected : selected_lanes) {
    const auto& lane = selected.lane;
    const auto& distances = lane->accumulate_s();
    const uint8_t is_intersection = lane->junctions().empty() ? 0 : 1;
    const uint8_t turn = static_cast<uint8_t>(TurnDirection(lane->lane()));
    const uint8_t has_traffic_control =
        (!lane->signals().empty() || !lane->stop_signs().empty() ||
         !lane->yield_signs().empty())
            ? 1
            : 0;
    struct LaneSegment {
      double dx = 0.0;
      double dy = 0.0;
      common::math::Vec2d start;
      std::vector<int> nearby_actor_indices;
    };
    std::vector<LaneSegment> lane_segments;
    for (std::size_t segment = 0; segment + 1 < distances.size(); ++segment) {
      const auto start = lane->GetSmoothPoint(distances[segment]);
      const auto end = lane->GetSmoothPoint(distances[segment + 1]);
      const double lane_dx = end.x() - start.x();
      const double lane_dy = end.y() - start.y();
      std::vector<int> nearby_actor_indices;
      nearby_actor_indices.reserve(actors.size());
      for (std::size_t actor_index = 0; actor_index < actors.size();
           ++actor_index) {
        const double dx = start.x() - world_positions[actor_index].x();
        const double dy = start.y() - world_positions[actor_index].y();
        if (std::hypot(dx, dy) <= kLocalRadius) {
          nearby_actor_indices.push_back(static_cast<int>(actor_index));
        }
      }

      if (nearby_actor_indices.empty()) {
        continue;
      }
      lane_segments.push_back({lane_dx,
                               lane_dy,
                               {start.x(), start.y()},
                               std::move(nearby_actor_indices)});
    }
    if (lane_segments.size() > static_cast<std::size_t>(kHiVTMaxLaneVectors) -
                                   input->lane_vectors.size() / 2) {
      ++capacity_skipped_lanes;
      continue;
    }
    for (const auto& segment : lane_segments) {
      const int lane_vector_index =
          static_cast<int>(input->lane_vectors.size() / 2);
      input->lane_vectors.push_back(static_cast<float>(
          segment.dx * input->origin_cos + segment.dy * input->origin_sin));
      input->lane_vectors.push_back(static_cast<float>(
          -segment.dx * input->origin_sin + segment.dy * input->origin_cos));
      input->is_intersections.push_back(is_intersection);
      input->turn_directions.push_back(turn);
      input->traffic_controls.push_back(has_traffic_control);
      const common::math::Vec2d scene_start =
          WorldToScene(segment.start, input->origin_x, input->origin_y,
                       input->origin_cos, input->origin_sin);
      for (const int actor_index : segment.nearby_actor_indices) {
        lane_rows.push_back(lane_vector_index);
        actor_cols.push_back(actor_index);
        input->lane_actor_vectors.push_back(static_cast<float>(
            scene_start.x() - current_positions[actor_index].x()));
        input->lane_actor_vectors.push_back(static_cast<float>(
            scene_start.y() - current_positions[actor_index].y()));
      }
    }
  }
  if (capacity_skipped_lanes > 0) {
    AWARN << "HiVT omitted " << capacity_skipped_lanes
          << " nearby lane polylines to stay within the " << kHiVTMaxLaneVectors
          << " lane-vector capacity";
  }

  input->lane_vector_count = static_cast<int>(input->lane_vectors.size() / 2);
  input->lane_actor_edge_count = static_cast<int>(lane_rows.size());
  if (input->lane_vector_count == 0 || input->lane_actor_edge_count == 0) {
    AERROR << "HiVT scene has no nearby lane-vector relations";
    return false;
  }
  input->lane_actor_index.clear();
  input->lane_actor_index.reserve(lane_rows.size() + actor_cols.size());
  input->lane_actor_index.insert(input->lane_actor_index.end(),
                                 lane_rows.begin(), lane_rows.end());
  input->lane_actor_index.insert(input->lane_actor_index.end(),
                                 actor_cols.begin(), actor_cols.end());
  return true;
}

}  // namespace prediction
}  // namespace apollo
