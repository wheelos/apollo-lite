/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/open_space/parking/parking_pose_selector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/polygon2d.h"

namespace apollo {
namespace planning {
namespace parking {

using apollo::common::math::Vec2d;

namespace {

constexpr double kAngledParkedBumperDepthRatio = 0.80;
constexpr double kEntranceBumperClearance = 0.30;

double Clamp(const double value, const double lower, const double upper) {
  return std::max(lower, std::min(value, upper));
}

bool IsHeadInPreference(const OpenSpaceRoiDeciderConfig& config) {
  if (config.has_parking_approach_preference()) {
    return config.parking_approach_preference() ==
           OpenSpaceRoiDeciderConfig::PARKING_APPROACH_PREFER_HEAD_IN;
  }
  return config.has_parking_inwards() && config.parking_inwards();
}

ParkingApproach DefaultAutoParkingApproach(const ParkingSlot& slot) {
  return slot.type == ParkingSlotType::kAngled ? ParkingApproach::kHeadIn
                                               : ParkingApproach::kTailIn;
}

ParkingApproach ResolveEffectiveParkingApproachPreference(
    const OpenSpaceRoiDeciderConfig& config, const ParkingSlot& slot) {
  const ParkingApproach configured_preference =
      ResolveParkingApproachPreference(config);
  if (configured_preference != ParkingApproach::kUnknown) {
    return configured_preference;
  }
  return DefaultAutoParkingApproach(slot);
}

Vec2d NormalizeOrFallback(const Vec2d& value, const Vec2d& fallback) {
  if (value.Length() > common::math::kMathEpsilon) {
    return value / value.Length();
  }
  if (fallback.Length() > common::math::kMathEpsilon) {
    return fallback / fallback.Length();
  }
  return Vec2d(1.0, 0.0);
}

Vec2d SlotDepthAxis(const ParkingSlot& slot) {
  return NormalizeOrFallback(slot.rear_center - slot.opening_center,
                             Vec2d::CreateUnitVec2d(slot.heading));
}

Vec2d SlotWidthAxis(const ParkingSlot& slot, const Vec2d& depth_axis) {
  return NormalizeOrFallback(slot.corners.right_top - slot.corners.left_top,
                             Vec2d(-depth_axis.y(), depth_axis.x()));
}

std::vector<Vec2d> SlotPolygon(const ParkingSlot& slot) {
  return {slot.corners.left_top, slot.corners.right_top,
          slot.corners.right_down, slot.corners.left_down};
}

bool VehicleFootprintInsideSlot(const std::vector<Vec2d>& slot_polygon,
                                const apollo::common::VehicleParam& vehicle_param,
                                const std::vector<double>& end_pose) {
  if (slot_polygon.size() < 3U || end_pose.size() < 3U) {
    return false;
  }
  const double shift_distance =
      0.5 * vehicle_param.length() - vehicle_param.back_edge_to_center();
  const Vec2d box_center(end_pose[0] +
                             shift_distance * std::cos(end_pose[2]),
                         end_pose[1] +
                             shift_distance * std::sin(end_pose[2]));
  const apollo::common::math::Box2d vehicle_box(
      box_center, end_pose[2], vehicle_param.length(), vehicle_param.width());
  return apollo::common::math::Polygon2d(slot_polygon)
      .Contains(apollo::common::math::Polygon2d(vehicle_box));
}

bool FitCandidateFootprintInsideSlot(
    const ParkingSlot& slot, const apollo::common::VehicleParam& vehicle_param,
    ParkingPoseCandidate* candidate) {
  if (candidate == nullptr || candidate->end_pose.size() < 3U) {
    return false;
  }
  const std::vector<Vec2d> slot_polygon = SlotPolygon(slot);
  if (VehicleFootprintInsideSlot(slot_polygon, vehicle_param,
                                 candidate->end_pose)) {
    return true;
  }

  const double heading = candidate->end_pose[2];
  const Vec2d longitudinal_axis(std::cos(heading), std::sin(heading));
  const Vec2d lateral_axis(-std::sin(heading), std::cos(heading));
  const double lateral_limit =
      std::max(0.5, 0.5 * std::max(0.0, slot.width - vehicle_param.width()));
  const double longitudinal_limit = std::min(0.75, 0.15 * slot.depth);
  constexpr double kSearchStep = 0.05;

  std::vector<double> best_pose = candidate->end_pose;
  double best_cost = std::numeric_limits<double>::infinity();
  for (double longitudinal_offset = -longitudinal_limit;
       longitudinal_offset <= longitudinal_limit + common::math::kMathEpsilon;
       longitudinal_offset += kSearchStep) {
    for (double lateral_offset = -lateral_limit;
         lateral_offset <= lateral_limit + common::math::kMathEpsilon;
         lateral_offset += kSearchStep) {
      std::vector<double> shifted_pose = candidate->end_pose;
      const Vec2d shifted_xy =
          Vec2d(candidate->end_pose[0], candidate->end_pose[1]) +
          longitudinal_axis * longitudinal_offset + lateral_axis * lateral_offset;
      shifted_pose[0] = shifted_xy.x();
      shifted_pose[1] = shifted_xy.y();
      if (!VehicleFootprintInsideSlot(slot_polygon, vehicle_param,
                                      shifted_pose)) {
        continue;
      }
      const double cost = longitudinal_offset * longitudinal_offset +
                          lateral_offset * lateral_offset;
      if (cost < best_cost) {
        best_cost = cost;
        best_pose = std::move(shifted_pose);
      }
    }
  }
  if (!std::isfinite(best_cost)) {
    return false;
  }
  candidate->end_pose = std::move(best_pose);
  return true;
}

std::vector<ParkingPoseCandidate> GenerateCandidates(
    const ParkingSlot& slot, const apollo::common::VehicleParam& vehicle_param,
    const OpenSpaceRoiDeciderConfig& config) {
  std::vector<ParkingPoseCandidate> candidates;
  const Vec2d depth_axis = SlotDepthAxis(slot);
  const Vec2d width_axis = SlotWidthAxis(slot, depth_axis);

  if (slot.type == ParkingSlotType::kParallel) {
    ParkingPoseCandidate tail_in;
    tail_in.approach = ParkingApproach::kTailIn;
    const double parallel_x = vehicle_param.back_edge_to_center() +
                              config.parallel_park_end_x_buffer();
    const Vec2d tail_reference_point = slot.corners.left_top +
                                       width_axis * parallel_x +
                                       depth_axis * (slot.depth * 0.5);
    tail_in.end_pose = {tail_reference_point.x(), tail_reference_point.y(),
                        width_axis.Angle(), 0.0};
    candidates.push_back(std::move(tail_in));

    ParkingPoseCandidate head_in;
    head_in.approach = ParkingApproach::kHeadIn;
    const Vec2d head_reference_point =
        slot.corners.left_top +
        width_axis * (slot.width - vehicle_param.front_edge_to_center()) +
        depth_axis * (slot.depth * 0.5);
    head_in.end_pose = {
        head_reference_point.x(), head_reference_point.y(),
        common::math::NormalizeAngle(width_axis.Angle() + M_PI), 0.0};
    candidates.push_back(std::move(head_in));
    return candidates;
  }

  const double parking_depth_buffer =
      std::max(0.0, config.parking_depth_buffer());
  const double depth_slack = slot.depth - vehicle_param.length();
  const double rear_depth_buffer =
      std::min(parking_depth_buffer,
               std::max(0.0, depth_slack - kEntranceBumperClearance));
  const double head_in_parked_bumper_depth =
      slot.type == ParkingSlotType::kAngled
          ? Clamp(kAngledParkedBumperDepthRatio * slot.depth,
                  0.6 * slot.depth, slot.depth - rear_depth_buffer)
          : slot.depth - rear_depth_buffer;
  const double tail_in_parked_bumper_depth = slot.depth - rear_depth_buffer;
  const double head_in_depth =
      Clamp(head_in_parked_bumper_depth - vehicle_param.front_edge_to_center(),
            -0.25 * slot.depth, slot.depth);
  const double tail_in_depth =
      Clamp(tail_in_parked_bumper_depth - vehicle_param.back_edge_to_center(),
            0.0, slot.depth);

  ParkingPoseCandidate head_in;
  head_in.approach = ParkingApproach::kHeadIn;
  const Vec2d head_reference_point =
      slot.opening_center + depth_axis * head_in_depth;
  head_in.end_pose = {head_reference_point.x(), head_reference_point.y(),
                      depth_axis.Angle(), 0.0};
  candidates.push_back(std::move(head_in));

  ParkingPoseCandidate tail_in;
  tail_in.approach = ParkingApproach::kTailIn;
  const Vec2d tail_reference_point =
      slot.opening_center + depth_axis * tail_in_depth;
  tail_in.end_pose = {tail_reference_point.x(), tail_reference_point.y(),
                      common::math::NormalizeAngle(depth_axis.Angle() + M_PI),
                      0.0};
  candidates.push_back(std::move(tail_in));
  return candidates;
}

int FindCandidateIndex(const std::vector<ParkingPoseCandidate>& candidates,
                       const ParkingApproach preferred) {
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (candidates[index].approach == preferred) {
      return static_cast<int>(index);
    }
  }
  return candidates.empty() ? -1 : 0;
}

}  // namespace

ParkingApproach ResolveParkingApproachPreference(
    const OpenSpaceRoiDeciderConfig& config) {
  if (config.has_parking_approach_preference()) {
    switch (config.parking_approach_preference()) {
      case OpenSpaceRoiDeciderConfig::PARKING_APPROACH_PREFER_HEAD_IN:
        return ParkingApproach::kHeadIn;
      case OpenSpaceRoiDeciderConfig::PARKING_APPROACH_PREFER_TAIL_IN:
        return ParkingApproach::kTailIn;
      case OpenSpaceRoiDeciderConfig::PARKING_APPROACH_AUTO:
      default:
        break;
    }
  }
  if (config.has_parking_inwards()) {
    return IsHeadInPreference(config) ? ParkingApproach::kHeadIn
                                      : ParkingApproach::kTailIn;
  }
  return ParkingApproach::kUnknown;
}

const char* ParkingApproachName(const ParkingApproach approach) {
  switch (approach) {
    case ParkingApproach::kHeadIn:
      return "head_in";
    case ParkingApproach::kTailIn:
      return "tail_in";
    case ParkingApproach::kUnknown:
    default:
      return "unknown";
  }
}

ParkingPoseSelector::ParkingPoseSelector(
    const OpenSpaceRoiDeciderConfig& config)
    : config_(config) {}

ParkingPoseSelection ParkingPoseSelector::Select(
    const ParkingSlot& normalized_slot, const ParkingRoiGeometry& roi_geometry,
    const apollo::common::VehicleParam& vehicle_param,
    const Vec2d& vehicle_position, const double vehicle_heading) const {
  static_cast<void>(vehicle_position);
  static_cast<void>(vehicle_heading);

  ParkingPoseSelection selection;
  selection.candidates =
      GenerateCandidates(normalized_slot, vehicle_param, config_);
  const ParkingApproach preferred =
      ResolveEffectiveParkingApproachPreference(config_, normalized_slot);
  const int preferred_index = FindCandidateIndex(selection.candidates, preferred);

  for (std::size_t index = 0; index < selection.candidates.size(); ++index) {
    auto& candidate = selection.candidates[index];
    candidate.aisle_width = roi_geometry.aisle_width;
    if (!FitCandidateFootprintInsideSlot(normalized_slot, vehicle_param,
                                         &candidate)) {
      candidate.feasible = false;
      candidate.rejection_reason =
          "goal vehicle footprint cannot fit inside parking slot";
      continue;
    }
    candidate.feasible = true;
    candidate.rejection_reason = "not selected by parking approach policy";
    if (static_cast<int>(index) == preferred_index) {
      selection.selected_index = preferred_index;
    }
  }
  if (selection.selected_index < 0) {
    for (std::size_t index = 0; index < selection.candidates.size(); ++index) {
      if (selection.candidates[index].feasible) {
        selection.selected_index = static_cast<int>(index);
        break;
      }
    }
  }
  if (!selection.has_feasible_candidate()) {
    return selection;
  }

  auto& selected = selection.candidates[selection.selected_index];
  selected.was_probed = true;
  selected.feasible = true;
  selected.score = 0.0;
  selected.path_length = 0.0;
  selected.reverse_distance = 0.0;
  selected.min_clearance = 0.0;
  selected.gear_switch_count = 0;
  selected.rejection_reason.clear();

  AINFO << "Parking pose candidate selected for slot " << normalized_slot.id
        << " type=" << ParkingSlotTypeName(normalized_slot.type)
        << " approach=" << ParkingApproachName(selected.approach)
        << " end_pose=(" << selected.end_pose[0] << ", "
        << selected.end_pose[1] << ", " << selected.end_pose[2] << ")"
        << " deterministic_policy=1";
  return selection;
}

}  // namespace parking
}  // namespace planning
}  // namespace apollo
