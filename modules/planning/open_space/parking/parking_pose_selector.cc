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
#include <memory>
#include <tuple>
#include <utility>

#include "modules/common/math/box2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"
#include "modules/planning/open_space/coarse_trajectory_generator/node3d.h"
#include "modules/planning/open_space/coarse_trajectory_generator/reeds_shepp_path.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"

namespace apollo {
namespace planning {
namespace parking {

using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

namespace {

constexpr double kParkedBumperDepthRatio = 0.75;

struct VariantSearchWindow {
  double nominal_depth = 0.0;
  double nominal_lateral = 0.0;
  double nominal_heading = 0.0;
  double depth_lower_bound = -0.5;
  double depth_upper_bound = 0.5;
  double lateral_lower_bound = -0.5;
  double lateral_upper_bound = 0.5;
  double depth_step = 0.1;
  double lateral_step = 0.1;
  double heading_step = 0.01;
  int heading_steps_per_side = 2;
};

class ProbeReedShepp : public ReedShepp {
 public:
  ProbeReedShepp(const common::VehicleParam& vehicle_param,
                 const PlannerOpenSpaceConfig& open_space_conf)
      : ReedShepp(vehicle_param, open_space_conf) {}

  using ReedShepp::GenerateRSPs;
  using ReedShepp::GenerateLocalConfigurations;
};

double ComputeBoxBoundaryClearance(const Box2d& ego_box,
                                   const Polygon2d& polygon);

bool BoxOverlapsBoundarySegments(const Box2d& ego_box,
                                 const ParkingRoiGeometry& roi_geometry,
                                 int* overlap_index,
                                 Vec2d* overlap_start,
                                 Vec2d* overlap_end);

double ComputeReedSheppPathCost(const std::shared_ptr<Node3d>& start_node,
                                const ReedSheppPath& path,
                                const double traj_forward_penalty,
                                const double traj_back_penalty,
                                const double traj_gear_switch_penalty,
                                const double traj_steer_penalty) {
  const double start_dire = start_node->GetDirec() ? 1.0 : -1.0;
  double cost = 0.0;
  for (std::size_t index = 0; index < path.segs_lengths.size(); ++index) {
    const double segment_length = path.segs_lengths[index];
    if (path.segs_types[index] == 'S') {
      cost += segment_length < 0.0 ? -segment_length * traj_back_penalty
                                   : segment_length * traj_forward_penalty;
    } else {
      cost += segment_length < 0.0
                  ? -segment_length * traj_steer_penalty * traj_back_penalty
                  : segment_length * traj_steer_penalty * traj_forward_penalty;
    }
    if (index > 0 && segment_length * path.segs_lengths[index - 1U] < 0.0) {
      cost += cost + traj_gear_switch_penalty;
    }
    if (index == 0 && start_dire * segment_length < 0.0) {
      cost += cost + traj_gear_switch_penalty;
    }
    if (std::fabs(segment_length) < 2.0) {
      cost += 50.0;
    }
  }
  return cost;
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

std::vector<std::size_t> BuildCandidateProbeOrder(
    const std::vector<ParkingPoseCandidate>& candidates,
    const ParkingApproach preferred) {
  std::vector<std::size_t> ordered_indices;
  ordered_indices.reserve(candidates.size());
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (candidates[index].approach == preferred) {
      ordered_indices.push_back(index);
    }
  }
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (candidates[index].approach != preferred) {
      ordered_indices.push_back(index);
    }
  }
  return ordered_indices;
}

PlannerOpenSpaceConfig BuildProbeConfig(
    const OpenSpaceRoiDeciderConfig& config) {
  PlannerOpenSpaceConfig probe_config;
  if (config.has_candidate_warm_start_config()) {
    probe_config.mutable_warm_start_config()->CopyFrom(
        config.candidate_warm_start_config());
    return probe_config;
  }
  probe_config.mutable_warm_start_config()->set_step_size(
      config.candidate_path_step_size());
  probe_config.mutable_warm_start_config()->set_traj_forward_penalty(1.0);
  probe_config.mutable_warm_start_config()->set_traj_back_penalty(1.0);
  probe_config.mutable_warm_start_config()->set_traj_gear_switch_penalty(
      config.candidate_gear_switch_penalty());
  probe_config.mutable_warm_start_config()->set_traj_steer_penalty(1.0);
  probe_config.mutable_warm_start_config()->set_xy_grid_resolution(0.3);
  probe_config.mutable_warm_start_config()->set_phi_grid_resolution(0.1);
  probe_config.mutable_warm_start_config()->set_next_node_num(10);
  probe_config.mutable_warm_start_config()->set_grid_a_star_xy_resolution(0.5);
  probe_config.mutable_warm_start_config()->set_node_radius(0.25);
  probe_config.mutable_warm_start_config()->set_traj_kappa_contraint_ratio(0.7);
  probe_config.set_delta_t(0.5);
  auto* anchoring = probe_config.mutable_iterative_anchoring_smoother_config();
  anchoring->set_max_forward_v(2.0);
  anchoring->set_max_reverse_v(2.0);
  anchoring->set_max_forward_acc(3.0);
  anchoring->set_max_reverse_acc(2.0);
  anchoring->set_max_acc_jerk(4.0);
  anchoring->set_delta_t(0.2);
  auto* s_curve = anchoring->mutable_s_curve_config();
  s_curve->set_acc_weight(1.0);
  s_curve->set_jerk_weight(1.0);
  s_curve->set_kappa_penalty_weight(100.0);
  s_curve->set_ref_s_weight(10.0);
  return probe_config;
}

bool TryHybridAStarProbe(
    const ParkingPoseCandidate& variant, const ParkingRoiGeometry& roi_geometry,
    const Polygon2d& free_space_polygon,
    const apollo::common::VehicleParam& vehicle_param,
    const Vec2d& vehicle_position, const double vehicle_heading,
    const PlannerOpenSpaceConfig& probe_config,
    const OpenSpaceRoiDeciderConfig& config, const double goal_clearance,
    const double box_clearance, ParkingPoseCandidate* feasible_candidate,
    ParkingPoseCandidate* rejection_candidate, std::string* rejection_reason) {
  CHECK_NOTNULL(feasible_candidate);
  CHECK_NOTNULL(rejection_candidate);
  CHECK_NOTNULL(rejection_reason);
  HybridAStar hybrid_a_star(probe_config);
  HybridAStartResult hybrid_result;
  if (!hybrid_a_star.Plan(vehicle_position.x(), vehicle_position.y(),
                          vehicle_heading, variant.end_pose[0],
                          variant.end_pose[1], variant.end_pose[2],
                          roi_geometry.xy_boundary, roi_geometry.boundary_segments,
                          &hybrid_result) ||
      hybrid_result.x.empty() || hybrid_result.y.size() != hybrid_result.x.size() ||
      hybrid_result.phi.size() != hybrid_result.x.size()) {
    *rejection_reason = "hybrid-a-star probe failed";
    return false;
  }

  ParkingPoseCandidate candidate = variant;
  candidate.was_probed = true;
  candidate.feasible = true;
  candidate.path_length = hybrid_result.accumulated_s.empty()
                              ? 0.0
                              : hybrid_result.accumulated_s.back();
  candidate.reverse_distance = 0.0;
  candidate.gear_switch_count = 0;
  candidate.min_clearance = std::min(goal_clearance, box_clearance);

  const std::size_t check_start_index = hybrid_result.x.size() > 1U ? 1U : 0U;
  for (std::size_t index = check_start_index; index < hybrid_result.x.size();
       ++index) {
    if (hybrid_result.x[index] < roi_geometry.xy_boundary[0] ||
        hybrid_result.x[index] > roi_geometry.xy_boundary[1] ||
        hybrid_result.y[index] < roi_geometry.xy_boundary[2] ||
        hybrid_result.y[index] > roi_geometry.xy_boundary[3]) {
      *rejection_candidate = variant;
      rejection_candidate->was_probed = true;
      rejection_candidate->collision_path_index = static_cast<int>(index);
      rejection_candidate->collision_pose = {hybrid_result.x[index],
                                             hybrid_result.y[index],
                                             hybrid_result.phi[index]};
      *rejection_reason = "hybrid-a-star exits roi bounds";
      return false;
    }
    const Box2d ego_box =
        Node3d::GetBoundingBox(vehicle_param, hybrid_result.x[index],
                               hybrid_result.y[index], hybrid_result.phi[index]);
    int overlap_boundary_index = -1;
    Vec2d overlap_boundary_start;
    Vec2d overlap_boundary_end;
    if (BoxOverlapsBoundarySegments(ego_box, roi_geometry, &overlap_boundary_index,
                                    &overlap_boundary_start,
                                    &overlap_boundary_end)) {
      *rejection_candidate = variant;
      rejection_candidate->was_probed = true;
      rejection_candidate->collision_path_index = static_cast<int>(index);
      rejection_candidate->collision_boundary_index = overlap_boundary_index;
      rejection_candidate->collision_pose = {hybrid_result.x[index],
                                             hybrid_result.y[index],
                                             hybrid_result.phi[index]};
      rejection_candidate->collision_boundary_start = overlap_boundary_start;
      rejection_candidate->collision_boundary_end = overlap_boundary_end;
      *rejection_reason = "hybrid-a-star vehicle box overlaps roi boundary";
      return false;
    }
    candidate.min_clearance =
        std::min(candidate.min_clearance,
                 ComputeBoxBoundaryClearance(ego_box, free_space_polygon));
    if (index > 0U) {
      const double delta_length =
          Vec2d(hybrid_result.x[index], hybrid_result.y[index])
              .DistanceTo(Vec2d(hybrid_result.x[index - 1U],
                                hybrid_result.y[index - 1U]));
      const double current_velocity =
          hybrid_result.v.size() > index ? hybrid_result.v[index] : 0.0;
      const double previous_velocity =
          hybrid_result.v.size() > index - 1U ? hybrid_result.v[index - 1U] : 0.0;
      if (current_velocity < -common::math::kMathEpsilon) {
        candidate.reverse_distance += delta_length;
      }
      if (std::fabs(current_velocity) > common::math::kMathEpsilon &&
          std::fabs(previous_velocity) > common::math::kMathEpsilon &&
          current_velocity * previous_velocity < 0.0) {
        ++candidate.gear_switch_count;
      }
    }
  }
  if (candidate.path_length <= 0.0) {
    for (std::size_t index = 1U; index < hybrid_result.x.size(); ++index) {
      candidate.path_length +=
          Vec2d(hybrid_result.x[index], hybrid_result.y[index])
              .DistanceTo(Vec2d(hybrid_result.x[index - 1U],
                                hybrid_result.y[index - 1U]));
    }
  }
  candidate.score =
      candidate.path_length * config.candidate_path_length_penalty() +
      candidate.reverse_distance * config.candidate_reverse_distance_penalty() +
      candidate.gear_switch_count * config.candidate_gear_switch_penalty() -
      candidate.min_clearance * config.candidate_clearance_penalty();
  *feasible_candidate = std::move(candidate);
  return true;
}

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
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

std::vector<double> BuildSearchOffsets(const double step,
                                       const int max_steps_per_side) {
  std::vector<double> offsets{0.0};
  for (int step_index = 1; step_index <= max_steps_per_side; ++step_index) {
    const double offset = step * static_cast<double>(step_index);
    offsets.push_back(offset);
    offsets.push_back(-offset);
  }
  return offsets;
}

double ComputeBoxBoundaryClearance(const Box2d& ego_box,
                                   const Polygon2d& free_space_polygon) {
  std::vector<Vec2d> corners;
  ego_box.GetAllCorners(&corners);
  double min_clearance = std::numeric_limits<double>::infinity();
  for (const auto& corner : corners) {
    min_clearance = std::min(min_clearance,
                             free_space_polygon.DistanceToBoundary(corner));
  }
  return min_clearance;
}

bool BoxOverlapsBoundarySegments(const Box2d& ego_box,
                                 const ParkingRoiGeometry& roi_geometry,
                                 int* overlap_boundary_index = nullptr,
                                 Vec2d* overlap_boundary_start = nullptr,
                                 Vec2d* overlap_boundary_end = nullptr) {
  int boundary_index = 0;
  for (const auto& obstacle_vertices : roi_geometry.boundary_segments) {
    if (obstacle_vertices.size() < 2U) {
      ++boundary_index;
      continue;
    }
    for (std::size_t index = 1; index < obstacle_vertices.size(); ++index) {
      const common::math::LineSegment2d line_segment(
          obstacle_vertices[index - 1U], obstacle_vertices[index]);
      if (ego_box.HasOverlap(line_segment)) {
        if (overlap_boundary_index != nullptr) {
          *overlap_boundary_index = boundary_index;
        }
        if (overlap_boundary_start != nullptr) {
          *overlap_boundary_start = obstacle_vertices[index - 1U];
        }
        if (overlap_boundary_end != nullptr) {
          *overlap_boundary_end = obstacle_vertices[index];
        }
        return true;
      }
    }
    ++boundary_index;
  }
  return false;
}

std::vector<ParkingPoseCandidate> BuildCandidateVariantsFromWindow(
    const ParkingSlot& slot, const ParkingPoseCandidate& candidate,
    const VariantSearchWindow& window) {
  const Vec2d depth_axis = SlotDepthAxis(slot);
  const Vec2d width_axis = SlotWidthAxis(slot, depth_axis);
  const auto depth_offsets =
      BuildSearchOffsets(window.depth_step,
                         std::max(0, static_cast<int>(std::ceil(
                                         std::max(std::fabs(window.nominal_depth -
                                                            window.depth_lower_bound),
                                                  std::fabs(window.depth_upper_bound -
                                                            window.nominal_depth)) /
                                         window.depth_step))));
  const auto lateral_offsets =
      BuildSearchOffsets(window.lateral_step,
                         std::max(0, static_cast<int>(std::ceil(
                                         std::max(std::fabs(window.nominal_lateral -
                                                            window.lateral_lower_bound),
                                                  std::fabs(window.lateral_upper_bound -
                                                            window.nominal_lateral)) /
                                         window.lateral_step))));
  const auto heading_offsets =
      BuildSearchOffsets(window.heading_step, window.heading_steps_per_side);

  std::vector<std::tuple<double, double, double, double, ParkingPoseCandidate>>
      variants;
  variants.reserve(depth_offsets.size() * lateral_offsets.size() *
                   heading_offsets.size());
  for (const double depth_offset : depth_offsets) {
    const double depth = Clamp(window.nominal_depth + depth_offset,
                               window.depth_lower_bound,
                               window.depth_upper_bound);
    for (const double lateral_offset : lateral_offsets) {
      const double lateral = Clamp(window.nominal_lateral + lateral_offset,
                                   window.lateral_lower_bound,
                                   window.lateral_upper_bound);
      for (const double heading_offset : heading_offsets) {
        ParkingPoseCandidate variant = candidate;
        const Vec2d adjusted_center =
            slot.opening_center + depth_axis * depth + width_axis * lateral;
        variant.end_pose[0] = adjusted_center.x();
        variant.end_pose[1] = adjusted_center.y();
        variant.end_pose[2] = common::math::NormalizeAngle(
            window.nominal_heading + heading_offset);
        variants.emplace_back(std::fabs(depth_offset) + std::fabs(lateral_offset) +
                                  4.0 * std::fabs(heading_offset),
                              std::fabs(lateral_offset), std::fabs(depth_offset),
                              std::fabs(heading_offset), std::move(variant));
      }
    }
  }
  std::sort(variants.begin(), variants.end(),
            [](const auto& lhs, const auto& rhs) {
              return std::tie(std::get<0>(lhs), std::get<1>(lhs),
                              std::get<2>(lhs), std::get<3>(lhs)) <
                     std::tie(std::get<0>(rhs), std::get<1>(rhs),
                              std::get<2>(rhs), std::get<3>(rhs));
            });

  std::vector<ParkingPoseCandidate> ordered_variants;
  ordered_variants.reserve(variants.size());
  for (auto& entry : variants) {
    ordered_variants.push_back(std::move(std::get<4>(entry)));
  }
  return ordered_variants;
}

bool StaticVehicleBoxFits(const ParkingPoseCandidate& candidate,
                          const ParkingRoiGeometry& roi_geometry,
                          const apollo::common::VehicleParam& vehicle_param,
                          const Polygon2d& free_space_polygon,
                          const Polygon2d& parking_envelope_polygon,
                          double* goal_clearance, double* box_clearance,
                          std::string* rejection_reason) {
  CHECK_NOTNULL(goal_clearance);
  CHECK_NOTNULL(box_clearance);
  CHECK_NOTNULL(rejection_reason);

  const Box2d ego_box = Node3d::GetBoundingBox(
      vehicle_param, candidate.end_pose[0], candidate.end_pose[1],
      candidate.end_pose[2]);
  const Polygon2d ego_polygon(ego_box);
  if (!parking_envelope_polygon.Contains(ego_polygon)) {
    bool accepted_with_opening_fallback = false;
    if (!roi_geometry.slot_polygon.empty() &&
        free_space_polygon.Contains(ego_polygon)) {
      const double required_extension =
          ComputeParkingEnvelopeOpeningExtension(roi_geometry.slot_polygon,
                                                ego_box);
      if (required_extension > common::math::kMathEpsilon) {
        const auto expanded_envelope = BuildExpandedParkingEnvelopePolygon(
            roi_geometry, required_extension + 1e-3);
        if (expanded_envelope.size() >= 3U) {
          const Polygon2d expanded_polygon(expanded_envelope);
          if (expanded_polygon.Contains(ego_polygon)) {
            *box_clearance =
                ComputeBoxBoundaryClearance(ego_box, expanded_polygon);
            *goal_clearance = *box_clearance;
            accepted_with_opening_fallback = true;
          }
        }
      }
    }
    if (!accepted_with_opening_fallback) {
      *rejection_reason = "goal vehicle box exits parking envelope";
      return false;
    }
  } else {
    *box_clearance =
        ComputeBoxBoundaryClearance(ego_box, parking_envelope_polygon);
    *goal_clearance = *box_clearance;
  }
  return true;
}

std::vector<ParkingPoseCandidate> BuildCandidateVariants(
    const ParkingSlot& slot, const ParkingPoseCandidate& candidate,
    const apollo::common::VehicleParam& vehicle_param,
    const OpenSpaceRoiDeciderConfig& config) {
  const Vec2d depth_axis = SlotDepthAxis(slot);
  const Vec2d width_axis = SlotWidthAxis(slot, depth_axis);
  const Vec2d center(candidate.end_pose[0], candidate.end_pose[1]);
  const Vec2d delta = center - slot.opening_center;
  const double nominal_depth = delta.InnerProd(depth_axis);
  const double nominal_lateral = delta.InnerProd(width_axis);

  const double depth_step = std::max(0.15, 0.5 * config.candidate_path_step_size());
  const double lateral_step =
      std::max(0.1, 0.35 * config.candidate_path_step_size());
  const double heading_step =
      slot.type == ParkingSlotType::kAngled ? 0.04 : 0.03;

  const double lateral_slack =
      std::max(0.2, 0.5 * std::max(0.0, slot.width - vehicle_param.width()) +
                         0.1);
  VariantSearchWindow window;
  window.nominal_depth = nominal_depth;
  window.nominal_lateral = nominal_lateral;
  window.nominal_heading = candidate.end_pose[2];
  window.depth_lower_bound = -0.5;
  window.depth_upper_bound = slot.depth + 0.5;
  window.lateral_lower_bound = nominal_lateral - lateral_slack;
  window.lateral_upper_bound = nominal_lateral + lateral_slack;
  window.depth_step = depth_step;
  window.lateral_step = lateral_step;
  window.heading_step = heading_step;
  window.heading_steps_per_side =
      slot.type == ParkingSlotType::kAngled ? 3 : 2;
  return BuildCandidateVariantsFromWindow(slot, candidate, window);
}

std::vector<ParkingPoseCandidate> BuildRefinedCandidateVariants(
    const ParkingSlot& slot, const ParkingPoseCandidate& candidate,
    const apollo::common::VehicleParam& vehicle_param,
    const OpenSpaceRoiDeciderConfig& config) {
  const Vec2d depth_axis = SlotDepthAxis(slot);
  const Vec2d width_axis = SlotWidthAxis(slot, depth_axis);
  const Vec2d center(candidate.end_pose[0], candidate.end_pose[1]);
  const Vec2d delta = center - slot.opening_center;
  const double nominal_depth = delta.InnerProd(depth_axis);
  const double nominal_lateral = delta.InnerProd(width_axis);
  const double lateral_slack =
      std::max(0.2, 0.5 * std::max(0.0, slot.width - vehicle_param.width()) +
                         0.1);

  VariantSearchWindow window;
  window.nominal_depth = nominal_depth;
  window.nominal_lateral = nominal_lateral;
  window.nominal_heading = candidate.end_pose[2];
  window.depth_lower_bound = std::max(-0.5, nominal_depth - 0.2);
  window.depth_upper_bound = std::min(slot.depth + 0.5, nominal_depth + 0.2);
  window.lateral_lower_bound =
      std::max(nominal_lateral - lateral_slack, nominal_lateral - 0.2);
  window.lateral_upper_bound =
      std::min(nominal_lateral + lateral_slack, nominal_lateral + 0.2);
  window.depth_step = 0.02;
  window.lateral_step = 0.02;
  window.heading_step = slot.type == ParkingSlotType::kAngled ? 0.01 : 0.008;
  window.heading_steps_per_side =
      slot.type == ParkingSlotType::kAngled ? 4 : 3;
  return BuildCandidateVariantsFromWindow(slot, candidate, window);
}

std::vector<ParkingPoseCandidate> GenerateCandidates(
    const ParkingSlot& slot, const apollo::common::VehicleParam& vehicle_param,
    const OpenSpaceRoiDeciderConfig& config) {
  std::vector<ParkingPoseCandidate> candidates;
  const Vec2d opening_to_rear = slot.rear_center - slot.opening_center;
  const Vec2d depth_axis =
      opening_to_rear.Length() > common::math::kMathEpsilon
          ? opening_to_rear / opening_to_rear.Length()
          : Vec2d::CreateUnitVec2d(slot.heading);
  const Vec2d width_axis =
      (slot.corners.right_top - slot.corners.left_top).Length() >
              common::math::kMathEpsilon
          ? (slot.corners.right_top - slot.corners.left_top) /
                (slot.corners.right_top - slot.corners.left_top).Length()
          : Vec2d::CreateUnitVec2d(slot.heading);

  if (slot.type == ParkingSlotType::kParallel) {
    ParkingPoseCandidate tail_in;
    tail_in.approach = ParkingApproach::kTailIn;
    const double parallel_x =
        vehicle_param.back_edge_to_center() +
        config.parallel_park_end_x_buffer();
    const Vec2d end_reference_point =
        slot.corners.left_top + width_axis * parallel_x +
        depth_axis * (slot.depth * 0.5);
    tail_in.end_pose = {end_reference_point.x(), end_reference_point.y(),
                        width_axis.Angle(), 0.0};
    ParkingPoseCandidate head_in = tail_in;
    head_in.approach = ParkingApproach::kHeadIn;
    const Vec2d head_in_reference_point =
        slot.corners.left_top +
        width_axis * (slot.width - vehicle_param.front_edge_to_center()) +
        depth_axis * (slot.depth * 0.5);
    head_in.end_pose[0] = head_in_reference_point.x();
    head_in.end_pose[1] = head_in_reference_point.y();
    head_in.end_pose[2] =
        common::math::NormalizeAngle(width_axis.Angle() + M_PI);
    candidates.push_back(std::move(tail_in));
    candidates.push_back(std::move(head_in));
    return candidates;
  }

  const double parking_depth_buffer = config.parking_depth_buffer();
  const double parked_bumper_depth =
      Clamp(kParkedBumperDepthRatio * slot.depth, 0.6 * slot.depth,
            slot.depth - parking_depth_buffer);
  const double head_in_depth =
      Clamp(parked_bumper_depth - vehicle_param.front_edge_to_center(),
            -0.15 * slot.depth, 0.75 * slot.depth);
  const double tail_in_depth =
      Clamp(parked_bumper_depth - vehicle_param.back_edge_to_center(),
            0.2 * slot.depth, 0.75 * slot.depth);

  ParkingPoseCandidate head_in;
  head_in.approach = ParkingApproach::kHeadIn;
  const Vec2d head_in_reference_point =
      slot.opening_center + depth_axis * head_in_depth;
  head_in.end_pose = {head_in_reference_point.x(), head_in_reference_point.y(),
                      depth_axis.Angle(), 0.0};
  candidates.push_back(std::move(head_in));

  ParkingPoseCandidate tail_in;
  tail_in.approach = ParkingApproach::kTailIn;
  const Vec2d tail_in_reference_point =
      slot.opening_center + depth_axis * tail_in_depth;
  tail_in.end_pose = {tail_in_reference_point.x(), tail_in_reference_point.y(),
                      common::math::NormalizeAngle(depth_axis.Angle() + M_PI),
                      0.0};
  candidates.push_back(std::move(tail_in));
  return candidates;
}

bool ProbeCandidate(const ParkingPoseCandidate& candidate,
                    const ParkingSlot& slot,
                    const ParkingRoiGeometry& roi_geometry,
                    const apollo::common::VehicleParam& vehicle_param,
                    const Vec2d& vehicle_position, double vehicle_heading,
                    const OpenSpaceRoiDeciderConfig& config,
                    ParkingPoseCandidate* probed_candidate) {
  CHECK_NOTNULL(probed_candidate);
  *probed_candidate = candidate;
  probed_candidate->was_probed = true;
  probed_candidate->aisle_width = roi_geometry.aisle_width;

  if (candidate.approach == ParkingApproach::kHeadIn &&
      roi_geometry.aisle_width < config.candidate_min_aisle_width_head_in()) {
    probed_candidate->rejection_reason =
        "aisle width below head-in threshold";
    return false;
  }
  if (candidate.approach == ParkingApproach::kTailIn &&
      roi_geometry.aisle_width < config.candidate_min_aisle_width_tail_in()) {
    probed_candidate->rejection_reason =
        "aisle width below tail-in threshold";
    return false;
  }

  if (roi_geometry.union_polygon.size() < 3U) {
    probed_candidate->rejection_reason = "roi polygon unavailable";
    return false;
  }
  Polygon2d free_space_polygon(roi_geometry.union_polygon);
  const auto parking_envelope =
      BuildParkingEnvelopePolygon(roi_geometry);
  if (parking_envelope.size() < 3U) {
    probed_candidate->rejection_reason = "parking envelope unavailable";
    return false;
  }
  Polygon2d parking_envelope_polygon(parking_envelope);
  PlannerOpenSpaceConfig probe_config = BuildProbeConfig(config);
  auto start_node = std::make_shared<Node3d>(
      vehicle_position.x(), vehicle_position.y(), vehicle_heading,
      roi_geometry.xy_boundary, probe_config);
  ProbeReedShepp reed_shepp(vehicle_param, probe_config);

  ParkingPoseCandidate best_candidate = *probed_candidate;
  bool found_feasible_variant = false;
  bool saw_static_box_rejection = false;
  bool saw_path_rejection = false;
  std::string path_rejection_reason;
  std::string last_rejection_reason = "reeds-shepp probe failed";
  ParkingPoseCandidate best_path_rejection_candidate = *probed_candidate;
  bool has_best_path_rejection_candidate = false;
  auto probe_variants = [&](const std::vector<ParkingPoseCandidate>& variants) {
    for (const auto& variant : variants) {
      double goal_clearance = 0.0;
      double box_clearance = 0.0;
      std::string static_rejection_reason;
      if (!StaticVehicleBoxFits(variant, roi_geometry, vehicle_param,
                                free_space_polygon, parking_envelope_polygon,
                                &goal_clearance, &box_clearance,
                                &static_rejection_reason)) {
        if (static_rejection_reason == "goal vehicle box exits parking envelope") {
          saw_static_box_rejection = true;
        }
        last_rejection_reason = static_rejection_reason;
        continue;
      }

      auto end_node = std::make_shared<Node3d>(
          variant.end_pose[0], variant.end_pose[1], variant.end_pose[2],
          roi_geometry.xy_boundary, probe_config);
      std::vector<ReedSheppPath> paths;
      if (!reed_shepp.GenerateRSPs(start_node, end_node, &paths) ||
          paths.empty()) {
        last_rejection_reason = "reeds-shepp probe failed";
        continue;
      }
      for (auto& path : paths) {
        if (path.segs_lengths.empty() || path.segs_types.empty() ||
            path.total_length <= 0.0) {
          path.cost = std::numeric_limits<double>::infinity();
          continue;
        }
        path.cost = ComputeReedSheppPathCost(
            start_node, path,
            probe_config.warm_start_config().traj_forward_penalty(),
            probe_config.warm_start_config().traj_back_penalty(),
            probe_config.warm_start_config().traj_gear_switch_penalty(),
            probe_config.warm_start_config().traj_steer_penalty());
      }
      std::sort(paths.begin(), paths.end(),
                [](const ReedSheppPath& lhs, const ReedSheppPath& rhs) {
                  return std::tie(lhs.cost, lhs.total_length) <
                         std::tie(rhs.cost, rhs.total_length);
                });

      bool variant_path_feasible = false;
      for (auto path : paths) {
        if (!std::isfinite(path.cost) ||
            !reed_shepp.GenerateLocalConfigurations(start_node, end_node,
                                                    &path)) {
          last_rejection_reason =
              "reeds-shepp local configuration generation failed";
          continue;
        }
        double reverse_distance = 0.0;
        int gear_switches = 0;
        double min_clearance = std::min(goal_clearance, box_clearance);
        bool path_feasible = true;
        ParkingPoseCandidate path_rejection_candidate = variant;
        path_rejection_candidate.was_probed = true;
        const std::size_t check_start_index = path.x.size() > 1U ? 1U : 0U;
        for (std::size_t index = check_start_index; index < path.x.size();
             ++index) {
          if (path.x[index] < roi_geometry.xy_boundary[0] ||
              path.x[index] > roi_geometry.xy_boundary[1] ||
              path.y[index] < roi_geometry.xy_boundary[2] ||
              path.y[index] > roi_geometry.xy_boundary[3]) {
            last_rejection_reason = "probe exits roi bounds";
            path_rejection_candidate.collision_path_index =
                static_cast<int>(index);
            path_rejection_candidate.collision_pose = {
                path.x[index], path.y[index], path.phi[index]};
            path_feasible = false;
            break;
          }
          const Box2d ego_box = Node3d::GetBoundingBox(
              vehicle_param, path.x[index], path.y[index], path.phi[index]);
          int overlap_boundary_index = -1;
          Vec2d overlap_boundary_start;
          Vec2d overlap_boundary_end;
          if (BoxOverlapsBoundarySegments(ego_box, roi_geometry,
                                          &overlap_boundary_index,
                                          &overlap_boundary_start,
                                          &overlap_boundary_end)) {
            last_rejection_reason = "probe vehicle box overlaps roi boundary";
            path_rejection_candidate.collision_path_index =
                static_cast<int>(index);
            path_rejection_candidate.collision_boundary_index =
                overlap_boundary_index;
            path_rejection_candidate.collision_pose = {
                path.x[index], path.y[index], path.phi[index]};
            path_rejection_candidate.collision_boundary_start =
                overlap_boundary_start;
            path_rejection_candidate.collision_boundary_end =
                overlap_boundary_end;
            path_feasible = false;
            break;
          }
          min_clearance =
              std::min(min_clearance,
                       ComputeBoxBoundaryClearance(ego_box, free_space_polygon));
          if (index > 0U) {
            const double delta_length =
                Vec2d(path.x[index], path.y[index])
                    .DistanceTo(Vec2d(path.x[index - 1U], path.y[index - 1U]));
            if (path.gear.size() > index && !path.gear[index]) {
              reverse_distance += delta_length;
            }
            if (path.gear.size() > index &&
                path.gear[index] != path.gear[index - 1U]) {
              ++gear_switches;
            }
          }
        }
        if (!path_feasible) {
          if (!has_best_path_rejection_candidate ||
              path_rejection_candidate.collision_path_index >= 0) {
            best_path_rejection_candidate = std::move(path_rejection_candidate);
            has_best_path_rejection_candidate = true;
          }
          continue;
        }

        variant_path_feasible = true;
        ParkingPoseCandidate feasible_candidate = variant;
        feasible_candidate.was_probed = true;
        feasible_candidate.feasible = true;
        feasible_candidate.path_length = path.total_length;
        feasible_candidate.reverse_distance = reverse_distance;
        feasible_candidate.gear_switch_count = gear_switches;
        feasible_candidate.min_clearance = min_clearance;
        feasible_candidate.score =
            path.total_length * config.candidate_path_length_penalty() +
            reverse_distance * config.candidate_reverse_distance_penalty() +
            gear_switches * config.candidate_gear_switch_penalty() -
            min_clearance * config.candidate_clearance_penalty();
        if (!found_feasible_variant ||
            feasible_candidate.score < best_candidate.score) {
          best_candidate = std::move(feasible_candidate);
          found_feasible_variant = true;
        }
      }
      if (!variant_path_feasible) {
        ParkingPoseCandidate hybrid_candidate;
        ParkingPoseCandidate hybrid_rejection_candidate = variant;
        std::string hybrid_rejection_reason;
        if (TryHybridAStarProbe(
                variant, roi_geometry, free_space_polygon, vehicle_param,
                vehicle_position, vehicle_heading, probe_config, config,
                goal_clearance, box_clearance, &hybrid_candidate,
                &hybrid_rejection_candidate, &hybrid_rejection_reason)) {
          variant_path_feasible = true;
          if (!found_feasible_variant ||
              hybrid_candidate.score < best_candidate.score) {
            best_candidate = std::move(hybrid_candidate);
            found_feasible_variant = true;
          }
          break;
        }
        saw_path_rejection = true;
        const bool use_hybrid_rejection =
            !hybrid_rejection_reason.empty() &&
            hybrid_rejection_reason != "hybrid-a-star probe failed";
        path_rejection_reason =
            use_hybrid_rejection ? hybrid_rejection_reason : last_rejection_reason;
        if (use_hybrid_rejection &&
            (!has_best_path_rejection_candidate ||
             hybrid_rejection_candidate.collision_path_index >= 0)) {
          best_path_rejection_candidate = std::move(hybrid_rejection_candidate);
          has_best_path_rejection_candidate = true;
        }
      }
      continue;
    }
  };
  probe_variants(BuildCandidateVariants(slot, candidate, vehicle_param, config));
  if (!found_feasible_variant &&
      (saw_static_box_rejection || saw_path_rejection)) {
    probe_variants(
        BuildRefinedCandidateVariants(slot, candidate, vehicle_param, config));
  }

  if (!found_feasible_variant) {
    if (saw_path_rejection) {
      last_rejection_reason = path_rejection_reason;
      if (has_best_path_rejection_candidate) {
        probed_candidate->collision_path_index =
            best_path_rejection_candidate.collision_path_index;
        probed_candidate->collision_boundary_index =
            best_path_rejection_candidate.collision_boundary_index;
        probed_candidate->collision_pose =
            best_path_rejection_candidate.collision_pose;
        probed_candidate->collision_boundary_start =
         best_path_rejection_candidate.collision_boundary_start;
        probed_candidate->collision_boundary_end =
            best_path_rejection_candidate.collision_boundary_end;
      }
    } else if (saw_static_box_rejection &&
               last_rejection_reason == "goal vehicle box exits parking envelope") {
      last_rejection_reason =
          "no statically feasible vehicle box inside parking envelope";
    }
    probed_candidate->rejection_reason = last_rejection_reason;
    return false;
  }
  *probed_candidate = std::move(best_candidate);
  return true;
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

const char* ParkingApproachName(ParkingApproach approach) {
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

ParkingPoseSelector::ParkingPoseSelector(const OpenSpaceRoiDeciderConfig& config)
    : config_(config) {}

ParkingPoseSelection ParkingPoseSelector::Select(
    const ParkingSlot& normalized_slot, const ParkingRoiGeometry& roi_geometry,
    const apollo::common::VehicleParam& vehicle_param,
    const Vec2d& vehicle_position, const double vehicle_heading) const {
  ParkingPoseSelection selection;
  selection.candidates =
      GenerateCandidates(normalized_slot, vehicle_param, config_);
  const ParkingApproach preferred =
      ResolveEffectiveParkingApproachPreference(config_, normalized_slot);
  const auto probe_order =
      BuildCandidateProbeOrder(selection.candidates, preferred);

  for (const std::size_t candidate_index : probe_order) {
    auto& candidate = selection.candidates[candidate_index];
    ParkingPoseCandidate probed_candidate;
    ProbeCandidate(candidate, normalized_slot, roi_geometry, vehicle_param,
                   vehicle_position,
                   vehicle_heading, config_, &probed_candidate);
    candidate = std::move(probed_candidate);
    if (!candidate.feasible) {
      continue;
    }
    if (!selection.has_feasible_candidate()) {
      selection.selected_index = static_cast<int>(candidate_index);
    }
  }
  return selection;
}

}  // namespace parking
}  // namespace planning
}  // namespace apollo
