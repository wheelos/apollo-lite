/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/planning/tasks/deciders/open_space_decider/open_space_roi_decider.h"

#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <queue>
#include <utility>

#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/open_space/parking/parking_pose_selector.h"
#include "modules/planning/open_space/parking/parking_roi_geometry.h"
#include "modules/planning/open_space/parking/parking_roi_validator.h"
#include "modules/planning/open_space/parking/parking_slot_provider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneInfoConstPtr;
using apollo::hdmap::LaneSegment;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;

namespace {

struct ParkingOrigin {
  Vec2d point;
  double heading = 0.0;
};

apollo::common::TrajectoryPoint MakeDebugTrajectoryPoint(const double x,
                                                         const double y,
                                                         const double theta) {
  apollo::common::TrajectoryPoint trajectory_point;
  auto *path_point = trajectory_point.mutable_path_point();
  path_point->set_x(x);
  path_point->set_y(y);
  path_point->set_theta(theta);
  return trajectory_point;
}

void SetDebugPoint(const Vec2d &point, apollo::common::PointENU *point_enu) {
  point_enu->set_x(point.x());
  point_enu->set_y(point.y());
  point_enu->set_z(0.0);
}

void PopulateParkingDebug(
    planning_internal::OpenSpaceDebug *open_space_debug,
    const parking::ParkingRoiGeometry &roi_geometry,
    const parking::ParkingPoseSelection &pose_selection,
    const parking::ParkingRoiValidationResult *validation_result) {
  open_space_debug->clear_parking_pose_candidates();
  auto *parking_roi_debug = open_space_debug->mutable_parking_roi();
  parking_roi_debug->clear_corridor_polygon();
  parking_roi_debug->clear_slot_polygon();
  parking_roi_debug->clear_union_polygon();
  parking_roi_debug->clear_bridge_polygon();
  parking_roi_debug->clear_attachment_polygon();
  parking_roi_debug->clear_connector_slice();
  parking_roi_debug->clear_outer_bridge_slice();
  for (const auto &point : roi_geometry.corridor_polygon) {
    SetDebugPoint(point, parking_roi_debug->add_corridor_polygon());
  }
  for (const auto &point : roi_geometry.bridge_polygon) {
    SetDebugPoint(point, parking_roi_debug->add_bridge_polygon());
  }
  for (const auto &point : roi_geometry.attachment_polygon) {
    SetDebugPoint(point, parking_roi_debug->add_attachment_polygon());
  }
  for (const auto &point : roi_geometry.connector_slice) {
    SetDebugPoint(point, parking_roi_debug->add_connector_slice());
  }
  for (const auto &point : roi_geometry.outer_bridge_slice) {
    SetDebugPoint(point, parking_roi_debug->add_outer_bridge_slice());
  }
  for (const auto &point : roi_geometry.slot_polygon) {
    SetDebugPoint(point, parking_roi_debug->add_slot_polygon());
  }
  for (const auto &point : roi_geometry.union_polygon) {
    SetDebugPoint(point, parking_roi_debug->add_union_polygon());
  }
  parking_roi_debug->set_area(roi_geometry.area);
  parking_roi_debug->set_min_aisle_width(roi_geometry.aisle_width);
  parking_roi_debug->set_connection_start_index(
      static_cast<uint32_t>(roi_geometry.connection_start_index));
  parking_roi_debug->set_connection_end_index(
      static_cast<uint32_t>(roi_geometry.connection_end_index));
  parking_roi_debug->set_slot_side_boundary_size(
      static_cast<uint32_t>(roi_geometry.slot_side_boundary_size));
  parking_roi_debug->set_connector_boundary_size(
      static_cast<uint32_t>(roi_geometry.connector_boundary_size));
  parking_roi_debug->set_is_valid(false);
  parking_roi_debug->clear_invalid_reason();
  parking_roi_debug->set_goal_clearance(0.0);
  parking_roi_debug->set_relaxed_mode(false);
  parking_roi_debug->set_goal_inside_roi(false);
  parking_roi_debug->set_goal_inside_envelope(false);
  if (validation_result != nullptr) {
    parking_roi_debug->set_is_valid(validation_result->valid);
    parking_roi_debug->set_invalid_reason(validation_result->reason);
    parking_roi_debug->set_goal_clearance(validation_result->goal_clearance);
    parking_roi_debug->set_goal_inside_roi(validation_result->goal_inside_roi);
    parking_roi_debug->set_goal_inside_envelope(
        validation_result->goal_inside_envelope);
  }

  if (pose_selection.has_feasible_candidate()) {
    open_space_debug->set_selected_parking_pose(
        parking::ParkingApproachName(pose_selection.selected().approach));
    open_space_debug->mutable_end_point()->CopyFrom(
        MakeDebugTrajectoryPoint(pose_selection.selected().end_pose[0],
                                 pose_selection.selected().end_pose[1],
                                 pose_selection.selected().end_pose[2]));
  } else {
    open_space_debug->clear_selected_parking_pose();
    open_space_debug->clear_end_point();
  }

  for (const auto &candidate : pose_selection.candidates) {
    if (!candidate.was_probed) {
      continue;
    }
    auto *candidate_debug = open_space_debug->add_parking_pose_candidates();
    candidate_debug->set_name(parking::ParkingApproachName(candidate.approach));
    candidate_debug->set_was_probed(candidate.was_probed);
    candidate_debug->set_feasible(candidate.feasible);
    candidate_debug->set_score(candidate.score);
    candidate_debug->set_rejection_reason(candidate.rejection_reason);
    candidate_debug->set_reverse_distance(candidate.reverse_distance);
    candidate_debug->set_gear_switch_count(
        static_cast<uint32_t>(candidate.gear_switch_count));
    candidate_debug->set_aisle_width(candidate.aisle_width);
    candidate_debug->set_path_length(candidate.path_length);
    candidate_debug->set_min_clearance(candidate.min_clearance);
    if (candidate.end_pose.size() >= 3U) {
      candidate_debug->mutable_end_point()->CopyFrom(MakeDebugTrajectoryPoint(
          candidate.end_pose[0], candidate.end_pose[1], candidate.end_pose[2]));
    }
    if (candidate.collision_path_index >= 0) {
      candidate_debug->set_collision_path_index(
          static_cast<uint32_t>(candidate.collision_path_index));
    }
    if (candidate.collision_boundary_index >= 0) {
      candidate_debug->set_collision_boundary_index(
          static_cast<uint32_t>(candidate.collision_boundary_index));
    }
    if (candidate.collision_pose.size() >= 3U) {
      candidate_debug->mutable_collision_point()->CopyFrom(
          MakeDebugTrajectoryPoint(candidate.collision_pose[0],
                                   candidate.collision_pose[1],
                                   candidate.collision_pose[2]));
    }
    if (candidate.collision_boundary_index >= 0) {
      SetDebugPoint(candidate.collision_boundary_start,
                    candidate_debug->mutable_collision_boundary_start());
      SetDebugPoint(candidate.collision_boundary_end,
                    candidate_debug->mutable_collision_boundary_end());
    }
  }
}

void ApplyParkingValidationDebug(
    planning_internal::OpenSpaceDebug *open_space_debug,
    const parking::ParkingRoiValidationResult &validation_result) {
  auto *parking_roi_debug = open_space_debug->mutable_parking_roi();
  parking_roi_debug->set_area(validation_result.area);
  parking_roi_debug->set_is_valid(validation_result.valid);
  parking_roi_debug->set_invalid_reason(validation_result.reason);
  parking_roi_debug->set_goal_clearance(validation_result.goal_clearance);
  parking_roi_debug->set_relaxed_mode(false);
  parking_roi_debug->set_goal_inside_roi(validation_result.goal_inside_roi);
  parking_roi_debug->set_goal_inside_envelope(
      validation_result.goal_inside_envelope);
}

void TransformBoundary(const Vec2d &origin_point, const double origin_heading,
                       std::vector<Vec2d> *boundary) {
  for (auto &point : *boundary) {
    point -= origin_point;
    point.SelfRotate(-origin_heading);
  }
}

void AppendBoundarySample(const double sampled_s, const Vec2d &left_point,
                          const Vec2d &right_point, const double left_width,
                          const double right_width,
                          std::vector<Vec2d> *left_lane_boundary,
                          std::vector<Vec2d> *right_lane_boundary,
                          std::vector<double> *center_lane_s_left,
                          std::vector<double> *center_lane_s_right,
                          std::vector<double> *left_lane_road_width,
                          std::vector<double> *right_lane_road_width) {
  CHECK_NOTNULL(left_lane_boundary);
  CHECK_NOTNULL(right_lane_boundary);
  CHECK_NOTNULL(center_lane_s_left);
  CHECK_NOTNULL(center_lane_s_right);
  CHECK_NOTNULL(left_lane_road_width);
  CHECK_NOTNULL(right_lane_road_width);

  const bool duplicate_sample =
      !center_lane_s_left->empty() &&
      std::fabs(sampled_s - center_lane_s_left->back()) <= 1e-6;
  const bool duplicate_points =
      !left_lane_boundary->empty() &&
      left_lane_boundary->back().DistanceTo(left_point) <= 1e-6 &&
      right_lane_boundary->back().DistanceTo(right_point) <= 1e-6;
  if (duplicate_sample || duplicate_points) {
    return;
  }

  left_lane_boundary->push_back(left_point);
  right_lane_boundary->push_back(right_point);
  center_lane_s_left->push_back(sampled_s);
  center_lane_s_right->push_back(sampled_s);
  left_lane_road_width->push_back(left_width);
  right_lane_road_width->push_back(right_width);
}

void SampleParkingRoadBoundariesDense(
    const hdmap::Path &nearby_path, const double sampling_start_s,
    const double sampling_end_s, const Vec2d &origin_point,
    const double origin_heading, std::vector<Vec2d> *left_lane_boundary,
    std::vector<Vec2d> *right_lane_boundary,
    std::vector<double> *center_lane_s_left,
    std::vector<double> *center_lane_s_right,
    std::vector<double> *left_lane_road_width,
    std::vector<double> *right_lane_road_width) {
  CHECK_NOTNULL(left_lane_boundary);
  CHECK_NOTNULL(right_lane_boundary);
  CHECK_NOTNULL(center_lane_s_left);
  CHECK_NOTNULL(center_lane_s_right);
  CHECK_NOTNULL(left_lane_road_width);
  CHECK_NOTNULL(right_lane_road_width);
  left_lane_boundary->clear();
  right_lane_boundary->clear();
  center_lane_s_left->clear();
  center_lane_s_right->clear();
  left_lane_road_width->clear();
  right_lane_road_width->clear();

  const double path_start_s = 0.0;
  const double path_end_s = std::max(path_start_s, nearby_path.length());
  const double start_s =
      std::max(path_start_s, std::min(path_end_s, sampling_start_s));
  const double end_s = std::max(start_s, std::min(path_end_s, sampling_end_s));
  const double step = 0.2;

  for (double sample_s = start_s;
       sample_s <= end_s + common::math::kMathEpsilon; sample_s += step) {
    const double clamped_s = std::max(start_s, std::min(sample_s, end_s));
    const auto path_point = nearby_path.GetSmoothPoint(clamped_s);
    const double heading = path_point.heading();
    const double left_width = nearby_path.GetRoadLeftWidth(clamped_s);
    const double right_width = nearby_path.GetRoadRightWidth(clamped_s);
    Vec2d left_point(left_width * std::cos(heading + M_PI_2),
                     left_width * std::sin(heading + M_PI_2));
    left_point = left_point + path_point;
    Vec2d right_point(right_width * std::cos(heading - M_PI_2),
                      right_width * std::sin(heading - M_PI_2));
    right_point = right_point + path_point;
    AppendBoundarySample(clamped_s, left_point, right_point, left_width,
                         right_width, left_lane_boundary, right_lane_boundary,
                         center_lane_s_left, center_lane_s_right,
                         left_lane_road_width, right_lane_road_width);
    if (clamped_s >= end_s) {
      break;
    }
  }

  if (center_lane_s_left->empty() ||
      std::fabs(center_lane_s_left->back() - end_s) >
          common::math::kMathEpsilon) {
    const auto path_point = nearby_path.GetSmoothPoint(end_s);
    const double heading = path_point.heading();
    const double left_width = nearby_path.GetRoadLeftWidth(end_s);
    const double right_width = nearby_path.GetRoadRightWidth(end_s);
    Vec2d left_point(left_width * std::cos(heading + M_PI_2),
                     left_width * std::sin(heading + M_PI_2));
    left_point = left_point + path_point;
    Vec2d right_point(right_width * std::cos(heading - M_PI_2),
                      right_width * std::sin(heading - M_PI_2));
    right_point = right_point + path_point;
    AppendBoundarySample(end_s, left_point, right_point, left_width,
                         right_width, left_lane_boundary, right_lane_boundary,
                         center_lane_s_left, center_lane_s_right,
                         left_lane_road_width, right_lane_road_width);
  }

  TransformBoundary(origin_point, origin_heading, left_lane_boundary);
  TransformBoundary(origin_point, origin_heading, right_lane_boundary);
}

double ComputeParkingRoiEgoSamplingMargin(
    const OpenSpaceRoiDeciderConfig &config,
    const apollo::common::VehicleParam &vehicle_param) {
  double probe_step = config.candidate_path_step_size();
  if (config.has_candidate_warm_start_config()) {
    probe_step =
        std::max(probe_step, config.candidate_warm_start_config().step_size());
  }
  return 0.5 * vehicle_param.length() + std::max(probe_step, 0.5);
}

void ComputeParkingRoiSamplingRange(
    const OpenSpaceRoiDeciderConfig &config, const hdmap::Path &nearby_path,
    const double center_line_s, const double ego_s,
    const apollo::common::VehicleParam &vehicle_param, double *sampling_start_s,
    double *sampling_end_s) {
  CHECK_NOTNULL(sampling_start_s);
  CHECK_NOTNULL(sampling_end_s);
  const double path_start_s = 0.0;
  const double path_end_s = std::max(path_start_s, nearby_path.length());
  const double slot_start_s =
      center_line_s - config.roi_longitudinal_range_start();
  const double slot_end_s = center_line_s + config.roi_longitudinal_range_end();
  const double ego_margin =
      ComputeParkingRoiEgoSamplingMargin(config, vehicle_param);
  *sampling_start_s =
      std::max(path_start_s, std::min(slot_start_s, ego_s - ego_margin));
  *sampling_end_s =
      std::min(path_end_s, std::max(slot_end_s, ego_s + ego_margin));
  if (*sampling_end_s < *sampling_start_s) {
    *sampling_end_s = *sampling_start_s;
  }
}

double InterpolateOpeningL(const double query_s, const double start_s,
                           const double start_l, const double end_s,
                           const double end_l) {
  if (std::fabs(end_s - start_s) < common::math::kMathEpsilon) {
    return 0.5 * (start_l + end_l);
  }
  const double ratio =
      std::max(0.0, std::min(1.0, (query_s - start_s) / (end_s - start_s)));
  return start_l + ratio * (end_l - start_l);
}

std::vector<Vec2d> BuildSlotSideConnectorBoundary(
    const hdmap::Path &nearby_path,
    const std::vector<Vec2d> &slot_side_outer_boundary,
    const std::vector<double> &boundary_s,
    const std::size_t connection_start_index,
    const std::size_t connection_end_index, const double opening_start_s,
    const double opening_start_l, const double opening_end_s,
    const double opening_end_l, const Vec2d &origin_point,
    const double origin_heading) {
  std::vector<Vec2d> connector_boundary;
  connector_boundary.reserve(slot_side_outer_boundary.size());
  if (slot_side_outer_boundary.size() != boundary_s.size()) {
    return connector_boundary;
  }
  double previous_sample_s = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t index = 0; index < boundary_s.size(); ++index) {
    const double sample_s = boundary_s[index];
    if (std::isfinite(previous_sample_s) &&
        std::fabs(sample_s - previous_sample_s) <= 1e-6) {
      continue;
    }
    Vec2d connector_point;
    if (index < connection_start_index || index > connection_end_index) {
      connector_point = slot_side_outer_boundary[index];
    } else {
      const auto path_point = nearby_path.GetSmoothPoint(sample_s);
      const double connector_l =
          InterpolateOpeningL(sample_s, opening_start_s, opening_start_l,
                              opening_end_s, opening_end_l);
      connector_point = Vec2d(path_point.x(), path_point.y());
      connector_point +=
          Vec2d(connector_l * std::cos(path_point.heading() + M_PI_2),
                connector_l * std::sin(path_point.heading() + M_PI_2));
      connector_point -= origin_point;
      connector_point.SelfRotate(-origin_heading);
    }
    if (connector_boundary.empty() ||
        connector_boundary.back().DistanceTo(connector_point) > 1e-6) {
      connector_boundary.push_back(connector_point);
    }
    previous_sample_s = sample_s;
  }
  return connector_boundary;
}

ParkingOrigin DetermineParkingOrigin(const parking::ParkingSlot &slot) {
  ParkingOrigin origin;
  origin.point = slot.opening_center;
  origin.heading = common::math::NormalizeAngle(slot.lane_heading);
  return origin;
}

void ApplyParkingOrigin(Frame *frame, const ParkingOrigin &origin) {
  frame->mutable_open_space_info()->set_origin_heading(origin.heading);
  frame->mutable_open_space_info()->mutable_origin_point()->set_x(
      origin.point.x());
  frame->mutable_open_space_info()->mutable_origin_point()->set_y(
      origin.point.y());
}

void ApplyParkingEndPose(Frame *frame,
                         const parking::ParkingPoseCandidate &candidate) {
  auto *end_pose =
      frame->mutable_open_space_info()->mutable_open_space_end_pose();
  end_pose->clear();
  end_pose->insert(end_pose->end(), candidate.end_pose.begin(),
                   candidate.end_pose.end());
  frame->mutable_open_space_info()->set_parking_head_in(
      candidate.approach == parking::ParkingApproach::kHeadIn);
}

bool TryReuseParkingEndPoseFromPreviousFrame(
    const std::shared_ptr<DependencyInjector> &injector,
    const std::string &parking_spot_id,
    const parking::ParkingRoiGeometry &roi_geometry, const Vec2d &vehicle_xy,
    const apollo::common::VehicleParam &vehicle_params,
    const OpenSpaceRoiDeciderConfig &roi_config,
    const parking::ParkingRoiValidator &roi_validator, Frame *frame,
    parking::ParkingPoseSelection *pose_selection) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(pose_selection);
  const auto &ptr_last_frame = injector->frame_history()->Latest();
  if (ptr_last_frame == nullptr) {
    return false;
  }
  const auto &previous_open_space_info = ptr_last_frame->open_space_info();
  if (previous_open_space_info.target_parking_spot_id() != parking_spot_id ||
      previous_open_space_info.open_space_end_pose().size() < 4U ||
      previous_open_space_info.ROI_xy_boundary().size() < 4U) {
    return false;
  }
  const auto &current_open_space_info = frame->open_space_info();
  if (current_open_space_info.origin_point().DistanceTo(
          previous_open_space_info.origin_point()) > 1e-3 ||
      std::abs(common::math::AngleDiff(
          current_open_space_info.origin_heading(),
          previous_open_space_info.origin_heading())) > 1e-3) {
    return false;
  }

  parking::ParkingPoseCandidate cached_candidate;
  cached_candidate.approach = previous_open_space_info.parking_head_in()
                                   ? parking::ParkingApproach::kHeadIn
                                   : parking::ParkingApproach::kTailIn;
  const auto configured_approach =
      parking::ResolveParkingApproachPreference(roi_config);
  if (configured_approach != parking::ParkingApproach::kUnknown &&
      cached_candidate.approach != configured_approach) {
    AINFO_EVERY(20) << "Reject cached parking end pose for slot "
                    << parking_spot_id
                    << " because approach preference changed from "
                    << parking::ParkingApproachName(cached_candidate.approach)
                    << " to "
                    << parking::ParkingApproachName(configured_approach);
    return false;
  }
  cached_candidate.end_pose = previous_open_space_info.open_space_end_pose();
  const auto validation_result = roi_validator.Validate(
      roi_geometry, vehicle_xy, cached_candidate.end_pose, vehicle_params);
  if (!validation_result.valid) {
    AINFO_EVERY(20) << "Reject cached parking end pose for slot "
                    << parking_spot_id
                    << " because current ROI validation failed: "
                    << validation_result.reason;
    return false;
  }

  auto *end_pose =
      frame->mutable_open_space_info()->mutable_open_space_end_pose();
  *end_pose = previous_open_space_info.open_space_end_pose();
  frame->mutable_open_space_info()->set_parking_head_in(
      previous_open_space_info.parking_head_in());
  frame->mutable_open_space_info()->set_parking_enforce_final_gear(
      previous_open_space_info.parking_enforce_final_gear());

  cached_candidate.was_probed = true;
  cached_candidate.feasible = true;
  cached_candidate.rejection_reason =
      "reused from previous frame after current ROI validation";
  pose_selection->candidates.clear();
  pose_selection->candidates.push_back(std::move(cached_candidate));
  pose_selection->selected_index = 0;
  AINFO_EVERY(20) << "Reuse parking end pose from previous frame for slot "
                  << parking_spot_id;
  return true;
}

void UpdateParkingDebug(
    Frame *frame, const parking::ParkingRoiGeometry &roi_geometry,
    const parking::ParkingPoseSelection &pose_selection,
    const parking::ParkingRoiValidationResult *validation_result) {
  auto *parking_debug =
      frame->mutable_open_space_info()->mutable_parking_debug();
  PopulateParkingDebug(parking_debug, roi_geometry, pose_selection,
                       validation_result);

  auto *debug_instance = frame->mutable_open_space_info()
                             ->mutable_debug_instance()
                             ->mutable_planning_data()
                             ->mutable_open_space();
  PopulateParkingDebug(debug_instance, roi_geometry, pose_selection,
                       validation_result);

  auto *debug = frame->mutable_open_space_info()->mutable_debug();
  if (debug != nullptr) {
    auto *open_space_debug =
        debug->mutable_planning_data()->mutable_open_space();
    PopulateParkingDebug(open_space_debug, roi_geometry, pose_selection,
                         validation_result);
  }
}

void UpdateParkingValidationDebug(
    Frame *frame, const parking::ParkingRoiValidationResult &result) {
  ApplyParkingValidationDebug(
      frame->mutable_open_space_info()->mutable_parking_debug(), result);
  ApplyParkingValidationDebug(frame->mutable_open_space_info()
                                  ->mutable_debug_instance()
                                  ->mutable_planning_data()
                                  ->mutable_open_space(),
                              result);

  auto *debug = frame->mutable_open_space_info()->mutable_debug();
  if (debug != nullptr) {
    ApplyParkingValidationDebug(
        debug->mutable_planning_data()->mutable_open_space(), result);
  }
}

void ApplyParkingRoiToOpenSpaceInfo(
    Frame *frame, const parking::ParkingRoiGeometry &roi_geometry) {
  auto *mutable_open_space_info = frame->mutable_open_space_info();
  auto *xy_boundary = mutable_open_space_info->mutable_ROI_xy_boundary();
  xy_boundary->assign(roi_geometry.xy_boundary.begin(),
                      roi_geometry.xy_boundary.end());
  auto *roi_polygon =
      mutable_open_space_info->mutable_roi_parking_boundary_polygon();
  roi_polygon->assign(roi_geometry.union_polygon.begin(),
                      roi_geometry.union_polygon.end());
  mutable_open_space_info->set_roi_parking_area(roi_geometry.area);
  mutable_open_space_info->set_roi_parking_aisle_width(
      roi_geometry.aisle_width);
}

bool LoadParkingRoiGeometryFromOpenSpaceInfo(
    const OpenSpaceInfo &open_space_info, parking::ParkingRoiGeometry *geometry,
    std::string *error) {
  CHECK_NOTNULL(geometry);
  geometry->union_polygon = open_space_info.roi_parking_boundary_polygon();
  geometry->xy_boundary = open_space_info.ROI_xy_boundary();
  geometry->area = open_space_info.roi_parking_area();
  geometry->aisle_width = open_space_info.roi_parking_aisle_width();
  geometry->slot_polygon.clear();

  const auto &debug_instance = open_space_info.debug_instance();
  const auto &parking_roi_debug =
      debug_instance.planning_data().open_space().parking_roi();
  geometry->slot_polygon.reserve(
      static_cast<std::size_t>(parking_roi_debug.slot_polygon_size()));
  for (const auto &point : parking_roi_debug.slot_polygon()) {
    geometry->slot_polygon.emplace_back(point.x(), point.y());
  }
  geometry->bridge_polygon.clear();
  geometry->bridge_polygon.reserve(
      static_cast<std::size_t>(parking_roi_debug.bridge_polygon_size()));
  for (const auto &point : parking_roi_debug.bridge_polygon()) {
    geometry->bridge_polygon.emplace_back(point.x(), point.y());
  }
  geometry->attachment_polygon.clear();
  geometry->attachment_polygon.reserve(
      static_cast<std::size_t>(parking_roi_debug.attachment_polygon_size()));
  for (const auto &point : parking_roi_debug.attachment_polygon()) {
    geometry->attachment_polygon.emplace_back(point.x(), point.y());
  }
  if (geometry->attachment_polygon.empty()) {
    geometry->attachment_polygon = geometry->bridge_polygon;
  }
  geometry->connector_slice.clear();
  geometry->connector_slice.reserve(
      static_cast<std::size_t>(parking_roi_debug.connector_slice_size()));
  for (const auto &point : parking_roi_debug.connector_slice()) {
    geometry->connector_slice.emplace_back(point.x(), point.y());
  }
  geometry->outer_bridge_slice.clear();
  geometry->outer_bridge_slice.reserve(
      static_cast<std::size_t>(parking_roi_debug.outer_bridge_slice_size()));
  for (const auto &point : parking_roi_debug.outer_bridge_slice()) {
    geometry->outer_bridge_slice.emplace_back(point.x(), point.y());
  }

  if (geometry->union_polygon.size() < 3U ||
      geometry->xy_boundary.size() < 4U) {
    if (error != nullptr) {
      *error = "parking roi geometry unavailable for validation";
    }
    return false;
  }
  return true;
}

void GetAllLaneSegments(const routing::RoutingResponse &routing_response,
                        std::vector<routing::LaneSegment> *routing_segments) {
  CHECK_NOTNULL(routing_segments);
  routing_segments->clear();
  for (const auto &road : routing_response.road()) {
    for (const auto &passage : road.passage()) {
      for (const auto &lane_segment : passage.segment()) {
        routing_segments->push_back(lane_segment);
      }
    }
  }
}

bool ResolveTargetParkingSpotId(Frame *frame, std::string *parking_spot_id,
                                std::string *error) {
  CHECK_NOTNULL(parking_spot_id);
  const auto &routing_request = frame->local_view().routing->routing_request();
  if (routing_request.has_parking_info() &&
      routing_request.parking_info().has_parking_space_id()) {
    *parking_spot_id = routing_request.parking_info().parking_space_id();
    *frame->mutable_open_space_info()->mutable_target_parking_spot_id() =
        *parking_spot_id;
    return true;
  }
  if (error != nullptr) {
    *error = "failed to get parking space id from routing";
  }
  return false;
}

bool GetTargetParkingSpotById(const hdmap::HDMap *hdmap,
                              const std::string &parking_spot_id,
                              ParkingSpaceInfoConstPtr *target_parking_spot) {
  CHECK_NOTNULL(target_parking_spot);
  hdmap::Id id;
  id.set_id(parking_spot_id);
  *target_parking_spot = hdmap->GetParkingSpaceById(id);
  return *target_parking_spot != nullptr;
}

bool CheckDistanceToParkingSpot(
    const OpenSpaceRoiDeciderConfig &config, Frame *frame,
    const common::VehicleState &vehicle_state,
    const ParkingSpaceInfoConstPtr &target_parking_spot) {
  if (target_parking_spot == nullptr ||
      target_parking_spot->polygon().points().empty()) {
    AERROR << "target parking spot is invalid";
    return false;
  }

  Vec2d parking_spot_center;
  if (!apollo::planning::util::GetParkingSpotCenterFromRouting(
          *frame, &parking_spot_center)) {
    parking_spot_center = apollo::planning::util::GetParkingSpotCenterFromMap(
        target_parking_spot);
  }

  const Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
  return vehicle_vec.DistanceTo(parking_spot_center) <
         config.parking_start_range();
}

bool ResolveTargetParkingLane(
    const std::shared_ptr<DependencyInjector> &injector,
    const hdmap::HDMap *hdmap, Frame *frame,
    const common::VehicleState &vehicle_state,
    const std::string &parking_spot_id_string, LaneInfoConstPtr *nearest_lane,
    std::string *error) {
  CHECK_NOTNULL(nearest_lane);

  const auto &ptr_last_frame = injector->frame_history()->Latest();
  if (ptr_last_frame != nullptr) {
    const auto &previous_open_space_info = ptr_last_frame->open_space_info();
    if (previous_open_space_info.target_parking_lane() != nullptr &&
        previous_open_space_info.target_parking_spot_id() ==
            parking_spot_id_string) {
      *nearest_lane = previous_open_space_info.target_parking_lane();
      return true;
    }
  }

  hdmap::Id parking_spot_id = hdmap::MakeMapId(parking_spot_id_string);
  auto parking_spot = hdmap->GetParkingSpaceById(parking_spot_id);
  if (parking_spot == nullptr) {
    if (error != nullptr) {
      *error = "target parking spot id is invalid";
    }
    return false;
  }
  const auto overlap_ids = parking_spot->parking_space().overlap_id();
  if (overlap_ids.empty()) {
    if (error != nullptr) {
      *error = "target parking spot has no overlapped lane";
    }
    return false;
  }

  std::vector<routing::LaneSegment> lane_segments;
  GetAllLaneSegments(*frame->local_view().routing, &lane_segments);
  LaneInfoConstPtr first_overlapped_lane = nullptr;
  std::size_t nearest_lane_index = std::numeric_limits<std::size_t>::max();
  for (const auto &id : overlap_ids) {
    auto overlap = hdmap->GetOverlapById(id);
    if (overlap == nullptr) {
      continue;
    }
    for (const auto &object : overlap->overlap().object()) {
      if (!object.has_lane_overlap_info()) {
        continue;
      }
      auto candidate_lane = hdmap->GetLaneById(object.id());
      if (candidate_lane == nullptr) {
        continue;
      }
      if (first_overlapped_lane == nullptr) {
        first_overlapped_lane = candidate_lane;
      }
      std::size_t candidate_lane_index = 0U;
      bool candidate_on_route = false;
      for (const auto &segment : lane_segments) {
        if (segment.id() == candidate_lane->id().id()) {
          candidate_on_route = true;
          break;
        }
        ++candidate_lane_index;
      }
      if (candidate_on_route && candidate_lane_index < nearest_lane_index) {
        *nearest_lane = candidate_lane;
        nearest_lane_index = candidate_lane_index;
      }
    }
  }
  if (nearest_lane_index == std::numeric_limits<std::size_t>::max()) {
    if (first_overlapped_lane != nullptr) {
      *nearest_lane = first_overlapped_lane;
      AINFO << "Use parking-space overlapped lane "
            << first_overlapped_lane->id().id()
            << " for direct parking ROI; it is not in the routing response.";
      return true;
    }
    if (error != nullptr) {
      *error = "cannot find routing lane nearest to the parking spot";
    }
    return false;
  }

  LaneInfoConstPtr nearest_lane_to_vehicle;
  const auto point = common::util::PointFactory::ToPointENU(vehicle_state);
  double vehicle_lane_s = 0.0;
  double vehicle_lane_l = 0.0;
  const int status = hdmap->GetNearestLaneWithHeading(
      point, 10.0, vehicle_state.heading(), M_PI / 2.0,
      &nearest_lane_to_vehicle, &vehicle_lane_s, &vehicle_lane_l);
  if (status == 0 && nearest_lane_to_vehicle != nullptr) {
    std::size_t nearest_lane_to_vehicle_index = 0U;
    bool has_found_nearest_lane_to_vehicle = false;
    for (const auto &segment : lane_segments) {
      if (segment.id() == nearest_lane_to_vehicle->id().id()) {
        has_found_nearest_lane_to_vehicle = true;
        break;
      }
      ++nearest_lane_to_vehicle_index;
    }
    if (has_found_nearest_lane_to_vehicle &&
        nearest_lane_to_vehicle_index < nearest_lane_index) {
      *nearest_lane = nearest_lane_to_vehicle;
    }
  }
  return true;
}

bool BuildNearbyPath(const hdmap::HDMap *hdmap,
                     const LaneInfoConstPtr &nearest_lane, Path *nearby_path) {
  CHECK_NOTNULL(nearby_path);
  if (nearest_lane == nullptr) {
    return false;
  }

  LaneSegment nearest_lane_segment(nearest_lane,
                                   nearest_lane->accumulate_s().front(),
                                   nearest_lane->accumulate_s().back());
  std::vector<LaneSegment> segments_vector{nearest_lane_segment};
  const int next_lanes_num = nearest_lane->lane().successor_id_size();
  for (int i = 0; i < next_lanes_num; ++i) {
    auto next_lane = hdmap->GetLaneById(nearest_lane->lane().successor_id(i));
    if (next_lane == nullptr) {
      continue;
    }
    segments_vector.emplace_back(next_lane, next_lane->accumulate_s().front(),
                                 next_lane->accumulate_s().back());
    const int succeed_lanes_num = next_lane->lane().successor_id_size();
    for (int j = 0; j < succeed_lanes_num; ++j) {
      auto succeed_lane = hdmap->GetLaneById(next_lane->lane().successor_id(j));
      if (succeed_lane == nullptr) {
        continue;
      }
      segments_vector.emplace_back(succeed_lane,
                                   succeed_lane->accumulate_s().front(),
                                   succeed_lane->accumulate_s().back());
    }
  }
  *nearby_path = Path(segments_vector);
  return true;
}

bool BuildParkingRoiGeometryFromBoundaries(
    Frame *frame, const parking::ParkingSlot &map_slot, const Path &nearby_path,
    const std::vector<Vec2d> &left_boundary,
    const std::vector<Vec2d> &right_boundary,
    const std::vector<double> &left_boundary_s,
    const std::vector<double> &right_boundary_s,
    const bool allow_disconnected_roi_fallback,
    parking::ParkingRoiGeometry *roi_geometry,
    std::vector<std::vector<Vec2d>> *roi_parking_boundary, std::string *error) {
  CHECK_NOTNULL(roi_geometry);
  CHECK_NOTNULL(roi_parking_boundary);

  const auto &origin_point = frame->open_space_info().origin_point();
  const double origin_heading = frame->open_space_info().origin_heading();
  const parking::ParkingSlot normalized_slot =
      parking::TransformParkingSlot(map_slot, origin_point, origin_heading);

  double left_top_s = 0.0;
  double left_top_l = 0.0;
  double right_top_s = 0.0;
  double right_top_l = 0.0;
  if (!(nearby_path.GetProjection(map_slot.corners.left_top, &left_top_s,
                                  &left_top_l) &&
        nearby_path.GetProjection(map_slot.corners.right_top, &right_top_s,
                                  &right_top_l))) {
    if (error != nullptr) {
      *error = "failed to project parking slot opening onto nearby path";
    }
    return false;
  }

  const bool slot_on_left = (left_top_l + right_top_l) * 0.5 > 0.0;
  const auto &boundary_s = slot_on_left ? left_boundary_s : right_boundary_s;
  if (boundary_s.empty()) {
    if (error != nullptr) {
      *error = "parking roi side boundary sampling is empty";
    }
    return false;
  }

  const double open_start_s = std::min(left_top_s, right_top_s);
  const double open_end_s = std::max(left_top_s, right_top_s);
  std::size_t start_index = static_cast<std::size_t>(std::distance(
      boundary_s.begin(),
      std::lower_bound(boundary_s.begin(), boundary_s.end(), open_start_s)));
  if (start_index > 0U) {
    --start_index;
  }
  std::size_t end_index = static_cast<std::size_t>(std::distance(
      boundary_s.begin(),
      std::upper_bound(boundary_s.begin(), boundary_s.end(), open_end_s)));
  if (end_index >= boundary_s.size()) {
    end_index = boundary_s.size() - 1U;
  }

  parking::ParkingRoiBuildInput build_input;
  build_input.left_boundary = left_boundary;
  build_input.right_boundary = right_boundary;
  build_input.connection_start_index = start_index;
  build_input.connection_end_index = end_index;
  build_input.slot_on_left = slot_on_left;
  build_input.slot = normalized_slot;
  const auto &slot_side_outer_boundary =
      slot_on_left ? left_boundary : right_boundary;
  build_input.slot_side_connector_boundary = BuildSlotSideConnectorBoundary(
      nearby_path, slot_side_outer_boundary, boundary_s, start_index, end_index,
      left_top_s, left_top_l, right_top_s, right_top_l, origin_point,
      origin_heading);
  build_input.allow_disconnected_roi_fallback = allow_disconnected_roi_fallback;
  if (!parking::BuildParkingRoiGeometry(build_input, roi_geometry, error)) {
    return false;
  }
  roi_geometry->connection_start_index = start_index;
  roi_geometry->connection_end_index = end_index;
  roi_geometry->slot_side_boundary_size = boundary_s.size();
  roi_geometry->connector_boundary_size =
      build_input.slot_side_connector_boundary.size();

  *roi_parking_boundary = roi_geometry->boundary_segments;
  ApplyParkingRoiToOpenSpaceInfo(frame, *roi_geometry);
  return true;
}

}  // namespace

OpenSpaceRoiDecider::OpenSpaceRoiDecider(
    const TaskConfig &config,
    const std::shared_ptr<DependencyInjector> &injector)
    : Decider(config, injector) {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  vehicle_params_ =
      apollo::common::VehicleConfigHelper::GetConfig().vehicle_param();
  AINFO << config_.DebugString();
}

Status OpenSpaceRoiDecider::Process(Frame *frame) {
  if (frame == nullptr) {
    const std::string msg =
        "Invalid frame, fail to process the OpenSpaceRoiDecider.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  vehicle_state_ = frame->vehicle_state();
  obstacles_by_frame_ = frame->GetObstacleList();

  std::array<Vec2d, 4> spot_vertices;
  Path nearby_path;
  // @brief vector of different obstacle consisting of vertice points.The
  // obstacle and the vertices order are in counter-clockwise order
  std::vector<std::vector<common::math::Vec2d>> roi_boundary;

  const auto &roi_type = config_.open_space_roi_decider_config().roi_type();
  if (roi_type == OpenSpaceRoiDeciderConfig::PARKING) {
    std::string error;
    if (!GetParkingBoundary(frame, &roi_boundary, &error)) {
      const std::string msg =
          error.empty() ? "Fail to build parking ROI from map" : error;
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  } else if (roi_type == OpenSpaceRoiDeciderConfig::PULL_OVER) {
    if (!GetPullOverSpot(frame, &spot_vertices, &nearby_path)) {
      const std::string msg = "Fail to get parking boundary from map";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    SetOrigin(frame, spot_vertices);

    SetPullOverSpotEndPose(frame);

    if (!GetPullOverBoundary(frame, spot_vertices, nearby_path,
                             &roi_boundary)) {
      const std::string msg = "Fail to get parking boundary from map";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  } else if (roi_type == OpenSpaceRoiDeciderConfig::PARK_AND_GO) {
    ADEBUG << "in Park_and_Go";
    nearby_path =
        frame->reference_line_info().front().reference_line().GetMapPath();

    ADEBUG << "nearby_path: " << nearby_path.DebugString();
    ADEBUG << "found nearby_path";
    if (!injector_->planning_context()
             ->planning_status()
             .park_and_go()
             .has_adc_init_position()) {
      const std::string msg = "ADC initial position is unavailable";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    SetOriginFromADC(frame, nearby_path);
    ADEBUG << "SetOrigin";
    SetParkAndGoEndPose(frame);
    ADEBUG << "SetEndPose";
    if (!GetParkAndGoBoundary(frame, nearby_path, &roi_boundary)) {
      const std::string msg = "Fail to get park and go boundary from map";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  } else {
    const std::string msg =
        "chosen open space roi secenario type not implemented";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!FormulateBoundaryConstraints(roi_boundary, frame)) {
    const std::string msg = "Fail to formulate boundary constraints";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  return Status::OK();
}

// get origin from ADC
void OpenSpaceRoiDecider::SetOriginFromADC(Frame *const frame,
                                           const hdmap::Path &nearby_path) {
  // get ADC box
  const auto &park_and_go_status =
      injector_->planning_context()->planning_status().park_and_go();

  const double adc_init_x = park_and_go_status.adc_init_position().x();
  const double adc_init_y = park_and_go_status.adc_init_position().y();
  const double adc_init_heading = park_and_go_status.adc_init_heading();
  common::math::Vec2d adc_init_position = {adc_init_x, adc_init_y};
  const double adc_length = vehicle_params_.length();
  const double adc_width = vehicle_params_.width();
  // ADC box
  Box2d adc_box(adc_init_position, adc_init_heading, adc_length, adc_width);
  // get vertices from ADC box
  std::vector<common::math::Vec2d> adc_corners;
  adc_box.GetAllCorners(&adc_corners);
  for (size_t i = 0; i < adc_corners.size(); ++i) {
    ADEBUG << "ADC [" << i << "]x: " << std::setprecision(9)
           << adc_corners[i].x();
    ADEBUG << "ADC [" << i << "]y: " << std::setprecision(9)
           << adc_corners[i].y();
  }
  auto left_top = adc_corners[3];

  ADEBUG << "left_top x: " << std::setprecision(9) << left_top.x();
  ADEBUG << "left_top y: " << std::setprecision(9) << left_top.y();

  // rotate the points to have the lane to be horizontal to x axis positive
  // direction and scale them base on the origin point
  // heading angle
  double heading;
  if (!nearby_path.GetHeadingAlongPath(left_top, &heading)) {
    AERROR << "fail to get heading on reference line";
    return;
  }

  frame->mutable_open_space_info()->set_origin_heading(
      common::math::NormalizeAngle(heading));
  ADEBUG << "heading: " << heading;
  frame->mutable_open_space_info()->mutable_origin_point()->set_x(left_top.x());
  frame->mutable_open_space_info()->mutable_origin_point()->set_y(left_top.y());
}

void OpenSpaceRoiDecider::SetOrigin(
    Frame *const frame, const std::array<common::math::Vec2d, 4> &vertices) {
  auto left_top = vertices[0];
  auto right_top = vertices[3];
  // rotate the points to have the lane to be horizontal to x axis positive
  // direction and scale them base on the origin point
  Vec2d heading_vec = right_top - left_top;
  frame->mutable_open_space_info()->set_origin_heading(heading_vec.Angle());
  frame->mutable_open_space_info()->mutable_origin_point()->set_x(left_top.x());
  frame->mutable_open_space_info()->mutable_origin_point()->set_y(left_top.y());
}

void OpenSpaceRoiDecider::SetPullOverSpotEndPose(Frame *const frame) {
  const auto &pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  const double pull_over_x = pull_over_status.position().x();
  const double pull_over_y = pull_over_status.position().y();
  double pull_over_theta = pull_over_status.theta();

  // Normalize according to origin_point and origin_heading
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();
  Vec2d center(pull_over_x, pull_over_y);
  center -= origin_point;
  center.SelfRotate(-origin_heading);
  pull_over_theta =
      common::math::NormalizeAngle(pull_over_theta - origin_heading);

  auto *end_pose =
      frame->mutable_open_space_info()->mutable_open_space_end_pose();
  end_pose->clear();
  end_pose->push_back(center.x());
  end_pose->push_back(center.y());
  end_pose->push_back(pull_over_theta);
  // end pose velocity set to be zero
  end_pose->push_back(0.0);
}

void OpenSpaceRoiDecider::SetParkAndGoEndPose(Frame *const frame) {
  const double kSTargetBuffer =
      config_.open_space_roi_decider_config().end_pose_s_distance();
  const double kSpeedRatio = 0.1;  // after adjust speed is 10% of speed limit
  // get vehicle current location
  // get vehicle s,l info
  auto park_and_go_status = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_park_and_go();

  const double adc_init_x = park_and_go_status->adc_init_position().x();
  const double adc_init_y = park_and_go_status->adc_init_position().y();

  ADEBUG << "ADC position (x): " << std::setprecision(9) << adc_init_x;
  ADEBUG << "ADC position (y): " << std::setprecision(9) << adc_init_y;

  const common::math::Vec2d adc_position = {adc_init_x, adc_init_y};
  common::SLPoint adc_position_sl;

  // get nearest reference line
  const auto &reference_line_list = frame->reference_line_info();
  ADEBUG << reference_line_list.size();
  const auto reference_line_info = std::min_element(
      reference_line_list.begin(), reference_line_list.end(),
      [&](const ReferenceLineInfo &ref_a, const ReferenceLineInfo &ref_b) {
        common::SLPoint adc_position_sl_a;
        common::SLPoint adc_position_sl_b;
        ref_a.reference_line().XYToSL(adc_position, &adc_position_sl_a);
        ref_b.reference_line().XYToSL(adc_position, &adc_position_sl_b);
        return std::fabs(adc_position_sl_a.l()) <
               std::fabs(adc_position_sl_b.l());
      });

  const auto &reference_line = reference_line_info->reference_line();
  reference_line.XYToSL(adc_position, &adc_position_sl);

  // target is at reference line
  const double target_s = adc_position_sl.s() + kSTargetBuffer;
  const auto reference_point = reference_line.GetReferencePoint(target_s);
  const double target_x = reference_point.x();
  const double target_y = reference_point.y();
  double target_theta = reference_point.heading();

  park_and_go_status->mutable_adc_adjust_end_pose()->set_x(target_x);
  park_and_go_status->mutable_adc_adjust_end_pose()->set_y(target_y);

  ADEBUG << "center.x(): " << std::setprecision(9) << target_x;
  ADEBUG << "center.y(): " << std::setprecision(9) << target_y;
  ADEBUG << "target_theta: " << std::setprecision(9) << target_theta;

  // Normalize according to origin_point and origin_heading
  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();
  Vec2d center(target_x, target_y);
  center -= origin_point;
  center.SelfRotate(-origin_heading);
  target_theta = common::math::NormalizeAngle(target_theta - origin_heading);

  auto *end_pose =
      frame->mutable_open_space_info()->mutable_open_space_end_pose();
  end_pose->clear();

  end_pose->push_back(center.x());
  end_pose->push_back(center.y());
  end_pose->push_back(target_theta);

  ADEBUG << "ADC position (x): " << std::setprecision(9) << (*end_pose)[0];
  ADEBUG << "ADC position (y): " << std::setprecision(9) << (*end_pose)[1];
  ADEBUG << "reference_line ID: " << reference_line_info->Lanes().Id();

  // end pose velocity set to be speed limit
  double target_speed = reference_line.GetSpeedLimitFromS(target_s);
  end_pose->push_back(kSpeedRatio * target_speed);
}

void OpenSpaceRoiDecider::GetRoadBoundary(
    const hdmap::Path &nearby_path, const double center_line_s,
    const common::math::Vec2d &origin_point, const double origin_heading,
    std::vector<Vec2d> *left_lane_boundary,
    std::vector<Vec2d> *right_lane_boundary,
    std::vector<Vec2d> *center_lane_boundary_left,
    std::vector<Vec2d> *center_lane_boundary_right,
    std::vector<double> *center_lane_s_left,
    std::vector<double> *center_lane_s_right,
    std::vector<double> *left_lane_road_width,
    std::vector<double> *right_lane_road_width) {
  double start_s =
      center_line_s -
      config_.open_space_roi_decider_config().roi_longitudinal_range_start();
  double end_s =
      center_line_s +
      config_.open_space_roi_decider_config().roi_longitudinal_range_end();

  hdmap::MapPathPoint start_point = nearby_path.GetSmoothPoint(start_s);
  double last_check_point_heading = start_point.heading();
  double index = 0.0;
  double check_point_s = start_s;

  // For the road boundary, add key points to left/right side boundary
  // separately. Iterate s_value to check key points at a step of
  // roi_line_segment_length. Key points include: start_point, end_point, points
  // where path curvature is large, points near left/right road-curb corners
  while (check_point_s <= end_s) {
    hdmap::MapPathPoint check_point = nearby_path.GetSmoothPoint(check_point_s);
    double check_point_heading = check_point.heading();
    bool is_center_lane_heading_change =
        std::abs(common::math::NormalizeAngle(check_point_heading -
                                              last_check_point_heading)) >
        config_.open_space_roi_decider_config().roi_line_segment_min_angle();
    last_check_point_heading = check_point_heading;

    ADEBUG << "is is_center_lane_heading_change: "
           << is_center_lane_heading_change;
    // Check if the current center-lane checking-point is start point || end
    // point || or point with larger curvature. If yes, mark it as an anchor
    // point.
    bool is_anchor_point = check_point_s == start_s || check_point_s == end_s ||
                           is_center_lane_heading_change;
    // Add key points to the left-half boundary
    AddBoundaryKeyPoint(nearby_path, check_point_s, start_s, end_s,
                        is_anchor_point, true, center_lane_boundary_left,
                        left_lane_boundary, center_lane_s_left,
                        left_lane_road_width);
    // Add key points to the right-half boundary
    AddBoundaryKeyPoint(nearby_path, check_point_s, start_s, end_s,
                        is_anchor_point, false, center_lane_boundary_right,
                        right_lane_boundary, center_lane_s_right,
                        right_lane_road_width);
    if (check_point_s == end_s) {
      break;
    }
    index += 1.0;
    check_point_s =
        start_s +
        index *
            config_.open_space_roi_decider_config().roi_line_segment_length();
    check_point_s = check_point_s >= end_s ? end_s : check_point_s;
  }

  size_t left_point_size = left_lane_boundary->size();
  size_t right_point_size = right_lane_boundary->size();
  for (size_t i = 0; i < left_point_size; i++) {
    left_lane_boundary->at(i) -= origin_point;
    left_lane_boundary->at(i).SelfRotate(-origin_heading);
  }
  for (size_t i = 0; i < right_point_size; i++) {
    right_lane_boundary->at(i) -= origin_point;
    right_lane_boundary->at(i).SelfRotate(-origin_heading);
  }
}

void OpenSpaceRoiDecider::GetRoadBoundaryFromMap(
    const hdmap::Path &nearby_path, const double center_line_s,
    const double sampling_start_s, const double sampling_end_s,
    const Vec2d &origin_point, const double origin_heading,
    std::vector<Vec2d> *left_lane_boundary,
    std::vector<Vec2d> *right_lane_boundary,
    std::vector<Vec2d> *center_lane_boundary_left,
    std::vector<Vec2d> *center_lane_boundary_right,
    std::vector<double> *center_lane_s_left,
    std::vector<double> *center_lane_s_right,
    std::vector<double> *left_lane_road_width,
    std::vector<double> *right_lane_road_width) {
  // Longitudinal range can be asymmetric.
  double start_s = sampling_start_s;
  double end_s = sampling_end_s;
  start_s = std::max(0.0, std::min(start_s, nearby_path.length()));
  end_s = std::max(start_s, std::min(end_s, nearby_path.length()));
  hdmap::MapPathPoint start_point = nearby_path.GetSmoothPoint(start_s);

  double check_point_s = start_s;

  while (check_point_s <= end_s + common::math::kMathEpsilon) {
    hdmap::MapPathPoint check_point = nearby_path.GetSmoothPoint(check_point_s);

    // get road boundaries
    double left_road_width = nearby_path.GetRoadLeftWidth(check_point_s);
    double right_road_width = nearby_path.GetRoadRightWidth(check_point_s);

    double current_road_width = std::max(left_road_width, right_road_width);

    // get road boundaries at current location
    common::PointENU check_point_xy;
    std::vector<hdmap::RoadRoiPtr> road_boundaries;
    std::vector<hdmap::JunctionInfoConstPtr> junctions;
    check_point_xy.set_x(check_point.x());
    check_point_xy.set_y(check_point.y());
    hdmap_->GetRoadBoundaries(check_point_xy, current_road_width,
                              &road_boundaries, &junctions);

    if (check_point_s < center_line_s) {
      for (size_t i = 0;
           i < (*road_boundaries.at(0)).left_boundary.line_points.size(); i++) {
        right_lane_boundary->emplace_back(
            Vec2d((*road_boundaries.at(0)).left_boundary.line_points[i].x(),
                  (*road_boundaries.at(0)).left_boundary.line_points[i].y()));
      }
      for (size_t i = 0;
           i < (*road_boundaries.at(0)).right_boundary.line_points.size();
           i++) {
        left_lane_boundary->emplace_back(
            Vec2d((*road_boundaries.at(0)).right_boundary.line_points[i].x(),
                  (*road_boundaries.at(0)).right_boundary.line_points[i].y()));
      }
    } else {
      for (size_t i = 0;
           i < (*road_boundaries.at(0)).left_boundary.line_points.size(); i++) {
        left_lane_boundary->emplace_back(
            Vec2d((*road_boundaries.at(0)).left_boundary.line_points[i].x(),
                  (*road_boundaries.at(0)).left_boundary.line_points[i].y()));
      }
      for (size_t i = 0;
           i < (*road_boundaries.at(0)).right_boundary.line_points.size();
           i++) {
        right_lane_boundary->emplace_back(
            Vec2d((*road_boundaries.at(0)).right_boundary.line_points[i].x(),
                  (*road_boundaries.at(0)).right_boundary.line_points[i].y()));
      }
    }

    center_lane_boundary_right->emplace_back(check_point);
    center_lane_boundary_left->emplace_back(check_point);
    center_lane_s_left->emplace_back(check_point_s);
    center_lane_s_right->emplace_back(check_point_s);
    left_lane_road_width->emplace_back(left_road_width);
    right_lane_road_width->emplace_back(right_road_width);

    check_point_s = std::min(
        end_s, check_point_s + config_.open_space_roi_decider_config()
                                   .roi_line_segment_length_from_map());
    if (check_point_s >= end_s) {
      if (center_lane_s_left->empty() ||
          std::fabs(center_lane_s_left->back() - end_s) >
              common::math::kMathEpsilon) {
        continue;
      }
      break;
    }
  }

  size_t left_point_size = left_lane_boundary->size();
  size_t right_point_size = right_lane_boundary->size();
  ADEBUG << "right_road_boundary size: " << right_lane_boundary->size();
  ADEBUG << "left_road_boundary size: " << left_lane_boundary->size();
  for (size_t i = 0; i < left_point_size; i++) {
    left_lane_boundary->at(i) -= origin_point;
    left_lane_boundary->at(i).SelfRotate(-origin_heading);
    ADEBUG << "left_road_boundary: [" << std::setprecision(9)
           << left_lane_boundary->at(i).x() << ", "
           << left_lane_boundary->at(i).y() << "]";
  }
  for (size_t i = 0; i < right_point_size; i++) {
    right_lane_boundary->at(i) -= origin_point;
    right_lane_boundary->at(i).SelfRotate(-origin_heading);
    ADEBUG << "right_road_boundary: [" << std::setprecision(9)
           << right_lane_boundary->at(i).x() << ", "
           << right_lane_boundary->at(i).y() << "]";
  }
  if (!left_lane_boundary->empty()) {
    sort(left_lane_boundary->begin(), left_lane_boundary->end(),
         [](const Vec2d &first_pt, const Vec2d &second_pt) {
           return first_pt.x() < second_pt.x() ||
                  (first_pt.x() == second_pt.x() &&
                   first_pt.y() < second_pt.y());
         });
    auto unique_end =
        std::unique(left_lane_boundary->begin(), left_lane_boundary->end());
    left_lane_boundary->erase(unique_end, left_lane_boundary->end());
  }
  if (!right_lane_boundary->empty()) {
    sort(right_lane_boundary->begin(), right_lane_boundary->end(),
         [](const Vec2d &first_pt, const Vec2d &second_pt) {
           return first_pt.x() < second_pt.x() ||
                  (first_pt.x() == second_pt.x() &&
                   first_pt.y() < second_pt.y());
         });
    auto unique_end =
        std::unique(right_lane_boundary->begin(), right_lane_boundary->end());
    right_lane_boundary->erase(unique_end, right_lane_boundary->end());
  }
}

void OpenSpaceRoiDecider::AddBoundaryKeyPoint(
    const hdmap::Path &nearby_path, const double check_point_s,
    const double start_s, const double end_s, const bool is_anchor_point,
    const bool is_left_curb, std::vector<Vec2d> *center_lane_boundary,
    std::vector<Vec2d> *curb_lane_boundary, std::vector<double> *center_lane_s,
    std::vector<double> *road_width) {
  // Check if current central-lane checking point's mapping on the left/right
  // road boundary is a key point. The road boundary point is a key point if
  // one of the following two confitions is satisfied:
  // 1. the current central-lane point is an anchor point: (a start/end point
  // or the point on path with large curvatures)
  // 2. the point on the left/right lane boundary is close to a curb corner
  // As indicated below:
  // (#) Key Point Type 1: Lane anchor points
  // (*) Key Point Type 2: Curb-corner points
  //                                                         #
  // Path Direction -->                                     /    /   #
  // Left Lane Boundary   #--------------------------------#    /   /
  //                                                           /   /
  // Center Lane          - - - - - - - - - - - - - - - - - - /   /
  //                                                             /
  // Right Lane Boundary  #--------*                 *----------#
  //                                \               /
  //                                 *-------------*

  // road width changes slightly at the turning point of a path
  // TODO(SHU): 1. consider distortion introduced by curvy road; 2. use both
  // round boundaries for single-track road; 3. longitudinal range may not be
  // symmetric
  const double previous_distance_s = std::min(
      config_.open_space_roi_decider_config().roi_line_segment_length(),
      check_point_s - start_s);
  const double next_distance_s = std::min(
      config_.open_space_roi_decider_config().roi_line_segment_length(),
      end_s - check_point_s);

  hdmap::MapPathPoint current_check_point =
      nearby_path.GetSmoothPoint(check_point_s);
  hdmap::MapPathPoint previous_check_point =
      nearby_path.GetSmoothPoint(check_point_s - previous_distance_s);
  hdmap::MapPathPoint next_check_point =
      nearby_path.GetSmoothPoint(check_point_s + next_distance_s);

  double current_check_point_heading = current_check_point.heading();
  double current_road_width =
      is_left_curb ? nearby_path.GetRoadLeftWidth(check_point_s)
                   : nearby_path.GetRoadRightWidth(check_point_s);
  // If the current center-lane checking point is an anchor point, then add
  // current left/right curb boundary point as a key point
  if (is_anchor_point) {
    double point_vec_cos =
        is_left_curb ? std::cos(current_check_point_heading + M_PI / 2.0)
                     : std::cos(current_check_point_heading - M_PI / 2.0);
    double point_vec_sin =
        is_left_curb ? std::sin(current_check_point_heading + M_PI / 2.0)
                     : std::sin(current_check_point_heading - M_PI / 2.0);
    Vec2d curb_lane_point = Vec2d(current_road_width * point_vec_cos,
                                  current_road_width * point_vec_sin);
    curb_lane_point = curb_lane_point + current_check_point;
    center_lane_boundary->push_back(current_check_point);
    curb_lane_boundary->push_back(curb_lane_point);
    center_lane_s->push_back(check_point_s);
    road_width->push_back(current_road_width);
    return;
  }
  double previous_road_width =
      is_left_curb
          ? nearby_path.GetRoadLeftWidth(check_point_s - previous_distance_s)
          : nearby_path.GetRoadRightWidth(check_point_s - previous_distance_s);
  double next_road_width =
      is_left_curb
          ? nearby_path.GetRoadLeftWidth(check_point_s + next_distance_s)
          : nearby_path.GetRoadRightWidth(check_point_s + next_distance_s);
  double previous_curb_segment_angle =
      (current_road_width - previous_road_width) / previous_distance_s;
  double next_segment_angle =
      (next_road_width - current_road_width) / next_distance_s;
  double current_curb_point_delta_theta =
      next_segment_angle - previous_curb_segment_angle;
  // If the delta angle between the previous curb segment and the next curb
  // segment is large (near a curb corner), then add current curb_lane_point
  // as a key point.
  if (std::abs(current_curb_point_delta_theta) >
      config_.open_space_roi_decider_config()
          .curb_heading_tangent_change_upper_limit()) {
    double point_vec_cos =
        is_left_curb ? std::cos(current_check_point_heading + M_PI / 2.0)
                     : std::cos(current_check_point_heading - M_PI / 2.0);
    double point_vec_sin =
        is_left_curb ? std::sin(current_check_point_heading + M_PI / 2.0)
                     : std::sin(current_check_point_heading - M_PI / 2.0);
    Vec2d curb_lane_point = Vec2d(current_road_width * point_vec_cos,
                                  current_road_width * point_vec_sin);
    curb_lane_point = curb_lane_point + current_check_point;
    center_lane_boundary->push_back(current_check_point);
    curb_lane_boundary->push_back(curb_lane_point);
    center_lane_s->push_back(check_point_s);
    road_width->push_back(current_road_width);
  }
}

bool OpenSpaceRoiDecider::GetPullOverBoundary(
    Frame *const frame, const std::array<common::math::Vec2d, 4> &vertices,
    const hdmap::Path &nearby_path,
    std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
  auto left_top = vertices[0];
  auto left_down = vertices[1];
  auto right_down = vertices[2];
  auto right_top = vertices[3];

  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();

  double left_top_s = 0.0;
  double left_top_l = 0.0;
  double right_top_s = 0.0;
  double right_top_l = 0.0;
  if (!(nearby_path.GetProjection(left_top, &left_top_s, &left_top_l) &&
        nearby_path.GetProjection(right_top, &right_top_s, &right_top_l))) {
    AERROR << "fail to get parking spot points' projections on reference line";
    return false;
  }

  left_top -= origin_point;
  left_top.SelfRotate(-origin_heading);
  left_down -= origin_point;
  left_down.SelfRotate(-origin_heading);
  right_top -= origin_point;
  right_top.SelfRotate(-origin_heading);
  right_down -= origin_point;
  right_down.SelfRotate(-origin_heading);

  const double center_line_s = (left_top_s + right_top_s) / 2.0;
  std::vector<Vec2d> left_lane_boundary;
  std::vector<Vec2d> right_lane_boundary;
  std::vector<Vec2d> center_lane_boundary_left;
  std::vector<Vec2d> center_lane_boundary_right;
  std::vector<double> center_lane_s_left;
  std::vector<double> center_lane_s_right;
  std::vector<double> left_lane_road_width;
  std::vector<double> right_lane_road_width;

  GetRoadBoundary(nearby_path, center_line_s, origin_point, origin_heading,
                  &left_lane_boundary, &right_lane_boundary,
                  &center_lane_boundary_left, &center_lane_boundary_right,
                  &center_lane_s_left, &center_lane_s_right,
                  &left_lane_road_width, &right_lane_road_width);

  // Load boundary as line segments in counter-clockwise order
  std::reverse(left_lane_boundary.begin(), left_lane_boundary.end());

  std::vector<Vec2d> boundary_points;
  std::copy(right_lane_boundary.begin(), right_lane_boundary.end(),
            std::back_inserter(boundary_points));
  std::copy(left_lane_boundary.begin(), left_lane_boundary.end(),
            std::back_inserter(boundary_points));

  size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
  for (size_t i = 0; i < right_lane_boundary_last_index; i++) {
    std::vector<Vec2d> segment{right_lane_boundary[i],
                               right_lane_boundary[i + 1]};
    roi_parking_boundary->push_back(segment);
  }

  size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
  for (size_t i = left_lane_boundary_last_index; i > 0; i--) {
    std::vector<Vec2d> segment{left_lane_boundary[i],
                               left_lane_boundary[i - 1]};
    roi_parking_boundary->push_back(segment);
  }

  // Fuse line segments into convex contraints
  if (!FuseLineSegments(roi_parking_boundary)) {
    return false;
  }
  // Get xy boundary
  auto xminmax = std::minmax_element(
      boundary_points.begin(), boundary_points.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.x() < b.x(); });
  auto yminmax = std::minmax_element(
      boundary_points.begin(), boundary_points.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.y() < b.y(); });
  std::vector<double> ROI_xy_boundary{xminmax.first->x(), xminmax.second->x(),
                                      yminmax.first->y(), yminmax.second->y()};
  auto *xy_boundary =
      frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
  xy_boundary->assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());

  Vec2d vehicle_xy = Vec2d(vehicle_state_.x(), vehicle_state_.y());
  vehicle_xy -= origin_point;
  vehicle_xy.SelfRotate(-origin_heading);
  if (vehicle_xy.x() < ROI_xy_boundary[0] ||
      vehicle_xy.x() > ROI_xy_boundary[1] ||
      vehicle_xy.y() < ROI_xy_boundary[2] ||
      vehicle_xy.y() > ROI_xy_boundary[3]) {
    AERROR << "vehicle outside of xy boundary of parking ROI";
    return false;
  }
  return true;
}

bool OpenSpaceRoiDecider::GetParkingBoundary(
    Frame *const frame,
    std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary,
    std::string *error) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(roi_parking_boundary);
  roi_parking_boundary->clear();

  std::string parking_spot_id;
  if (!ResolveTargetParkingSpotId(frame, &parking_spot_id, error)) {
    return false;
  }

  LaneInfoConstPtr nearest_lane;
  if (!ResolveTargetParkingLane(injector_, hdmap_, frame, vehicle_state_,
                                parking_spot_id, &nearest_lane, error)) {
    return false;
  }
  frame->mutable_open_space_info()->set_target_parking_lane(nearest_lane);

  ParkingSpaceInfoConstPtr target_parking_spot = nullptr;
  if (!GetTargetParkingSpotById(hdmap_, parking_spot_id,
                                &target_parking_spot)) {
    if (error != nullptr) {
      *error = "no such parking spot found in hdmap";
    }
    return false;
  }
  *frame->mutable_open_space_info()->mutable_target_parking_spot() =
      target_parking_spot;

  Path nearby_path;
  if (!BuildNearbyPath(hdmap_, nearest_lane, &nearby_path)) {
    if (error != nullptr) {
      *error = "failed to build nearby path for target parking spot";
    }
    return false;
  }
  const auto &roi_config = config_.open_space_roi_decider_config();
  if (!CheckDistanceToParkingSpot(roi_config, frame, vehicle_state_,
                                  target_parking_spot)) {
    AINFO_EVERY(20) << "Target parking spot is farther than "
                    << "parking_start_range; continue direct parking ROI "
                    << "construction and let ROI/warm-start validation decide";
  }

  parking::ParkingSlot parking_slot;
  parking::ParkingSlotProvider provider;
  Vec2d parking_entry_reference(vehicle_state_.x(), vehicle_state_.y());
  if (last_parking_entry_spot_id_ == parking_spot_id &&
      last_parking_entry_reference_.Length() > common::math::kMathEpsilon) {
    parking_entry_reference = last_parking_entry_reference_;
  } else if (injector_ != nullptr && injector_->frame_history() != nullptr) {
    if (const auto *last_frame = injector_->frame_history()->Latest();
        last_frame != nullptr &&
        last_frame->open_space_info().target_parking_spot_id() ==
            parking_spot_id &&
        last_frame->open_space_info().origin_point().Length() >
            common::math::kMathEpsilon) {
      parking_entry_reference = last_frame->open_space_info().origin_point();
    }
  }
  if (!provider.BuildFromMap(target_parking_spot, nearby_path,
                             parking_entry_reference, &parking_slot, error)) {
    return false;
  }

  last_parking_entry_spot_id_ = parking_spot_id;
  last_parking_entry_reference_ = parking_slot.opening_center;
  ApplyParkingOrigin(frame, DetermineParkingOrigin(parking_slot));

  double left_top_s = 0.0;
  double left_top_l = 0.0;
  double right_top_s = 0.0;
  double right_top_l = 0.0;
  if (!(nearby_path.GetProjection(parking_slot.corners.left_top, &left_top_s,
                                  &left_top_l) &&
        nearby_path.GetProjection(parking_slot.corners.right_top, &right_top_s,
                                  &right_top_l))) {
    if (error != nullptr) {
      *error = "failed to project parking slot opening on nearby path";
    }
    return false;
  }
  const double center_line_s = 0.5 * (left_top_s + right_top_s);
  const auto &origin_point = frame->open_space_info().origin_point();
  const double origin_heading = frame->open_space_info().origin_heading();
  double vehicle_s = 0.0;
  double vehicle_l = 0.0;
  if (!nearby_path.GetProjection(Vec2d(vehicle_state_.x(), vehicle_state_.y()),
                                 &vehicle_s, &vehicle_l)) {
    if (error != nullptr) {
      *error = "failed to project ego on nearby path for parking roi";
    }
    return false;
  }
  double sampling_start_s = 0.0;
  double sampling_end_s = 0.0;
  ComputeParkingRoiSamplingRange(roi_config, nearby_path, center_line_s,
                                 vehicle_s, vehicle_params_, &sampling_start_s,
                                 &sampling_end_s);

  std::vector<Vec2d> left_lane_boundary;
  std::vector<Vec2d> right_lane_boundary;
  std::vector<Vec2d> center_lane_boundary_left;
  std::vector<Vec2d> center_lane_boundary_right;
  std::vector<double> center_lane_s_left;
  std::vector<double> center_lane_s_right;
  std::vector<double> left_lane_road_width;
  std::vector<double> right_lane_road_width;
  if (FLAGS_use_road_boundary_from_map) {
    GetRoadBoundaryFromMap(
        nearby_path, center_line_s, sampling_start_s, sampling_end_s,
        origin_point, origin_heading, &left_lane_boundary, &right_lane_boundary,
        &center_lane_boundary_left, &center_lane_boundary_right,
        &center_lane_s_left, &center_lane_s_right, &left_lane_road_width,
        &right_lane_road_width);
  } else {
    SampleParkingRoadBoundariesDense(
        nearby_path, sampling_start_s, sampling_end_s, origin_point,
        origin_heading, &left_lane_boundary, &right_lane_boundary,
        &center_lane_s_left, &center_lane_s_right, &left_lane_road_width,
        &right_lane_road_width);
  }

  parking::ParkingRoiGeometry roi_geometry;
  if (!BuildParkingRoiGeometryFromBoundaries(
          frame, parking_slot, nearby_path, left_lane_boundary,
          right_lane_boundary, center_lane_s_left, center_lane_s_right,
          true, &roi_geometry, roi_parking_boundary, error)) {
    parking::ParkingPoseSelection empty_selection;
    parking::ParkingRoiValidationResult invalid_result;
    invalid_result.reason = error == nullptr ? "roi build failed" : *error;
    UpdateParkingDebug(frame, roi_geometry, empty_selection, &invalid_result);
    return false;
  }

  const parking::ParkingSlot normalized_slot =
      parking::TransformParkingSlot(parking_slot, origin_point, origin_heading);
  Vec2d vehicle_xy(vehicle_state_.x(), vehicle_state_.y());
  vehicle_xy -= origin_point;
  vehicle_xy.SelfRotate(-origin_heading);
  const double vehicle_heading =
      common::math::NormalizeAngle(vehicle_state_.heading() - origin_heading);
  double start_escape_distance = roi_config.candidate_path_step_size();
  double start_pose_buffer = 0.1;
  if (roi_config.has_candidate_warm_start_config()) {
    start_escape_distance =
        std::max(start_escape_distance,
                 roi_config.candidate_warm_start_config().step_size());
    start_pose_buffer = std::clamp(
        0.5 * roi_config.candidate_warm_start_config().xy_grid_resolution(),
        0.05, 0.15);
  }
  if (normalized_slot.type == parking::ParkingSlotType::kAngled) {
    start_escape_distance =
        std::max(start_escape_distance, 0.5 * vehicle_params_.wheel_base());
  }

  if (!parking::ExpandParkingRoiToIncludeVehicleFootprint(
          vehicle_xy, vehicle_heading, vehicle_params_, start_pose_buffer,
          start_escape_distance, &roi_geometry, error, true)) {
    parking::ParkingPoseSelection empty_selection;
    parking::ParkingRoiValidationResult invalid_result;
    invalid_result.reason =
        error == nullptr ? "roi ego-footprint expansion failed" : *error;
    UpdateParkingDebug(frame, roi_geometry, empty_selection, &invalid_result);
    return false;
  }
  ApplyParkingRoiToOpenSpaceInfo(frame, roi_geometry);
  *roi_parking_boundary = roi_geometry.boundary_segments;

  parking::ParkingRoiValidator roi_validator(
      config_.open_space_roi_decider_config());
  const auto roi_geometry_validation = roi_validator.ValidateGeometryOnly(
      roi_geometry, vehicle_xy, vehicle_heading, vehicle_params_);
  if (!roi_geometry_validation.valid) {
    AINFO << "Parking map ROI validation before start-goal template failed: "
          << roi_geometry_validation.reason;
  }

  parking::ParkingPoseSelection pose_selection;
  if (!TryReuseParkingEndPoseFromPreviousFrame(
          injector_, parking_spot_id, roi_geometry, vehicle_xy, vehicle_params_,
          config_.open_space_roi_decider_config(), roi_validator, frame,
          &pose_selection)) {
    parking::ParkingPoseSelector pose_selector(
        config_.open_space_roi_decider_config());
    pose_selection =
        pose_selector.Select(normalized_slot, roi_geometry, vehicle_params_,
                             vehicle_xy, vehicle_heading);
  }
  if (!pose_selection.has_feasible_candidate()) {
    UpdateParkingDebug(frame, roi_geometry, pose_selection,
                       &roi_geometry_validation);
    if (error != nullptr) {
      *error = "no feasible parking end pose candidate";
    }
    return false;
  }

  const parking::ParkingRoiGeometry roi_geometry_seed = roi_geometry;
  std::vector<int> candidate_indices;
  candidate_indices.reserve(pose_selection.candidates.size());
  candidate_indices.push_back(pose_selection.selected_index);
  for (int index = 0; index < static_cast<int>(pose_selection.candidates.size());
       ++index) {
    if (index != pose_selection.selected_index) {
      candidate_indices.push_back(index);
    }
  }
  parking::ParkingRoiValidationResult final_validation_result;
  bool has_selected_candidate = false;
  for (const int candidate_index : candidate_indices) {
    if (candidate_index < 0 ||
        candidate_index >= static_cast<int>(pose_selection.candidates.size())) {
      continue;
    }
    auto &candidate = pose_selection.candidates[candidate_index];
    if (candidate.end_pose.size() < 3U) {
      continue;
    }
    if (!candidate.feasible) {
      final_validation_result.reason = candidate.rejection_reason;
      continue;
    }
    candidate.was_probed = true;
    parking::ParkingRoiGeometry candidate_roi_geometry = roi_geometry_seed;
    std::string candidate_error;
    if (!parking::ApplyStartGoalParkingRoiTemplate(
            vehicle_xy, vehicle_heading, normalized_slot, candidate.end_pose,
            vehicle_params_, std::max(1.0, start_pose_buffer),
            &candidate_roi_geometry, &candidate_error)) {
      candidate.feasible = false;
      candidate.rejection_reason = candidate_error.empty()
                                       ? "roi start-goal template failed"
                                       : candidate_error;
      final_validation_result.reason = candidate.rejection_reason;
      continue;
    }
    const auto roi_goal_validation =
        roi_validator.Validate(candidate_roi_geometry, vehicle_xy,
                               candidate.end_pose, vehicle_params_);
    if (!roi_goal_validation.valid) {
      candidate.feasible = false;
      candidate.rejection_reason =
          "parking end pose validation failed: " + roi_goal_validation.reason;
      final_validation_result = roi_goal_validation;
      continue;
    }
    candidate.feasible = true;
    candidate.rejection_reason.clear();
    pose_selection.selected_index = candidate_index;
    roi_geometry = std::move(candidate_roi_geometry);
    final_validation_result = roi_goal_validation;
    has_selected_candidate = true;
    break;
  }
  if (!has_selected_candidate) {
    UpdateParkingDebug(frame, roi_geometry_seed, pose_selection,
                       &final_validation_result);
    if (error != nullptr) {
      *error = final_validation_result.reason.empty()
                   ? "no feasible parking end pose candidate"
                   : final_validation_result.reason;
    }
    return false;
  }

  ApplyParkingRoiToOpenSpaceInfo(frame, roi_geometry);
  *roi_parking_boundary = roi_geometry.boundary_segments;

  if (frame->open_space_info().open_space_end_pose().empty()) {
    ApplyParkingEndPose(frame, pose_selection.selected());
  }
  frame->mutable_open_space_info()->set_parking_enforce_final_gear(
      normalized_slot.type != parking::ParkingSlotType::kAngled);
  UpdateParkingDebug(frame, roi_geometry, pose_selection, &final_validation_result);
  return true;
}

bool OpenSpaceRoiDecider::GetParkAndGoBoundary(
    Frame *const frame, const hdmap::Path &nearby_path,
    std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
  const auto &park_and_go_status =
      injector_->planning_context()->planning_status().park_and_go();
  const double adc_init_x = park_and_go_status.adc_init_position().x();
  const double adc_init_y = park_and_go_status.adc_init_position().y();
  const double adc_init_heading = park_and_go_status.adc_init_heading();
  common::math::Vec2d adc_init_position = {adc_init_x, adc_init_y};
  const double adc_length = vehicle_params_.length();
  const double adc_width = vehicle_params_.width();
  // ADC box
  Box2d adc_box(adc_init_position, adc_init_heading, adc_length, adc_width);
  // get vertices from ADC box
  std::vector<common::math::Vec2d> adc_corners;
  adc_box.GetAllCorners(&adc_corners);
  auto left_top = adc_corners[1];
  auto right_top = adc_corners[0];

  const auto &origin_point = frame->open_space_info().origin_point();
  const auto &origin_heading = frame->open_space_info().origin_heading();

  double left_top_s = 0.0;
  double left_top_l = 0.0;
  double right_top_s = 0.0;
  double right_top_l = 0.0;
  if (!(nearby_path.GetProjection(left_top, &left_top_s, &left_top_l) &&
        nearby_path.GetProjection(right_top, &right_top_s, &right_top_l))) {
    AERROR << "fail to get parking spot points' projections on reference line";
    return false;
  }
  left_top -= origin_point;
  left_top.SelfRotate(-origin_heading);
  right_top -= origin_point;
  right_top.SelfRotate(-origin_heading);

  const double center_line_s = (left_top_s + right_top_s) / 2.0;
  std::vector<Vec2d> left_lane_boundary;
  std::vector<Vec2d> right_lane_boundary;
  std::vector<Vec2d> center_lane_boundary_left;
  std::vector<Vec2d> center_lane_boundary_right;
  std::vector<double> center_lane_s_left;
  std::vector<double> center_lane_s_right;
  std::vector<double> left_lane_road_width;
  std::vector<double> right_lane_road_width;

  if (FLAGS_use_road_boundary_from_map) {
    const auto &roi_config = config_.open_space_roi_decider_config();
    const double sampling_start_s =
        center_line_s - roi_config.roi_longitudinal_range_start();
    const double sampling_end_s =
        center_line_s + roi_config.roi_longitudinal_range_end();
    GetRoadBoundaryFromMap(
        nearby_path, center_line_s, sampling_start_s, sampling_end_s,
        origin_point, origin_heading, &left_lane_boundary, &right_lane_boundary,
        &center_lane_boundary_left, &center_lane_boundary_right,
        &center_lane_s_left, &center_lane_s_right, &left_lane_road_width,
        &right_lane_road_width);
  } else {
    GetRoadBoundary(nearby_path, center_line_s, origin_point, origin_heading,
                    &left_lane_boundary, &right_lane_boundary,
                    &center_lane_boundary_left, &center_lane_boundary_right,
                    &center_lane_s_left, &center_lane_s_right,
                    &left_lane_road_width, &right_lane_road_width);
  }

  // Load boundary as line segments in counter-clockwise order
  std::reverse(left_lane_boundary.begin(), left_lane_boundary.end());

  std::vector<Vec2d> boundary_points;
  std::copy(right_lane_boundary.begin(), right_lane_boundary.end(),
            std::back_inserter(boundary_points));
  std::copy(left_lane_boundary.begin(), left_lane_boundary.end(),
            std::back_inserter(boundary_points));

  size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
  for (size_t i = 0; i < right_lane_boundary_last_index; i++) {
    std::vector<Vec2d> segment{right_lane_boundary[i],
                               right_lane_boundary[i + 1]};
    ADEBUG << "right segment";
    ADEBUG << "right_road_boundary: [" << std::setprecision(9)
           << right_lane_boundary[i].x() << ", " << right_lane_boundary[i].y()
           << "]";
    ADEBUG << "right_road_boundary: [" << std::setprecision(9)
           << right_lane_boundary[i + 1].x() << ", "
           << right_lane_boundary[i + 1].y() << "]";
    roi_parking_boundary->push_back(segment);
  }

  size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
  for (size_t i = left_lane_boundary_last_index; i > 0; i--) {
    std::vector<Vec2d> segment{left_lane_boundary[i],
                               left_lane_boundary[i - 1]};
    roi_parking_boundary->push_back(segment);
  }

  ADEBUG << "roi_parking_boundary size: [" << roi_parking_boundary->size()
         << "]";

  // Fuse line segments into convex contraints
  if (!FuseLineSegments(roi_parking_boundary)) {
    return false;
  }

  ADEBUG << "roi_parking_boundary size: [" << roi_parking_boundary->size()
         << "]";
  // Get xy boundary
  auto xminmax = std::minmax_element(
      boundary_points.begin(), boundary_points.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.x() < b.x(); });
  auto yminmax = std::minmax_element(
      boundary_points.begin(), boundary_points.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.y() < b.y(); });
  std::vector<double> ROI_xy_boundary{xminmax.first->x(), xminmax.second->x(),
                                      yminmax.first->y(), yminmax.second->y()};
  auto *xy_boundary =
      frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
  xy_boundary->assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());

  Vec2d vehicle_xy = Vec2d(vehicle_state_.x(), vehicle_state_.y());
  vehicle_xy -= origin_point;
  vehicle_xy.SelfRotate(-origin_heading);
  if (vehicle_xy.x() < ROI_xy_boundary[0] ||
      vehicle_xy.x() > ROI_xy_boundary[1] ||
      vehicle_xy.y() < ROI_xy_boundary[2] ||
      vehicle_xy.y() > ROI_xy_boundary[3]) {
    AERROR << "vehicle outside of xy boundary of parking ROI";
    return false;
  }
  return true;
}

bool OpenSpaceRoiDecider::GetPullOverSpot(
    Frame *const frame, std::array<common::math::Vec2d, 4> *vertices,
    hdmap::Path *nearby_path) {
  const auto &pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  if (!pull_over_status.has_position() ||
      !pull_over_status.position().has_x() ||
      !pull_over_status.position().has_y() || !pull_over_status.has_theta()) {
    AERROR << "Pull over position not set in planning context";
    return false;
  }

  if (frame->reference_line_info().size() > 1) {
    AERROR << "Should not be in pull over when changing lane in open space "
              "planning";
    return false;
  }

  *nearby_path =
      frame->reference_line_info().front().reference_line().GetMapPath();

  // Construct left_top, left_down, right_down, right_top points
  double pull_over_x = pull_over_status.position().x();
  double pull_over_y = pull_over_status.position().y();
  const double pull_over_theta = pull_over_status.theta();
  const double pull_over_length_front = pull_over_status.length_front();
  const double pull_over_length_back = pull_over_status.length_back();
  const double pull_over_width_left = pull_over_status.width_left();
  const double pull_over_width_right = pull_over_status.width_right();

  Vec2d center_shift_vec((pull_over_length_front - pull_over_length_back) * 0.5,
                         (pull_over_width_left - pull_over_width_right) * 0.5);
  center_shift_vec.SelfRotate(pull_over_theta);
  pull_over_x += center_shift_vec.x();
  pull_over_y += center_shift_vec.y();

  const double half_length =
      (pull_over_length_front + pull_over_length_back) / 2.0;
  const double half_width =
      (pull_over_width_left + pull_over_width_right) / 2.0;

  const double cos_heading = std::cos(pull_over_theta);
  const double sin_heading = std::sin(pull_over_theta);

  const double dx1 = cos_heading * half_length;
  const double dy1 = sin_heading * half_length;
  const double dx2 = sin_heading * half_width;
  const double dy2 = -cos_heading * half_width;

  Vec2d left_top(pull_over_x - dx1 + dx2, pull_over_y - dy1 + dy2);
  Vec2d left_down(pull_over_x - dx1 - dx2, pull_over_y - dy1 - dy2);
  Vec2d right_down(pull_over_x + dx1 - dx2, pull_over_y + dy1 - dy2);
  Vec2d right_top(pull_over_x + dx1 + dx2, pull_over_y + dy1 + dy2);

  std::array<Vec2d, 4> pull_over_vertices{left_top, left_down, right_down,
                                          right_top};
  *vertices = std::move(pull_over_vertices);

  return true;
}

bool OpenSpaceRoiDecider::FuseLineSegments(
    std::vector<std::vector<common::math::Vec2d>> *line_segments_vec) {
  static constexpr double kEpsilon = 1.0e-8;
  auto cur_segment = line_segments_vec->begin();
  while (cur_segment != line_segments_vec->end() - 1) {
    auto next_segment = cur_segment + 1;
    auto cur_last_point = cur_segment->back();
    auto next_first_point = next_segment->front();
    // Check if they are the same points
    if (cur_last_point.DistanceTo(next_first_point) > kEpsilon) {
      ++cur_segment;
      continue;
    }
    if (cur_segment->size() < 2 || next_segment->size() < 2) {
      AERROR << "Single point line_segments vec not expected";
      return false;
    }
    size_t cur_segments_size = cur_segment->size();
    auto cur_second_to_last_point = cur_segment->at(cur_segments_size - 2);
    auto next_second_point = next_segment->at(1);
    if (CrossProd(cur_second_to_last_point, cur_last_point, next_second_point) <
        0.0) {
      cur_segment->push_back(next_second_point);
      next_segment->erase(next_segment->begin(), next_segment->begin() + 2);
      if (next_segment->empty()) {
        line_segments_vec->erase(next_segment);
      }
    } else {
      ++cur_segment;
    }
  }
  return true;
}

bool OpenSpaceRoiDecider::FormulateBoundaryConstraints(
    const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
    Frame *const frame) {
  // Gather vertice needed by warm start and distance approach
  if (!LoadObstacleInVertices(roi_parking_boundary, frame)) {
    AERROR << "fail at LoadObstacleInVertices()";
    return false;
  }
  // Validate ROI (connectivity, corridor width, goal feasibility) on the
  // vertex representation before converting to H-representation. This
  // prevents sending infeasible / dead-end ROI to the optimizers.
  if (!ValidateROIOnVertices(frame)) {
    AERROR << "ROI validation failed";
    return false;
  }
  // Transform vertices into the form of Ax>b
  if (!LoadObstacleInHyperPlanes(frame)) {
    AERROR << "fail at LoadObstacleInHyperPlanes()";
    return false;
  }
  return true;
}

bool OpenSpaceRoiDecider::LoadObstacleInVertices(
    const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
    Frame *const frame) {
  auto *mutable_open_space_info = frame->mutable_open_space_info();
  const auto &open_space_info = frame->open_space_info();
  auto *obstacles_vertices_vec =
      mutable_open_space_info->mutable_obstacles_vertices_vec();
  auto *obstacles_edges_num_vec =
      mutable_open_space_info->mutable_obstacles_edges_num();
  obstacles_vertices_vec->clear();

  // load vertices for parking boundary (not need to repeat the first
  // vertice to get close hull)
  size_t parking_boundaries_num = roi_parking_boundary.size();
  size_t perception_obstacles_num = 0;

  for (size_t i = 0; i < parking_boundaries_num; ++i) {
    obstacles_vertices_vec->push_back(roi_parking_boundary[i]);
  }

  Eigen::MatrixXi parking_boundaries_obstacles_edges_num(parking_boundaries_num,
                                                         1);
  for (size_t i = 0; i < parking_boundaries_num; i++) {
    CHECK_GT(roi_parking_boundary[i].size(), 1U);
    parking_boundaries_obstacles_edges_num(i, 0) =
        static_cast<int>(roi_parking_boundary[i].size()) - 1;
  }

  if (config_.open_space_roi_decider_config().enable_perception_obstacles()) {
    if (perception_obstacles_num == 0) {
      ADEBUG << "no obstacle given by perception";
    }

    // load vertices for perception obstacles(repeat the first vertice at the
    // last to form closed convex hull)
    const auto &origin_point = open_space_info.origin_point();
    const auto &origin_heading = open_space_info.origin_heading();
    for (const auto &obstacle : obstacles_by_frame_->Items()) {
      if (FilterOutObstacle(*frame, *obstacle)) {
        continue;
      }
      ++perception_obstacles_num;

      Box2d original_box = obstacle->PerceptionBoundingBox();
      original_box.Shift(-1.0 * origin_point);
      original_box.LongitudinalExtend(
          config_.open_space_roi_decider_config().perception_obstacle_buffer());
      original_box.LateralExtend(
          config_.open_space_roi_decider_config().perception_obstacle_buffer());

      // TODO(Jinyun): Check correctness of ExpandByDistance() in polygon
      // Polygon2d buffered_box(original_box);
      // buffered_box = buffered_box.ExpandByDistance(
      //     config_.open_space_roi_decider_config().perception_obstacle_buffer());
      // TODO(Runxin): Rotate from origin instead
      // original_box.RotateFromCenter(-1.0 * origin_heading);
      std::vector<Vec2d> vertices_ccw = original_box.GetAllCorners();
      std::vector<Vec2d> vertices_cw;
      while (!vertices_ccw.empty()) {
        auto current_corner_pt = vertices_ccw.back();
        current_corner_pt.SelfRotate(-1.0 * origin_heading);
        vertices_cw.push_back(current_corner_pt);
        vertices_ccw.pop_back();
      }
      // As the perception obstacle is a closed convex set, the first vertice
      // is repeated at the end of the vector to help transform all four edges
      // to inequality constraint
      vertices_cw.push_back(vertices_cw.front());
      obstacles_vertices_vec->push_back(vertices_cw);
    }

    // obstacle boundary box is used, thus the edges are set to be 4
    Eigen::MatrixXi perception_obstacles_edges_num =
        4 * Eigen::MatrixXi::Ones(perception_obstacles_num, 1);

    obstacles_edges_num_vec->resize(
        parking_boundaries_obstacles_edges_num.rows() +
            perception_obstacles_edges_num.rows(),
        1);
    *(obstacles_edges_num_vec) << parking_boundaries_obstacles_edges_num,
        perception_obstacles_edges_num;

  } else {
    obstacles_edges_num_vec->resize(
        parking_boundaries_obstacles_edges_num.rows(), 1);
    *(obstacles_edges_num_vec) << parking_boundaries_obstacles_edges_num;
  }

  mutable_open_space_info->set_obstacles_num(parking_boundaries_num +
                                             perception_obstacles_num);
  return true;
}

bool OpenSpaceRoiDecider::ValidateROIOnVertices(Frame *const frame) {
  if (config_.open_space_roi_decider_config().roi_type() !=
      OpenSpaceRoiDeciderConfig::PARKING) {
    return true;
  }

  parking::ParkingRoiGeometry geometry;
  std::string error;
  if (!LoadParkingRoiGeometryFromOpenSpaceInfo(frame->open_space_info(),
                                               &geometry, &error)) {
    AERROR << error;
    return false;
  }

  parking::ParkingRoiValidator validator(
      config_.open_space_roi_decider_config());
  Vec2d vehicle_xy(frame->vehicle_state().x(), frame->vehicle_state().y());
  vehicle_xy -= frame->open_space_info().origin_point();
  vehicle_xy.SelfRotate(-frame->open_space_info().origin_heading());
  const double vehicle_heading =
      common::math::NormalizeAngle(frame->vehicle_state().heading() -
                                   frame->open_space_info().origin_heading());
  const parking::ParkingRoiValidationResult validation_result =
      validator.ValidateGeometryOnly(geometry, vehicle_xy, vehicle_heading,
                                     vehicle_params_);
  UpdateParkingValidationDebug(frame, validation_result);
  if (!validation_result.valid) {
    error = "Invalid parking ROI: " + validation_result.reason;
    AERROR << error;
    return false;
  }
  return true;
}

bool OpenSpaceRoiDecider::FilterOutObstacle(const Frame &frame,
                                            const Obstacle &obstacle) {
  if (obstacle.IsVirtual()) {
    return true;
  }

  const auto &open_space_info = frame.open_space_info();
  const auto &origin_point = open_space_info.origin_point();
  const auto &origin_heading = open_space_info.origin_heading();
  const auto &obstacle_box = obstacle.PerceptionBoundingBox();
  auto obstacle_center_xy = obstacle_box.center();

  // xy_boundary in xmin, xmax, ymin, ymax.
  const auto &roi_xy_boundary = open_space_info.ROI_xy_boundary();
  obstacle_center_xy -= origin_point;
  obstacle_center_xy.SelfRotate(-origin_heading);
  if (obstacle_center_xy.x() < roi_xy_boundary[0] ||
      obstacle_center_xy.x() > roi_xy_boundary[1] ||
      obstacle_center_xy.y() < roi_xy_boundary[2] ||
      obstacle_center_xy.y() > roi_xy_boundary[3]) {
    return true;
  }

  // Translate the end pose back to world frame with endpose in x, y, phi, v
  const auto &end_pose = open_space_info.open_space_end_pose();
  Vec2d end_pose_x_y(end_pose[0], end_pose[1]);
  end_pose_x_y.SelfRotate(origin_heading);
  end_pose_x_y += origin_point;

  // Get vehicle state
  Vec2d vehicle_x_y(vehicle_state_.x(), vehicle_state_.y());

  // Use vehicle position and end position to filter out obstacle
  const double vehicle_center_to_obstacle =
      obstacle_box.DistanceTo(vehicle_x_y);
  const double end_pose_center_to_obstacle =
      obstacle_box.DistanceTo(end_pose_x_y);
  const double filtering_distance =
      config_.open_space_roi_decider_config()
          .perception_obstacle_filtering_distance();
  if (vehicle_center_to_obstacle > filtering_distance &&
      end_pose_center_to_obstacle > filtering_distance) {
    return true;
  }
  return false;
}

bool OpenSpaceRoiDecider::LoadObstacleInHyperPlanes(Frame *const frame) {
  *(frame->mutable_open_space_info()->mutable_obstacles_A()) =
      Eigen::MatrixXd::Zero(
          frame->open_space_info().obstacles_edges_num().sum(), 2);
  *(frame->mutable_open_space_info()->mutable_obstacles_b()) =
      Eigen::MatrixXd::Zero(
          frame->open_space_info().obstacles_edges_num().sum(), 1);
  // vertices using H-representation
  if (!GetHyperPlanes(
          frame->open_space_info().obstacles_num(),
          frame->open_space_info().obstacles_edges_num(),
          frame->open_space_info().obstacles_vertices_vec(),
          frame->mutable_open_space_info()->mutable_obstacles_A(),
          frame->mutable_open_space_info()->mutable_obstacles_b())) {
    AERROR << "Fail to present obstacle in hyperplane";
    return false;
  }
  return true;
}

bool OpenSpaceRoiDecider::GetHyperPlanes(
    const size_t &obstacles_num, const Eigen::MatrixXi &obstacles_edges_num,
    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
    Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all) {
  if (obstacles_num != obstacles_vertices_vec.size()) {
    AERROR << "obstacles_num != obstacles_vertices_vec.size()";
    return false;
  }

  A_all->resize(obstacles_edges_num.sum(), 2);
  b_all->resize(obstacles_edges_num.sum(), 1);

  int counter = 0;
  double kEpsilon = 1.0e-5;
  // start building H representation
  for (size_t i = 0; i < obstacles_num; ++i) {
    size_t current_vertice_num = obstacles_edges_num(i, 0);
    Eigen::MatrixXd A_i(current_vertice_num, 2);
    Eigen::MatrixXd b_i(current_vertice_num, 1);

    // take two subsequent vertices, and computer hyperplane
    for (size_t j = 0; j < current_vertice_num; ++j) {
      Vec2d v1 = obstacles_vertices_vec[i][j];
      Vec2d v2 = obstacles_vertices_vec[i][j + 1];

      Eigen::MatrixXd A_tmp(2, 1), b_tmp(1, 1), ab(2, 1);
      // find hyperplane passing through v1 and v2
      if (std::abs(v1.x() - v2.x()) < kEpsilon) {
        if (v2.y() < v1.y()) {
          A_tmp << 1, 0;
          b_tmp << v1.x();
        } else {
          A_tmp << -1, 0;
          b_tmp << -v1.x();
        }
      } else if (std::abs(v1.y() - v2.y()) < kEpsilon) {
        if (v1.x() < v2.x()) {
          A_tmp << 0, 1;
          b_tmp << v1.y();
        } else {
          A_tmp << 0, -1;
          b_tmp << -v1.y();
        }
      } else {
        Eigen::MatrixXd tmp1(2, 2);
        tmp1 << v1.x(), 1, v2.x(), 1;
        Eigen::MatrixXd tmp2(2, 1);
        tmp2 << v1.y(), v2.y();
        ab = tmp1.inverse() * tmp2;
        double a = ab(0, 0);
        double b = ab(1, 0);

        if (v1.x() < v2.x()) {
          A_tmp << -a, 1;
          b_tmp << b;
        } else {
          A_tmp << a, -1;
          b_tmp << -b;
        }
      }

      // store vertices
      A_i.block(j, 0, 1, 2) = A_tmp.transpose();
      b_i.block(j, 0, 1, 1) = b_tmp;
    }

    A_all->block(counter, 0, A_i.rows(), 2) = A_i;
    b_all->block(counter, 0, b_i.rows(), 1) = b_i;
    counter += static_cast<int>(current_vertice_num);
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
