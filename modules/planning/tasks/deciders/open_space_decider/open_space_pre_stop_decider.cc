/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/open_space_decider/open_space_pre_stop_decider.h"

#include <memory>
#include <string>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"

namespace apollo {
namespace planning {

namespace {

bool GetParkingSpotCenterFromRouting(
    const Frame& frame, apollo::common::math::Vec2d* parking_spot_center) {
  const auto& routing_request = frame.local_view().routing->routing_request();
  if (!routing_request.has_parking_info()) {
    return false;
  }
  const auto& corner_point = routing_request.parking_info().corner_point();
  if (corner_point.point_size() <= 0) {
    return false;
  }
  double center_x = 0.0;
  double center_y = 0.0;
  for (const auto& point : corner_point.point()) {
    center_x += point.x();
    center_y += point.y();
  }
  center_x /= static_cast<double>(corner_point.point_size());
  center_y /= static_cast<double>(corner_point.point_size());
  parking_spot_center->set_x(center_x);
  parking_spot_center->set_y(center_y);
  return true;
}

apollo::common::math::Vec2d GetParkingSpotCenterFromMap(
    const apollo::hdmap::ParkingSpaceInfoConstPtr& parking_spot_ptr) {
  const auto& points = parking_spot_ptr->polygon().points();
  double center_x = 0.0;
  double center_y = 0.0;
  for (const auto& point : points) {
    center_x += point.x();
    center_y += point.y();
  }
  center_x /= static_cast<double>(points.size());
  center_y /= static_cast<double>(points.size());
  return apollo::common::math::Vec2d(center_x, center_y);
}

}  // namespace

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;

OpenSpacePreStopDecider::OpenSpacePreStopDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  ACHECK(config.has_open_space_pre_stop_decider_config());
}

Status OpenSpacePreStopDecider::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  open_space_pre_stop_decider_config_ =
      config_.open_space_pre_stop_decider_config();
  double target_s = 0.0;
  const auto& stop_type = open_space_pre_stop_decider_config_.stop_type();
  switch (stop_type) {
    case OpenSpacePreStopDeciderConfig::PARKING:
      if (!CheckParkingSpotPreStop(frame, reference_line_info, &target_s)) {
        const std::string msg = "Checking parking spot pre stop fails";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      SetParkingSpotStopFence(target_s, frame, reference_line_info);
      break;
    case OpenSpacePreStopDeciderConfig::PULL_OVER:
      if (!CheckPullOverPreStop(frame, reference_line_info, &target_s)) {
        const std::string msg = "Checking pull over pre stop fails";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      SetPullOverStopFence(target_s, frame, reference_line_info);
      break;
    default:
      const std::string msg = "This stop type not implemented";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

bool OpenSpacePreStopDecider::CheckPullOverPreStop(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    double* target_s) {
  *target_s = 0.0;
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  if (pull_over_status.has_position() && pull_over_status.position().has_x() &&
      pull_over_status.position().has_y()) {
    common::SLPoint pull_over_sl;
    const auto& reference_line = reference_line_info->reference_line();
    reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
    *target_s = pull_over_sl.s();
  }
  return true;
}

bool OpenSpacePreStopDecider::CheckParkingSpotPreStop(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    double* target_s) {
  const auto& target_parking_spot_id =
      frame->open_space_info().target_parking_spot_id();
  if (target_parking_spot_id.empty()) {
    AERROR << "no target parking spot id found when setting pre stop fence";
    return false;
  }

  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap);
  hdmap::Id id;
  id.set_id(target_parking_spot_id);
  ParkingSpaceInfoConstPtr target_parking_spot_ptr =
      hdmap->GetParkingSpaceById(id);
  if (target_parking_spot_ptr == nullptr) {
    AERROR << "no target parking spot found in HDMap";
    return false;
  }
  if (target_parking_spot_ptr->polygon().points().empty()) {
    AERROR << "target parking spot polygon is empty";
    return false;
  }

  Vec2d parking_spot_center;
  if (!GetParkingSpotCenterFromRouting(*frame, &parking_spot_center)) {
    parking_spot_center = GetParkingSpotCenterFromMap(target_parking_spot_ptr);
  }

  common::PointENU center_point;
  center_point.set_x(parking_spot_center.x());
  center_point.set_y(parking_spot_center.y());
  common::SLPoint center_sl;
  reference_line_info->reference_line().XYToSL(center_point, &center_sl);
  *target_s = center_sl.s();
  return true;
}

void OpenSpacePreStopDecider::SetParkingSpotStopFence(
    const double target_s, Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  const auto& nearby_path = reference_line_info->reference_line().map_path();
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  const VehicleState& vehicle_state = frame->vehicle_state();
  double stop_line_s = 0.0;
  double stop_distance_to_target =
      open_space_pre_stop_decider_config_.stop_distance_to_target();
  double static_linear_velocity_epsilon = 1.0e-2;
  CHECK_GE(stop_distance_to_target, 1.0e-8);
  double target_vehicle_offset = target_s - adc_front_edge_s;
  if (target_vehicle_offset > stop_distance_to_target) {
    stop_line_s = target_s - stop_distance_to_target;
  } else if (std::abs(target_vehicle_offset) < stop_distance_to_target) {
    stop_line_s = target_s + stop_distance_to_target;
  } else if (target_vehicle_offset < -stop_distance_to_target) {
    if (!frame->open_space_info().pre_stop_rightaway_flag()) {
      // TODO(Jinyun) Use constant comfortable deacceleration rather than
      // distance by config to set stop fence
      stop_line_s =
          adc_front_edge_s +
          open_space_pre_stop_decider_config_.rightaway_stop_distance();
      if (std::abs(vehicle_state.linear_velocity()) <
          static_linear_velocity_epsilon) {
        stop_line_s = adc_front_edge_s;
      }
      *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) =
          nearby_path.GetSmoothPoint(stop_line_s);
      frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(true);
    } else {
      double stop_point_s = 0.0;
      double stop_point_l = 0.0;
      nearby_path.GetNearestPoint(
          frame->open_space_info().pre_stop_rightaway_point(), &stop_point_s,
          &stop_point_l);
      stop_line_s = stop_point_s;
    }
  }

  const std::string stop_wall_id = OPEN_SPACE_STOP_ID;
  std::vector<std::string> wait_for_obstacles;
  frame->mutable_open_space_info()->set_open_space_pre_stop_fence_s(
      stop_line_s);
  util::BuildStopDecision(stop_wall_id, stop_line_s, 0.0,
                          StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                          wait_for_obstacles, "OpenSpacePreStopDecider", frame,
                          reference_line_info);
}

void OpenSpacePreStopDecider::SetPullOverStopFence(
    const double target_s, Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  const auto& nearby_path = reference_line_info->reference_line().map_path();
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  const VehicleState& vehicle_state = frame->vehicle_state();
  double stop_line_s = 0.0;
  double stop_distance_to_target =
      open_space_pre_stop_decider_config_.stop_distance_to_target();
  double static_linear_velocity_epsilon = 1.0e-2;
  CHECK_GE(stop_distance_to_target, 1.0e-8);
  double target_vehicle_offset = target_s - adc_front_edge_s;
  if (target_vehicle_offset > stop_distance_to_target) {
    stop_line_s = target_s - stop_distance_to_target;
  } else {
    if (!frame->open_space_info().pre_stop_rightaway_flag()) {
      // TODO(Jinyun) Use constant comfortable deacceleration rather than
      // distance by config to set stop fence
      stop_line_s =
          adc_front_edge_s +
          open_space_pre_stop_decider_config_.rightaway_stop_distance();
      if (std::abs(vehicle_state.linear_velocity()) <
          static_linear_velocity_epsilon) {
        stop_line_s = adc_front_edge_s;
      }
      *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) =
          nearby_path.GetSmoothPoint(stop_line_s);
      frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(true);
    } else {
      double stop_point_s = 0.0;
      double stop_point_l = 0.0;
      nearby_path.GetNearestPoint(
          frame->open_space_info().pre_stop_rightaway_point(), &stop_point_s,
          &stop_point_l);
      stop_line_s = stop_point_s;
    }
  }

  const std::string stop_wall_id = OPEN_SPACE_STOP_ID;
  std::vector<std::string> wait_for_obstacles;
  frame->mutable_open_space_info()->set_open_space_pre_stop_fence_s(
      stop_line_s);
  util::BuildStopDecision(stop_wall_id, stop_line_s, 0.0,
                          StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                          wait_for_obstacles, "OpenSpacePreStopDecider", frame,
                          reference_line_info);
}
}  // namespace planning
}  // namespace apollo
