/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/vehicle_state/vehicle_state_provider.h"

#include <cmath>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/vehicle_state/reference_point_transformer.h"
#include "modules/common/vehicle_state/vehicle_description.h"

namespace apollo {
namespace common {
namespace {

Status GetConfiguredReferencePoint(ReferencePoint* reference_point) {
  if (reference_point == nullptr) {
    return Status(ErrorCode::LOCALIZATION_ERROR,
                  "reference point output is null");
  }
  switch (FLAGS_vehicle_state_reference_point) {
    case REAR_AXLE_CENTER:
    case FRONT_AXLE_CENTER:
    case CENTER_OF_MASS:
      *reference_point =
          static_cast<ReferencePoint>(FLAGS_vehicle_state_reference_point);
      return Status::OK();
    default:
      return Status(ErrorCode::LOCALIZATION_ERROR,
                    absl::StrCat("Unsupported vehicle state reference point: ",
                                 FLAGS_vehicle_state_reference_point));
  }
}

}  // namespace

Status VehicleStateProvider::Update(
    const localization::LocalizationEstimate& localization,
    const canbus::Chassis& chassis) {
  VehicleMotionState motion_state = motion_state_snapshot_;
  VehicleOperatingState operating_state;
  VehicleState next_state;
  if (!ConstructMotionState(localization, &motion_state)) {
    return Status(
        ErrorCode::LOCALIZATION_ERROR,
        absl::StrCat("Failed to construct motion state. localization:\n",
                     localization.DebugString()));
  }
  double timestamp = 0.0;
  if (localization.has_measurement_time()) {
    timestamp = localization.measurement_time();
  } else if (localization.header().has_timestamp_sec()) {
    timestamp = localization.header().timestamp_sec();
  } else if (chassis.has_header() && chassis.header().has_timestamp_sec()) {
    AERROR << "Unable to use location timestamp for vehicle state. Use chassis "
              "time instead.";
    timestamp = chassis.header().timestamp_sec();
  }
  motion_state.set_timestamp(timestamp);
  if (!ConstructOperatingState(chassis, timestamp, &operating_state)) {
    return Status(ErrorCode::LOCALIZATION_ERROR,
                  "Failed to construct operating state.");
  }
  if (chassis.has_speed_mps()) {
    motion_state.set_linear_velocity(chassis.speed_mps());
    if (!FLAGS_reverse_heading_vehicle_state &&
        operating_state.gear() == canbus::Chassis::GEAR_REVERSE) {
      motion_state.set_linear_velocity(-motion_state.linear_velocity());
    }
  }
  static constexpr double kEpsilon = 0.1;
  if (std::abs(motion_state.linear_velocity()) < kEpsilon) {
    motion_state.set_kappa(0.0);
  } else {
    motion_state.set_kappa(motion_state.angular_velocity() /
                           motion_state.linear_velocity());
  }
  ReferencePoint configured_reference_point;
  const auto reference_point_status =
      GetConfiguredReferencePoint(&configured_reference_point);
  if (!reference_point_status.ok()) {
    return reference_point_status;
  }
  if (configured_reference_point != REAR_AXLE_CENTER) {
    VehicleState source_state;
    source_state.set_x(motion_state.x());
    source_state.set_y(motion_state.y());
    source_state.set_z(motion_state.z());
    source_state.set_heading(motion_state.heading());
    source_state.set_linear_velocity(motion_state.linear_velocity());
    source_state.set_lateral_velocity(motion_state.lateral_velocity());
    source_state.set_angular_velocity(motion_state.angular_velocity());
    source_state.set_linear_acceleration(motion_state.linear_acceleration());
    source_state.set_kappa(motion_state.kappa());
    source_state.set_timestamp(motion_state.timestamp());
    source_state.set_reference_point(REAR_AXLE_CENTER);
    if (motion_state.has_pose()) {
      source_state.mutable_pose()->CopyFrom(motion_state.pose());
    }
    VehicleState transformed_state;
    ReferencePointTransformer transformer;
    const auto transform_status = transformer.TransformState(
        source_state, configured_reference_point, &transformed_state);
    if (!transform_status.ok()) {
      return transform_status;
    }
    motion_state.set_x(transformed_state.x());
    motion_state.set_y(transformed_state.y());
    motion_state.set_z(transformed_state.z());
    motion_state.set_heading(transformed_state.heading());
    motion_state.set_linear_velocity(transformed_state.linear_velocity());
    motion_state.set_lateral_velocity(transformed_state.lateral_velocity());
    motion_state.set_angular_velocity(transformed_state.angular_velocity());
    motion_state.set_linear_acceleration(
        transformed_state.linear_acceleration());
    motion_state.set_kappa(transformed_state.kappa());
    if (transformed_state.has_pose()) {
      motion_state.mutable_pose()->CopyFrom(transformed_state.pose());
    }
    motion_state.set_reference_point(configured_reference_point);
  }
  if (!MergeStates(motion_state, operating_state, &next_state)) {
    return Status(ErrorCode::LOCALIZATION_ERROR,
                  "Failed to merge vehicle state.");
  }
  motion_state_snapshot_ = motion_state;
  operating_state_snapshot_ = operating_state;
  state_snapshot_ = next_state;
  return Status::OK();
}

bool VehicleStateProvider::ConstructMotionState(
    const localization::LocalizationEstimate& localization,
    VehicleMotionState* motion_state) const {
  if (motion_state == nullptr || !localization.has_pose()) {
    AERROR << "Invalid localization input.";
    return false;
  }

  motion_state->mutable_pose()->CopyFrom(localization.pose());
  if (localization.pose().has_position()) {
    motion_state->set_x(localization.pose().position().x());
    motion_state->set_y(localization.pose().position().y());
    motion_state->set_z(localization.pose().position().z());
  }

  const auto& orientation = localization.pose().orientation();

  if (localization.pose().has_heading()) {
    motion_state->set_heading(localization.pose().heading());
  } else {
    motion_state->set_heading(
        math::QuaternionToHeading(orientation.qw(), orientation.qx(),
                                  orientation.qy(), orientation.qz()));
  }

  if (localization.pose().has_linear_velocity()) {
    const double heading = motion_state->heading();
    const auto& velocity = localization.pose().linear_velocity();
    motion_state->set_lateral_velocity(-std::sin(heading) * velocity.x() +
                                       std::cos(heading) * velocity.y());
  } else {
    motion_state->set_lateral_velocity(0.0);
  }

  if (FLAGS_enable_map_reference_unify) {
    if (!localization.pose().has_angular_velocity_vrf()) {
      AERROR << "localization.pose().has_angular_velocity_vrf() must be true "
                "when FLAGS_enable_map_reference_unify is true.";
      return false;
    }
    motion_state->set_angular_velocity(
        localization.pose().angular_velocity_vrf().z());

    if (!localization.pose().has_linear_acceleration_vrf()) {
      AERROR << "localization.pose().has_linear_acceleration_vrf() must be "
                "true when FLAGS_enable_map_reference_unify is true.";
      return false;
    }
    motion_state->set_linear_acceleration(
        localization.pose().linear_acceleration_vrf().y());
  } else {
    if (!localization.pose().has_angular_velocity()) {
      AERROR << "localization.pose() has no angular velocity.";
      return false;
    }
    motion_state->set_angular_velocity(
        localization.pose().angular_velocity().z());

    if (!localization.pose().has_linear_acceleration()) {
      AERROR << "localization.pose() has no linear acceleration.";
      return false;
    }
    motion_state->set_linear_acceleration(
        localization.pose().linear_acceleration().y());
  }

  if (localization.pose().has_euler_angles()) {
    motion_state->set_roll(localization.pose().euler_angles().y());
    motion_state->set_pitch(localization.pose().euler_angles().x());
    motion_state->set_yaw(localization.pose().euler_angles().z());
  } else {
    math::EulerAnglesZXYd euler_angle(orientation.qw(), orientation.qx(),
                                      orientation.qy(), orientation.qz());
    motion_state->set_roll(euler_angle.roll());
    motion_state->set_pitch(euler_angle.pitch());
    motion_state->set_yaw(euler_angle.yaw());
  }

  return true;
}

bool VehicleStateProvider::ConstructOperatingState(
    const canbus::Chassis& chassis, const double timestamp,
    VehicleOperatingState* operating_state) const {
  if (operating_state == nullptr) {
    return false;
  }
  operating_state->set_timestamp(timestamp);
  const auto gear = chassis.has_gear_location() ? chassis.gear_location()
                                                : canbus::Chassis::GEAR_NONE;
  operating_state->set_gear(gear);
  operating_state->set_travel_direction(
      gear == canbus::Chassis::GEAR_REVERSE ? TRAVEL_DIRECTION_REVERSE
      : (gear == canbus::Chassis::GEAR_DRIVE ||
         gear == canbus::Chassis::GEAR_LOW)
          ? TRAVEL_DIRECTION_FORWARD
          : TRAVEL_DIRECTION_UNKNOWN);
  if (chassis.has_steering_percentage()) {
    operating_state->set_steering_percentage(chassis.steering_percentage());
  }
  operating_state->set_driving_mode(chassis.driving_mode());
  return true;
}

bool VehicleStateProvider::MergeStates(
    const VehicleMotionState& motion_state,
    const VehicleOperatingState& operating_state, VehicleState* state) const {
  if (state == nullptr) {
    return false;
  }
  state->Clear();
  state->set_x(motion_state.x());
  state->set_y(motion_state.y());
  state->set_z(motion_state.z());
  state->set_timestamp(motion_state.timestamp());
  state->set_roll(motion_state.roll());
  state->set_pitch(motion_state.pitch());
  state->set_yaw(motion_state.yaw());
  state->set_heading(motion_state.heading());
  state->set_kappa(motion_state.kappa());
  state->set_linear_velocity(motion_state.linear_velocity());
  state->set_lateral_velocity(motion_state.lateral_velocity());
  state->set_angular_velocity(motion_state.angular_velocity());
  state->set_linear_acceleration(motion_state.linear_acceleration());
  state->set_reference_point(motion_state.reference_point());
  if (motion_state.has_pose()) {
    state->mutable_pose()->CopyFrom(motion_state.pose());
  }
  state->set_gear(operating_state.gear());
  state->set_driving_mode(operating_state.driving_mode());
  state->set_travel_direction(operating_state.travel_direction());
  state->set_steering_percentage(operating_state.steering_percentage());
  return true;
}

const VehicleState& VehicleStateProvider::state() const {
  return state_snapshot_;
}

}  // namespace common
}  // namespace apollo
