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

namespace apollo {
namespace common {

Status VehicleStateProvider::Update(
    const localization::LocalizationEstimate& localization,
    const canbus::Chassis& chassis) {
  const VehicleState previous_vehicle_state = vehicle_state_;
  const VehicleOperatingState previous_operating_state = operating_state_;
  const auto previous_original_localization = original_localization_;
  original_localization_ = localization;
  if (!ConstructExceptLinearVelocity(localization)) {
    vehicle_state_ = previous_vehicle_state;
    operating_state_ = previous_operating_state;
    original_localization_ = previous_original_localization;
    std::string msg = absl::StrCat(
        "Fail to update because ConstructExceptLinearVelocity error.",
        "localization:\n", localization.DebugString());
    return Status(ErrorCode::LOCALIZATION_ERROR, msg);
  }
  if (localization.has_measurement_time()) {
    vehicle_state_.set_timestamp(localization.measurement_time());
  } else if (localization.header().has_timestamp_sec()) {
    vehicle_state_.set_timestamp(localization.header().timestamp_sec());
  } else if (chassis.has_header() && chassis.header().has_timestamp_sec()) {
    AERROR << "Unable to use location timestamp for vehicle state. Use chassis "
              "time instead.";
    vehicle_state_.set_timestamp(chassis.header().timestamp_sec());
  }

  if (chassis.has_gear_location()) {
    vehicle_state_.set_gear(chassis.gear_location());
    operating_state_.set_gear(chassis.gear_location());
  } else {
    vehicle_state_.set_gear(canbus::Chassis::GEAR_NONE);
    operating_state_.set_gear(canbus::Chassis::GEAR_NONE);
  }
  operating_state_.set_travel_direction(
      operating_state_.gear() == canbus::Chassis::GEAR_REVERSE
          ? TRAVEL_DIRECTION_REVERSE
      : (operating_state_.gear() == canbus::Chassis::GEAR_DRIVE ||
         operating_state_.gear() == canbus::Chassis::GEAR_LOW)
          ? TRAVEL_DIRECTION_FORWARD
          : TRAVEL_DIRECTION_UNKNOWN);
  vehicle_state_.set_reference_point(REAR_AXLE_CENTER);
  operating_state_.set_timestamp(vehicle_state_.timestamp());

  if (chassis.has_speed_mps()) {
    vehicle_state_.set_linear_velocity(chassis.speed_mps());
    if (!FLAGS_reverse_heading_vehicle_state &&
        operating_state_.gear() == canbus::Chassis::GEAR_REVERSE) {
      vehicle_state_.set_linear_velocity(-vehicle_state_.linear_velocity());
    }
  }

  if (chassis.has_steering_percentage()) {
    vehicle_state_.set_steering_percentage(chassis.steering_percentage());
    operating_state_.set_steering_percentage(chassis.steering_percentage());
  }

  static constexpr double kEpsilon = 0.1;
  if (std::abs(vehicle_state_.linear_velocity()) < kEpsilon) {
    vehicle_state_.set_kappa(0.0);
  } else {
    vehicle_state_.set_kappa(vehicle_state_.angular_velocity() /
                             vehicle_state_.linear_velocity());
  }

  vehicle_state_.set_driving_mode(chassis.driving_mode());
  operating_state_.set_driving_mode(chassis.driving_mode());

  return Status::OK();
}

bool VehicleStateProvider::ConstructExceptLinearVelocity(
    const localization::LocalizationEstimate& localization) {
  if (!localization.has_pose()) {
    AERROR << "Invalid localization input.";
    return false;
  }

  // skip localization update when it is in use_navigation_mode.
  if (FLAGS_use_navigation_mode) {
    ADEBUG << "Skip localization update when it is in use_navigation_mode.";
    return true;
  }

  vehicle_state_.mutable_pose()->CopyFrom(localization.pose());
  if (localization.pose().has_position()) {
    vehicle_state_.set_x(localization.pose().position().x());
    vehicle_state_.set_y(localization.pose().position().y());
    vehicle_state_.set_z(localization.pose().position().z());
  }

  const auto& orientation = localization.pose().orientation();

  if (localization.pose().has_heading()) {
    vehicle_state_.set_heading(localization.pose().heading());
  } else {
    vehicle_state_.set_heading(
        math::QuaternionToHeading(orientation.qw(), orientation.qx(),
                                  orientation.qy(), orientation.qz()));
  }

  if (FLAGS_enable_map_reference_unify) {
    if (!localization.pose().has_angular_velocity_vrf()) {
      AERROR << "localization.pose().has_angular_velocity_vrf() must be true "
                "when FLAGS_enable_map_reference_unify is true.";
      return false;
    }
    vehicle_state_.set_angular_velocity(
        localization.pose().angular_velocity_vrf().z());

    if (!localization.pose().has_linear_acceleration_vrf()) {
      AERROR << "localization.pose().has_linear_acceleration_vrf() must be "
                "true when FLAGS_enable_map_reference_unify is true.";
      return false;
    }
    vehicle_state_.set_linear_acceleration(
        localization.pose().linear_acceleration_vrf().y());
  } else {
    if (!localization.pose().has_angular_velocity()) {
      AERROR << "localization.pose() has no angular velocity.";
      return false;
    }
    vehicle_state_.set_angular_velocity(
        localization.pose().angular_velocity().z());

    if (!localization.pose().has_linear_acceleration()) {
      AERROR << "localization.pose() has no linear acceleration.";
      return false;
    }
    vehicle_state_.set_linear_acceleration(
        localization.pose().linear_acceleration().y());
  }

  if (localization.pose().has_euler_angles()) {
    vehicle_state_.set_roll(localization.pose().euler_angles().y());
    vehicle_state_.set_pitch(localization.pose().euler_angles().x());
    vehicle_state_.set_yaw(localization.pose().euler_angles().z());
  } else {
    math::EulerAnglesZXYd euler_angle(orientation.qw(), orientation.qx(),
                                      orientation.qy(), orientation.qz());
    vehicle_state_.set_roll(euler_angle.roll());
    vehicle_state_.set_pitch(euler_angle.pitch());
    vehicle_state_.set_yaw(euler_angle.yaw());
  }

  return true;
}

double VehicleStateProvider::x() const { return vehicle_state_.x(); }

double VehicleStateProvider::y() const { return vehicle_state_.y(); }

double VehicleStateProvider::z() const { return vehicle_state_.z(); }

double VehicleStateProvider::roll() const { return vehicle_state_.roll(); }

double VehicleStateProvider::pitch() const { return vehicle_state_.pitch(); }

double VehicleStateProvider::yaw() const { return vehicle_state_.yaw(); }

double VehicleStateProvider::heading() const {
  return vehicle_state_.heading();
}

double VehicleStateProvider::kappa() const { return vehicle_state_.kappa(); }

double VehicleStateProvider::linear_velocity() const {
  return vehicle_state_.linear_velocity();
}

double VehicleStateProvider::angular_velocity() const {
  return vehicle_state_.angular_velocity();
}

double VehicleStateProvider::linear_acceleration() const {
  return vehicle_state_.linear_acceleration();
}

canbus::Chassis::GearPosition VehicleStateProvider::gear() const {
  return operating_state_.gear();
}

double VehicleStateProvider::steering_percentage() const {
  return vehicle_state_.steering_percentage();
}

double VehicleStateProvider::timestamp() const {
  return vehicle_state_.timestamp();
}

const localization::Pose& VehicleStateProvider::pose() const {
  return vehicle_state_.pose();
}

const localization::Pose& VehicleStateProvider::original_pose() const {
  return original_localization_.pose();
}

void VehicleStateProvider::set_linear_velocity(const double linear_velocity) {
  vehicle_state_.set_linear_velocity(linear_velocity);
}

const VehicleState& VehicleStateProvider::canonical_state() const {
  return vehicle_state_;
}

const VehicleOperatingState& VehicleStateProvider::operating_state() const {
  return operating_state_;
}

}  // namespace common
}  // namespace apollo
