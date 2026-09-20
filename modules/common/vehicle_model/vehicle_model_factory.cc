// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "modules/common/vehicle_model/vehicle_model_factory.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "absl/strings/str_cat.h"

#include "modules/common/math/math_utils.h"

namespace apollo {
namespace common {
namespace {

constexpr double kTimeTolerance = 1.0e-9;
constexpr double kLowSpeedVelocityTolerance = 1.0e-9;
constexpr double kRearSteeringTolerance = 1.0e-9;
constexpr double kMaxPredictionHorizon = 60.0;
constexpr double kHalfPi = 1.5707963267948966;

struct KinematicMotion {
  double curvature = 0.0;
  double lateral_velocity_ratio = 0.0;
  double longitudinal_acceleration = 0.0;
};

Status ValidatePrediction(const double horizon, const double dt,
                          const VehicleReferencePoint canonical_point,
                          const VehicleState& state,
                          VehicleState* predicted_state) {
  if (predicted_state == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "predicted canonical state is null");
  }
  if (!std::isfinite(horizon) || horizon < 0.0 ||
      horizon > kMaxPredictionHorizon) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "prediction horizon must be finite, nonnegative, and at "
                  "most 60 seconds");
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "vehicle model time step must be finite and positive");
  }
  if (state.reference_point() != canonical_point) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        absl::StrCat("vehicle model expected canonical reference point ",
                     canonical_point, " but received ",
                     state.reference_point()));
  }
  if (!std::isfinite(state.x()) || !std::isfinite(state.y()) ||
      !std::isfinite(state.heading()) ||
      !std::isfinite(state.linear_velocity()) ||
      !std::isfinite(state.lateral_velocity())) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "vehicle model state contains a non-finite value");
  }
  return Status::OK();
}

Status Propagate(const double horizon, const double dt,
                 const VehicleReferencePoint canonical_point,
                 const VehicleState& state, const KinematicMotion& motion,
                 VehicleState* predicted_state) {
  auto status =
      ValidatePrediction(horizon, dt, canonical_point, state, predicted_state);
  if (!status.ok()) {
    return status;
  }
  if (!std::isfinite(motion.curvature) ||
      !std::isfinite(motion.lateral_velocity_ratio) ||
      !std::isfinite(motion.longitudinal_acceleration)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "vehicle model input contains a non-finite value");
  }

  *predicted_state = state;
  double remaining_time = horizon;
  double x = state.x();
  double y = state.y();
  double heading = state.heading();
  double velocity = state.linear_velocity();

  while (remaining_time > kTimeTolerance) {
    const double step = std::min(dt, remaining_time);
    const double next_velocity =
        velocity + motion.longitudinal_acceleration * step;
    const double distance = 0.5 * (velocity + next_velocity) * step;
    const double midpoint_heading = heading + 0.5 * distance * motion.curvature;
    x +=
        distance * (std::cos(midpoint_heading) -
                    motion.lateral_velocity_ratio * std::sin(midpoint_heading));
    y +=
        distance * (std::sin(midpoint_heading) +
                    motion.lateral_velocity_ratio * std::cos(midpoint_heading));
    heading += distance * motion.curvature;
    velocity = next_velocity;
    remaining_time -= step;
  }

  heading = math::NormalizeAngle(heading);
  predicted_state->set_x(x);
  predicted_state->set_y(y);
  predicted_state->set_heading(heading);
  predicted_state->set_kappa(motion.curvature);
  predicted_state->set_linear_velocity(velocity);
  predicted_state->set_lateral_velocity(velocity *
                                        motion.lateral_velocity_ratio);
  predicted_state->set_angular_velocity(velocity * motion.curvature);
  predicted_state->set_linear_acceleration(motion.longitudinal_acceleration);
  predicted_state->set_reference_point(canonical_point);
  if (state.has_timestamp()) {
    predicted_state->set_timestamp(state.timestamp() + horizon);
  }
  if (predicted_state->has_pose()) {
    auto* pose = predicted_state->mutable_pose();
    pose->mutable_position()->set_x(x);
    pose->mutable_position()->set_y(y);
    pose->set_heading(heading);
  }
  return Status::OK();
}

Status ValidateSteeringAngle(const double angle, const double limit,
                             const char* name) {
  if (!std::isfinite(angle) || std::abs(angle) > limit) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        absl::StrCat(name, " exceeds the configured road-wheel limit"));
  }
  return Status::OK();
}

Status ValidateAcceleration(const double acceleration,
                            const double max_acceleration,
                            const double max_deceleration) {
  if (!std::isfinite(acceleration) || acceleration > max_acceleration ||
      acceleration < max_deceleration) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        "longitudinal acceleration exceeds configured vehicle limits");
  }
  return Status::OK();
}

KinematicMotion HeldCurvatureMotion(const VehicleState& state) {
  KinematicMotion motion;
  motion.curvature = state.kappa();
  // At near-zero longitudinal speed, the lateral velocity ratio is
  // undefined; the held-curvature fallback intentionally discards it.
  if (std::abs(state.linear_velocity()) > kLowSpeedVelocityTolerance) {
    motion.lateral_velocity_ratio =
        state.lateral_velocity() / state.linear_velocity();
  }
  motion.longitudinal_acceleration = state.linear_acceleration();
  return motion;
}

}  // namespace

AckermannKinematicModel::AckermannKinematicModel(
    const double dt, const double wheel_base, const double max_road_wheel_angle,
    const double max_acceleration, const double max_deceleration)
    : dt_(dt),
      wheel_base_(wheel_base),
      max_road_wheel_angle_(max_road_wheel_angle),
      max_acceleration_(max_acceleration),
      max_deceleration_(max_deceleration) {
}

Status AckermannKinematicModel::Predict(
    const double horizon, const VehicleState& canonical_state,
    const VehicleModelInput& input,
    VehicleState* predicted_canonical_state) const {
  auto status =
      ValidateSteeringAngle(input.front_steering_angle, max_road_wheel_angle_,
                            "front steering angle");
  if (!status.ok()) {
    return status;
  }
  if (!std::isfinite(input.rear_steering_angle)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Ackermann rear steering angle must be finite");
  }
  if (std::abs(input.rear_steering_angle) > kRearSteeringTolerance) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Ackermann model does not accept rear steering");
  }
  status = ValidateAcceleration(input.longitudinal_acceleration,
                                max_acceleration_, max_deceleration_);
  if (!status.ok()) {
    return status;
  }
  KinematicMotion motion;
  motion.curvature = std::tan(input.front_steering_angle) / wheel_base_;
  motion.longitudinal_acceleration = input.longitudinal_acceleration;
  return Propagate(horizon, dt_, canonical_reference_point(), canonical_state,
                   motion, predicted_canonical_state);
}

Status AckermannKinematicModel::PredictWithHeldCurvature(
    const double horizon, const VehicleState& canonical_state,
    VehicleState* predicted_canonical_state) const {
  return Propagate(horizon, dt_, canonical_reference_point(), canonical_state,
                   HeldCurvatureMotion(canonical_state),
                   predicted_canonical_state);
}

FourWheelSteeringKinematicModel::FourWheelSteeringKinematicModel(
    const double dt, const double wheel_base,
    const double center_of_mass_offset, const double max_road_wheel_angle,
    const double max_acceleration, const double max_deceleration)
    : dt_(dt),
      wheel_base_(wheel_base),
      distance_cg_to_rear_axle_(center_of_mass_offset),
      distance_cg_to_front_axle_(wheel_base - center_of_mass_offset),
      max_road_wheel_angle_(max_road_wheel_angle),
      max_acceleration_(max_acceleration),
      max_deceleration_(max_deceleration) {
}

Status FourWheelSteeringKinematicModel::Predict(
    const double horizon, const VehicleState& canonical_state,
    const VehicleModelInput& input,
    VehicleState* predicted_canonical_state) const {
  auto status =
      ValidateSteeringAngle(input.front_steering_angle, max_road_wheel_angle_,
                            "front steering angle");
  if (!status.ok()) {
    return status;
  }
  status = ValidateSteeringAngle(input.rear_steering_angle,
                                 max_road_wheel_angle_, "rear steering angle");
  if (!status.ok()) {
    return status;
  }
  status = ValidateAcceleration(input.longitudinal_acceleration,
                                max_acceleration_, max_deceleration_);
  if (!status.ok()) {
    return status;
  }

  const double tan_front = std::tan(input.front_steering_angle);
  const double tan_rear = std::tan(input.rear_steering_angle);
  KinematicMotion motion;
  motion.lateral_velocity_ratio = distance_cg_to_rear_axle_ * tan_front +
                                  distance_cg_to_front_axle_ * tan_rear;
  motion.lateral_velocity_ratio /= wheel_base_;
  motion.curvature = (tan_front - tan_rear) / wheel_base_;
  motion.longitudinal_acceleration = input.longitudinal_acceleration;
  return Propagate(horizon, dt_, canonical_reference_point(), canonical_state,
                   motion, predicted_canonical_state);
}

Status FourWheelSteeringKinematicModel::PredictWithHeldCurvature(
    const double horizon, const VehicleState& canonical_state,
    VehicleState* predicted_canonical_state) const {
  return Propagate(horizon, dt_, canonical_reference_point(), canonical_state,
                   HeldCurvatureMotion(canonical_state),
                   predicted_canonical_state);
}

Status VehicleModelFactory::Create(
    const VehicleModelConfig& config, const VehicleDescription& description,
    std::unique_ptr<VehicleModelImplementation>* model) {
  if (model == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "vehicle model output is null");
  }
  model->reset();

  const double wheel_base = description.wheel_base();
  if (!std::isfinite(wheel_base) || wheel_base <= 0.0) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "vehicle wheel base must be finite and positive");
  }
  if (!config.has_model_type()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "vehicle model type must be configured");
  }
  const double max_road_wheel_angle = description.max_road_wheel_angle();
  const double max_acceleration = description.max_acceleration();
  const double max_deceleration = description.max_deceleration();
  if (!std::isfinite(max_road_wheel_angle) || max_road_wheel_angle <= 0.0 ||
      max_road_wheel_angle >= kHalfPi || !std::isfinite(max_acceleration) ||
      max_acceleration < 0.0 || !std::isfinite(max_deceleration) ||
      max_deceleration > 0.0) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "vehicle steering or acceleration limits are invalid");
  }

  switch (config.model_type()) {
    case VEHICLE_MODEL_TYPE_ACKERMANN_KINEMATIC: {
      if (!config.has_ackermann_kinematic_model()) {
        return Status(ErrorCode::PLANNING_ERROR,
                      "Ackermann model configuration is missing");
      }
      const double dt = config.ackermann_kinematic_model().dt();
      if (!std::isfinite(dt) || dt <= 0.0) {
        return Status(ErrorCode::PLANNING_ERROR,
                      "Ackermann model dt must be finite and positive");
      }
      *model = std::unique_ptr<VehicleModelImplementation>(
          new AckermannKinematicModel(dt, wheel_base, max_road_wheel_angle,
                                      max_acceleration, max_deceleration));
      return Status::OK();
    }
    case VEHICLE_MODEL_TYPE_FOUR_WHEEL_STEERING_KINEMATIC: {
      const double center_of_mass_offset = description.center_of_mass_offset();
      if (!std::isfinite(center_of_mass_offset) ||
          center_of_mass_offset <= 0.0 || center_of_mass_offset >= wheel_base) {
        return Status(
            ErrorCode::PLANNING_ERROR,
            "center_of_mass_offset must lie between the rear and front axles");
      }
      if (!config.has_fws_kinematic_model()) {
        return Status(ErrorCode::PLANNING_ERROR,
                      "4WS model configuration is missing");
      }
      const double dt = config.fws_kinematic_model().dt();
      if (!std::isfinite(dt) || dt <= 0.0) {
        return Status(ErrorCode::PLANNING_ERROR,
                      "4WS model dt must be finite and positive");
      }
      *model = std::unique_ptr<VehicleModelImplementation>(
          new FourWheelSteeringKinematicModel(
              dt, wheel_base, center_of_mass_offset, max_road_wheel_angle,
              max_acceleration, max_deceleration));
      return Status::OK();
    }
    default:
      return Status(ErrorCode::PLANNING_ERROR,
                    absl::StrCat("Unsupported vehicle model type: ",
                                 config.model_type()));
  }
}

}  // namespace common
}  // namespace apollo
