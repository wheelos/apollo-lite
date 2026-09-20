/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "modules/common/vehicle_model/vehicle_model.h"

#include <cmath>
#include <limits>
#include <memory>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace {

VehicleDescription TestDescription(const double center_of_mass_offset = 1.4) {
  VehicleConfig config;
  auto* vehicle_param = config.mutable_vehicle_param();
  vehicle_param->set_wheel_base(2.8);
  vehicle_param->set_max_steer_angle(8.0);
  vehicle_param->set_steer_ratio(16.0);
  vehicle_param->set_max_acceleration(2.0);
  vehicle_param->set_max_deceleration(-6.0);
  return VehicleDescription(config, center_of_mass_offset);
}

VehicleModelConfig AckermannConfig(const double dt = 0.01) {
  VehicleModelConfig config;
  config.set_model_type(VEHICLE_MODEL_TYPE_ACKERMANN_KINEMATIC);
  config.mutable_ackermann_kinematic_model()->set_dt(dt);
  return config;
}

VehicleModelConfig FourWheelSteeringConfig(const double dt = 0.01) {
  VehicleModelConfig config;
  config.set_model_type(VEHICLE_MODEL_TYPE_FOUR_WHEEL_STEERING_KINEMATIC);
  config.mutable_fws_kinematic_model()->set_dt(dt);
  return config;
}

VehicleState RearAxleState(const double velocity = 10.0) {
  VehicleState state;
  state.set_x(0.0);
  state.set_y(0.0);
  state.set_heading(0.0);
  state.set_linear_velocity(velocity);
  state.set_linear_acceleration(0.0);
  state.set_kappa(0.0);
  state.set_reference_point(REAR_AXLE_CENTER);
  state.set_timestamp(100.0);
  state.set_gear(velocity < 0.0 ? canbus::Chassis::GEAR_REVERSE
                                : canbus::Chassis::GEAR_DRIVE);
  return state;
}

std::unique_ptr<VehicleModel> CreateModel(const VehicleModelConfig& config) {
  std::unique_ptr<VehicleModel> model;
  EXPECT_TRUE(VehicleModel::Create(config, TestDescription(), &model).ok());
  return model;
}

}  // namespace

TEST(VehicleModelTest, CanonicalReferencePointBelongsToImplementation) {
  auto ackermann = CreateModel(AckermannConfig());
  auto four_wheel_steering = CreateModel(FourWheelSteeringConfig());
  ASSERT_NE(ackermann, nullptr);
  ASSERT_NE(four_wheel_steering, nullptr);

  EXPECT_EQ(ackermann->canonical_reference_point(), REAR_AXLE_CENTER);
  EXPECT_EQ(four_wheel_steering->canonical_reference_point(), CENTER_OF_MASS);
}

TEST(VehicleModelTest, AckermannUsesExplicitFrontSteering) {
  auto model = CreateModel(AckermannConfig());
  VehicleModelInput input;
  input.front_steering_angle = 0.14;
  input.longitudinal_acceleration = 1.0;

  VehicleState predicted;
  ASSERT_TRUE(
      model->Predict(1.0, RearAxleState(), input, REAR_AXLE_CENTER, &predicted)
          .ok());
  EXPECT_NEAR(predicted.kappa(), std::tan(0.14) / 2.8, 1e-9);
  EXPECT_GT(predicted.heading(), 0.0);
  EXPECT_NEAR(predicted.linear_velocity(), 11.0, 1e-9);
}

TEST(VehicleModelTest, ZeroHorizonReturnsEquivalentStateAtTargetPoint) {
  auto model = CreateModel(AckermannConfig());
  VehicleState predicted;
  VehicleModelInput input;
  ASSERT_TRUE(
      model->Predict(0.0, RearAxleState(), input, FRONT_AXLE_CENTER,
                     &predicted)
          .ok());
  EXPECT_EQ(predicted.reference_point(), FRONT_AXLE_CENTER);
  EXPECT_NEAR(predicted.x(), 2.8, 1e-9);
  EXPECT_DOUBLE_EQ(predicted.y(), 0.0);
  EXPECT_DOUBLE_EQ(predicted.timestamp(), 100.0);
}

TEST(VehicleModelTest, FourWheelSteeringModesHaveExpectedYawOrdering) {
  auto model = CreateModel(FourWheelSteeringConfig());
  const VehicleState state = RearAxleState();

  VehicleModelInput front_only;
  front_only.front_steering_angle = 0.12;
  VehicleState front_only_state;
  ASSERT_TRUE(
      model
          ->Predict(1.0, state, front_only, REAR_AXLE_CENTER, &front_only_state)
          .ok());

  VehicleModelInput opposite_phase = front_only;
  opposite_phase.rear_steering_angle = -0.06;
  VehicleState opposite_state;
  ASSERT_TRUE(model
                  ->Predict(1.0, state, opposite_phase, REAR_AXLE_CENTER,
                            &opposite_state)
                  .ok());

  VehicleModelInput same_phase = front_only;
  same_phase.rear_steering_angle = 0.06;
  VehicleState same_state;
  ASSERT_TRUE(
      model->Predict(1.0, state, same_phase, REAR_AXLE_CENTER, &same_state)
          .ok());

  EXPECT_GT(opposite_state.heading(), front_only_state.heading());
  EXPECT_GT(front_only_state.heading(), same_state.heading());
}

TEST(VehicleModelTest, FourWheelSteeringSupportsCrabMotion) {
  auto model = CreateModel(FourWheelSteeringConfig());
  VehicleModelInput input;
  input.front_steering_angle = 0.1;
  input.rear_steering_angle = 0.1;

  VehicleState predicted;
  ASSERT_TRUE(
      model->Predict(1.0, RearAxleState(), input, REAR_AXLE_CENTER, &predicted)
          .ok());
  EXPECT_NEAR(predicted.heading(), 0.0, 1e-9);
  EXPECT_GT(predicted.y(), 0.0);
}

TEST(VehicleModelTest, FourWheelSteeringRoundTripsExternalReferencePoint) {
  auto model = CreateModel(FourWheelSteeringConfig());
  VehicleState state = RearAxleState();
  state.set_x(4.0);
  state.set_y(-2.0);
  state.set_heading(0.3);

  VehicleState predicted;
  ASSERT_TRUE(
      model->Predict(0.0, state, VehicleModelInput(), FRONT_AXLE_CENTER,
                     &predicted)
          .ok());
  EXPECT_EQ(predicted.reference_point(), FRONT_AXLE_CENTER);
  EXPECT_NEAR(predicted.x(), state.x() + 2.8 * std::cos(state.heading()),
              1e-9);
  EXPECT_NEAR(predicted.y(), state.y() + 2.8 * std::sin(state.heading()),
              1e-9);
  EXPECT_DOUBLE_EQ(predicted.heading(), state.heading());
}

TEST(VehicleModelTest, FourWheelSteeringFrontOnlyMatchesAckermann) {
  auto ackermann = CreateModel(AckermannConfig(0.001));
  auto four_wheel_steering = CreateModel(FourWheelSteeringConfig(0.001));
  VehicleModelInput input;
  input.front_steering_angle = 0.1;

  VehicleState ackermann_state;
  VehicleState four_wheel_steering_state;
  ASSERT_TRUE(ackermann
                  ->Predict(1.0, RearAxleState(), input, REAR_AXLE_CENTER,
                            &ackermann_state)
                  .ok());
  ASSERT_TRUE(four_wheel_steering
                  ->Predict(1.0, RearAxleState(), input, REAR_AXLE_CENTER,
                            &four_wheel_steering_state)
                  .ok());

  EXPECT_NEAR(ackermann_state.x(), four_wheel_steering_state.x(), 2e-3);
  EXPECT_NEAR(ackermann_state.y(), four_wheel_steering_state.y(), 2e-3);
  EXPECT_NEAR(ackermann_state.heading(), four_wheel_steering_state.heading(),
              2e-3);
}

TEST(VehicleModelTest, PreservesRequestedReferencePointAndMetadata) {
  auto model = CreateModel(AckermannConfig());
  VehicleState state = RearAxleState();
  state.set_driving_mode(canbus::Chassis::COMPLETE_AUTO_DRIVE);
  VehicleModelInput input;
  input.front_steering_angle = 0.1;

  VehicleState predicted;
  ASSERT_TRUE(
      model->Predict(0.5, state, input, FRONT_AXLE_CENTER, &predicted).ok());
  EXPECT_EQ(predicted.reference_point(), FRONT_AXLE_CENTER);
  EXPECT_EQ(predicted.gear(), state.gear());
  EXPECT_EQ(predicted.driving_mode(), state.driving_mode());
  EXPECT_NEAR(predicted.timestamp(), 100.5, 1e-9);

  math::Vec2d position;
  ASSERT_TRUE(
      model->PredictPositionWithHeldCurvature(0.0, state, &position).ok());
  EXPECT_DOUBLE_EQ(position.x(), state.x());
  EXPECT_DOUBLE_EQ(position.y(), state.y());
}

TEST(VehicleModelTest, PredictPreservesPoseRollAndPitch) {
  auto model = CreateModel(AckermannConfig());
  VehicleState state = RearAxleState();
  state.mutable_pose()->mutable_position()->set_x(state.x());
  state.mutable_pose()->mutable_position()->set_y(state.y());
  state.mutable_pose()->set_heading(state.heading());
  state.mutable_pose()->mutable_orientation()->set_qw(0.9);
  state.mutable_pose()->mutable_orientation()->set_qx(0.1);
  state.mutable_pose()->mutable_orientation()->set_qy(0.2);
  state.mutable_pose()->mutable_orientation()->set_qz(0.3);

  VehicleModelInput input;
  VehicleState predicted;
  ASSERT_TRUE(
      model->Predict(0.1, state, input, REAR_AXLE_CENTER, &predicted).ok());
  EXPECT_DOUBLE_EQ(predicted.pose().orientation().qw(), 0.9);
  EXPECT_DOUBLE_EQ(predicted.pose().orientation().qx(), 0.1);
  EXPECT_DOUBLE_EQ(predicted.pose().orientation().qy(), 0.2);
  EXPECT_DOUBLE_EQ(predicted.pose().orientation().qz(), 0.3);
  EXPECT_DOUBLE_EQ(predicted.pose().heading(), predicted.heading());
}

TEST(VehicleModelTest, SupportsReverseHeldCurvaturePrediction) {
  auto model = CreateModel(AckermannConfig());
  VehicleState state = RearAxleState(-4.0);
  state.set_kappa(0.1);
  state.set_linear_acceleration(-0.5);

  VehicleState predicted;
  ASSERT_TRUE(
      model->PredictWithHeldCurvature(1.0, state, REAR_AXLE_CENTER, &predicted)
          .ok());
  EXPECT_LT(predicted.x(), state.x());
  EXPECT_LT(predicted.heading(), state.heading());
  EXPECT_NEAR(predicted.linear_velocity(), -4.5, 1e-9);
}

TEST(VehicleModelTest, HeldCurvaturePreservesLateralVelocity) {
  auto model = CreateModel(FourWheelSteeringConfig());
  VehicleState state = RearAxleState();
  state.set_reference_point(CENTER_OF_MASS);
  state.set_lateral_velocity(1.0);
  state.set_kappa(0.05);

  VehicleState predicted;
  ASSERT_TRUE(model
                  ->PredictWithHeldCurvature(1.0, state, CENTER_OF_MASS,
                                             &predicted)
                  .ok());
  EXPECT_NEAR(predicted.lateral_velocity(), 1.0, 1e-9);
}

TEST(VehicleModelTest, HeldCurvatureDropsLateralRatioAtZeroSpeed) {
  auto model = CreateModel(FourWheelSteeringConfig());
  VehicleState state = RearAxleState(0.0);
  state.set_lateral_velocity(1.0);

  VehicleState predicted;
  ASSERT_TRUE(model
                  ->PredictWithHeldCurvature(1.0, state, CENTER_OF_MASS,
                                             &predicted)
                  .ok());
  EXPECT_DOUBLE_EQ(predicted.lateral_velocity(), 0.0);
}

TEST(VehicleModelTest, RejectsInvalidConfigurationAndInput) {
  std::unique_ptr<VehicleModel> model;
  EXPECT_FALSE(
      VehicleModel::Create(AckermannConfig(0.0), TestDescription(), &model)
          .ok());

  auto invalid_cg = FourWheelSteeringConfig();
  EXPECT_FALSE(
      VehicleModel::Create(invalid_cg, TestDescription(3.0), &model).ok());
  EXPECT_TRUE(
      VehicleModel::Create(AckermannConfig(), TestDescription(3.0), &model)
          .ok());

  model = CreateModel(AckermannConfig());
  VehicleModelInput rear_steering;
  rear_steering.rear_steering_angle = 0.1;
  VehicleState predicted;
  EXPECT_FALSE(model
                   ->Predict(1.0, RearAxleState(), rear_steering,
                             REAR_AXLE_CENTER, &predicted)
                   .ok());
  rear_steering.rear_steering_angle = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(model
                   ->Predict(1.0, RearAxleState(), rear_steering,
                             REAR_AXLE_CENTER, &predicted)
                   .ok());
  auto four_wheel_model = CreateModel(FourWheelSteeringConfig());
  rear_steering.rear_steering_angle =
      std::numeric_limits<double>::infinity();
  EXPECT_FALSE(four_wheel_model
                   ->Predict(1.0, RearAxleState(), rear_steering,
                             REAR_AXLE_CENTER, &predicted)
                   .ok());
  EXPECT_FALSE(model
                   ->PredictWithHeldCurvature(-1.0, RearAxleState(),
                                              REAR_AXLE_CENTER, &predicted)
                   .ok());
  EXPECT_FALSE(model
                   ->Predict(std::numeric_limits<double>::infinity(),
                             RearAxleState(), VehicleModelInput(),
                             REAR_AXLE_CENTER, &predicted)
                   .ok());
  EXPECT_FALSE(model
                   ->Predict(61.0, RearAxleState(), VehicleModelInput(),
                             REAR_AXLE_CENTER, &predicted)
                   .ok());
  EXPECT_FALSE(model
                   ->Predict(1.0, RearAxleState(), VehicleModelInput(),
                             static_cast<VehicleReferencePoint>(99), &predicted)
                   .ok());
  EXPECT_FALSE(model
                   ->PredictWithHeldCurvature(1.0, RearAxleState(),
                                              REAR_AXLE_CENTER, nullptr)
                   .ok());

  VehicleModelInput excessive_steering;
  excessive_steering.front_steering_angle = 0.6;
  EXPECT_FALSE(model
                   ->Predict(1.0, RearAxleState(), excessive_steering,
                             REAR_AXLE_CENTER, &predicted)
                   .ok());

  VehicleModelInput excessive_acceleration;
  excessive_acceleration.longitudinal_acceleration = 3.0;
  EXPECT_FALSE(model
                   ->Predict(1.0, RearAxleState(), excessive_acceleration,
                             REAR_AXLE_CENTER, &predicted)
                   .ok());
}

TEST(VehicleModelTest, SmallerIntegrationStepConverges) {
  auto coarse_model = CreateModel(AckermannConfig(0.1));
  auto fine_model = CreateModel(AckermannConfig(0.001));
  VehicleModelInput input;
  input.front_steering_angle = 0.2;

  VehicleState coarse;
  VehicleState fine;
  ASSERT_TRUE(
      coarse_model
          ->Predict(1.0, RearAxleState(), input, REAR_AXLE_CENTER, &coarse)
          .ok());
  ASSERT_TRUE(
      fine_model->Predict(1.0, RearAxleState(), input, REAR_AXLE_CENTER, &fine)
          .ok());
  const double curvature = std::tan(input.front_steering_angle) / 2.8;
  const double expected_heading = 10.0 * curvature;
  const double expected_x = std::sin(expected_heading) / curvature;
  const double expected_y = (1.0 - std::cos(expected_heading)) / curvature;
  EXPECT_NEAR(coarse.heading(), expected_heading, 1e-9);
  EXPECT_NEAR(fine.heading(), expected_heading, 1e-9);
  EXPECT_LT(std::abs(fine.x() - expected_x), std::abs(coarse.x() - expected_x));
  EXPECT_LT(std::abs(fine.y() - expected_y), std::abs(coarse.y() - expected_y));
}

}  // namespace common
}  // namespace apollo
