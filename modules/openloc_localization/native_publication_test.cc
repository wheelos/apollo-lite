// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_publication.h"

#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

namespace apollo::localization::openloc_diagnostic {
namespace {
using Clock = NativePublication::Clock;

NativePublication::State State() {
  NativePublication::State state;
  state.stamp = {100, 100.05, 1, "unix"};
  state.epoch = {"test", 1};
  state.valid = true;
  state.position = Eigen::Vector3d(1, 2, 3);
  state.velocity = Eigen::Vector3d(4, 5, 6);
  return state;
}

TEST(NativePublicationTest, PreservesSourceAndExpiresWithoutRenewingTime) {
  NativePublication publication(0.2);
  const auto arrival = Clock::now();
  EXPECT_FALSE(publication.Sample(101, arrival).has_odom_from_imu());
  const auto state = State();
  publication.Update(state, {}, arrival);
  const auto first = publication.Sample(101, arrival);
  ASSERT_TRUE(first.usable());
  EXPECT_TRUE(first.new_measurement());
  EXPECT_FALSE(first.trusted());
  EXPECT_TRUE(first.diagnostic_only());
  EXPECT_FALSE(first.has_recorded_source_clock());
  EXPECT_FALSE(first.has_processing_delay_s());
  EXPECT_DOUBLE_EQ(first.measurement_time(), 100);
  EXPECT_DOUBLE_EQ(first.receive_time(), 100.05);
  EXPECT_DOUBLE_EQ(first.odom_from_imu().header().timestamp_sec(), 100);
  EXPECT_EQ(first.odom_from_imu().header().frame_id(),
            "openloc_diag/odom/test/1");
  EXPECT_DOUBLE_EQ(first.odom_from_imu().transform().translation().x(), 1);
  const auto held =
      publication.Sample(102, arrival + std::chrono::milliseconds(100));
  EXPECT_TRUE(held.usable());
  EXPECT_FALSE(held.new_measurement());
  EXPECT_EQ(held.odom_from_imu().SerializeAsString(),
            first.odom_from_imu().SerializeAsString());
  const auto stale =
      publication.Sample(103, arrival + std::chrono::milliseconds(200));
  EXPECT_FALSE(stale.usable());
  EXPECT_FALSE(stale.has_odom_from_imu());
  EXPECT_FALSE(stale.has_velocity_in_odom());
  EXPECT_FALSE(stale.has_native_error_state());
  EXPECT_EQ(stale.reason(), "IMU_STALE");
  EXPECT_DOUBLE_EQ(stale.measurement_time(), 100);
}

TEST(NativePublicationTest, CarriesActualNativeErrorWithoutInventingQualification) {
  NativePublication publication(0.2);
  const auto arrival = Clock::now();
  auto state = State();
  state.covariance.setIdentity();
  state.covariance(0, 3) = state.covariance(3, 0) = 0.125;
  state.gyro_bias = Eigen::Vector3d(0.01, 0.02, 0.03);
  state.accel_bias = Eigen::Vector3d(0.1, 0.2, 0.3);
  state.angular_velocity = Eigen::Vector3d(0.4, 0.5, 0.6);
  state.covariance_model_valid = false;
  state.uncertainty_qualified = false;
  publication.Update(state, {}, arrival);
  const auto first = publication.Sample(101, arrival);
  ASSERT_TRUE(first.usable());
  ASSERT_TRUE(first.has_native_error_state());
  const auto& error = first.native_error_state();
  ASSERT_TRUE(error.IsInitialized());
  ASSERT_EQ(error.covariance_size(), 225);
  for (int row = 0; row < 15; ++row)
    for (int column = 0; column < 15; ++column)
      EXPECT_DOUBLE_EQ(error.covariance(15 * row + column),
                       state.covariance(row, column));
  EXPECT_DOUBLE_EQ(error.gyro_bias().y(), state.gyro_bias.y());
  EXPECT_DOUBLE_EQ(error.accel_bias().z(), state.accel_bias.z());
  EXPECT_DOUBLE_EQ(error.angular_velocity().x(), state.angular_velocity.x());
  EXPECT_FALSE(error.covariance_model_valid());
  EXPECT_FALSE(error.uncertainty_qualified());
  EXPECT_FALSE(first.trusted());
  const auto held = publication.Sample(102, arrival);
  EXPECT_FALSE(held.new_measurement());
  EXPECT_EQ(held.native_error_state().SerializeAsString(),
            error.SerializeAsString());
  EXPECT_DOUBLE_EQ(held.measurement_time(), first.measurement_time());
  state.stamp.sequence++;
  state.stamp.time += 0.01;
  state.stamp.receive_time += 0.01;
  state.covariance(0, 0) = 2;
  publication.Update(state, {}, arrival);
  const auto second = publication.Sample(103, arrival);
  EXPECT_TRUE(second.new_measurement());
  EXPECT_DOUBLE_EQ(second.native_error_state().covariance(0), 2);
  EXPECT_EQ(second.source_sequence(), state.stamp.sequence);
}

TEST(NativePublicationTest, RecordedSourceClockPreservesDelayAndDoesNotRelaxSourceAge) {
  NativePublication recorded(0.2, true);
  NativePublication live(0.2);
  const auto arrival = Clock::now();
  const auto state = State();
  recorded.Update(state, {}, arrival);
  live.Update(state, {}, arrival);
  const auto now = arrival + std::chrono::seconds(2);
  const auto output = recorded.Sample(103, now);
  EXPECT_TRUE(output.usable());
  EXPECT_TRUE(output.recorded_input());
  EXPECT_TRUE(output.recorded_source_clock());
  EXPECT_FALSE(output.trusted());
  EXPECT_DOUBLE_EQ(output.processing_delay_s(), 2);
  EXPECT_NEAR(output.measurement_age(), 0.05, 1e-12);
  EXPECT_DOUBLE_EQ(output.measurement_time(), state.stamp.time);
  EXPECT_DOUBLE_EQ(output.receive_time(), state.stamp.receive_time);
  EXPECT_FALSE(live.Sample(103, now).usable());

  auto stale = state;
  ++stale.stamp.sequence;
  stale.stamp.time += 1;
  stale.stamp.receive_time = stale.stamp.time + 0.3;
  recorded.Update(stale, {}, arrival);
  EXPECT_FALSE(recorded.Sample(104, now).usable());
  recorded.Invalidate({::openloc::localization::unified::Reason::INVALID_INPUT,
                       "terminal replay source failure"});
  EXPECT_FALSE(recorded.Sample(105, now).usable());
}

TEST(NativePublicationTest, RejectsNonfiniteNativeErrorBeforeReplacingSnapshot) {
  NativePublication publication(0.2);
  const auto arrival = Clock::now();
  const auto initial = State();
  publication.Update(initial, {}, arrival);
  for (int field = 0; field < 4; ++field) {
    auto state = initial;
    state.stamp.sequence++;
    state.stamp.time += 0.01;
    state.stamp.receive_time += 0.01;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (field == 0) state.covariance(4, 7) = nan;
    else if (field == 1) state.gyro_bias.x() = nan;
    else if (field == 2) state.accel_bias.y() = nan;
    else state.angular_velocity.z() = nan;
    EXPECT_THROW(publication.Update(state, {}, arrival), std::invalid_argument);
    const auto retained = publication.Sample(101, arrival);
    EXPECT_EQ(retained.source_sequence(), initial.stamp.sequence);
    EXPECT_DOUBLE_EQ(retained.native_error_state().covariance(4 * 15 + 7), 0);
  }
}

TEST(NativePublicationTest, EpochChangeGetsDistinctFramesAndRejectsRegression) {
  NativePublication publication(0.2);
  const auto arrival = Clock::now();
  auto state = State();
  publication.Update(state, {}, arrival);
  const auto first = publication.Sample(101, arrival);
  EXPECT_THROW(publication.Update(state, {}, arrival), std::invalid_argument);
  state.epoch.generation = 2;
  publication.Update(state, {}, arrival);
  const auto reset = publication.Sample(102, arrival);
  EXPECT_TRUE(reset.epoch_changed());
  EXPECT_TRUE(reset.new_measurement());
  EXPECT_NE(reset.odom_from_imu().header().frame_id(),
            first.odom_from_imu().header().frame_id());
  EXPECT_NE(reset.odom_from_imu().child_frame_id(),
            first.odom_from_imu().child_frame_id());
  state.epoch.generation = 1;
  EXPECT_THROW(publication.Update(state, {}, arrival), std::invalid_argument);
}

TEST(NativePublicationTest, FaultsAndInvalidGeometryNeverEmitAPose) {
  NativePublication publication(0.2);
  const auto arrival = Clock::now();
  auto state = State();
  publication.Update(state,
                     {::openloc::localization::unified::Reason::IMU_GAP, "gap"},
                     arrival);
  const auto fault = publication.Sample(101, arrival);
  EXPECT_FALSE(fault.usable());
  EXPECT_FALSE(fault.has_odom_from_imu());
  EXPECT_FALSE(fault.has_native_error_state());
  EXPECT_EQ(fault.reason(), "IMU_GAP");
  state.stamp.sequence++;
  state.stamp.time += 0.01;
  state.position.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(publication.Update(state, {}, arrival), std::invalid_argument);
  state = State();
  state.epoch.session = "../bad";
  EXPECT_THROW(publication.Update(state, {}, arrival), std::invalid_argument);
}

TEST(NativePublicationTest, TerminalFailureWithdrawsPoseAndCannotBeRenewed) {
  namespace u = ::openloc::localization::unified;
  NativePublication publication(0.2);
  const auto arrival = Clock::now();
  auto state = State();
  publication.Update(state, {}, arrival);
  ASSERT_TRUE(publication.Sample(101, arrival).usable());
  publication.Invalidate({u::Reason::INVALID_INPUT, "original producer gap"});
  publication.Invalidate({u::Reason::IMU_GAP, "later shutdown error"});
  const auto failed = publication.Sample(102, arrival);
  EXPECT_FALSE(failed.usable());
  EXPECT_FALSE(failed.new_measurement());
  EXPECT_FALSE(failed.has_odom_from_imu());
  EXPECT_FALSE(failed.has_velocity_in_odom());
  EXPECT_FALSE(failed.has_native_error_state());
  EXPECT_EQ(failed.reason(), "INVALID_INPUT");
  EXPECT_EQ(failed.detail(), "original producer gap");
  EXPECT_EQ(failed.source_sequence(), state.stamp.sequence);
  EXPECT_DOUBLE_EQ(failed.measurement_time(), state.stamp.time);
  EXPECT_DOUBLE_EQ(failed.receive_time(), state.stamp.receive_time);
  state.stamp.sequence++;
  state.stamp.time += 0.01;
  EXPECT_THROW(publication.Update(state, {}, arrival), std::runtime_error);
  EXPECT_FALSE(publication.Sample(103, arrival).usable());
  NativePublication uninitialized(0.2);
  EXPECT_THROW(uninitialized.Invalidate({}), std::invalid_argument);
  uninitialized.Invalidate({u::Reason::INVALID_INPUT, "incomplete first input"});
  const auto early = uninitialized.Sample(101, arrival);
  EXPECT_EQ(early.detail(), "incomplete first input");
  EXPECT_FALSE(early.has_measurement_time());
  EXPECT_FALSE(early.usable());
}

}  // namespace
}  // namespace apollo::localization::openloc_diagnostic
