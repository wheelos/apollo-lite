// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_graph_transport.h"

#include <stdexcept>

#include "gtest/gtest.h"

namespace apollo::localization::openloc_diagnostic {
namespace {
namespace u = ::openloc::localization::unified;

NativeGraphFrame Frame() {
  NativeGraphFrame frame;
  frame.nominal.stamp = {1.1, 1.11, 12, "unix"};
  frame.nominal.epoch = {"graph-transport", 1};
  frame.nominal.valid = true;
  frame.nominal.covariance.setIdentity();
  frame.joint = frame.nominal;
  frame.joint.joint_error = u::JointLocalError{
      Eigen::MatrixXd::Identity(21, 21), Eigen::MatrixXd::Zero(15, 21)};
  frame.joint.joint_error->output_jacobian.leftCols(15).setIdentity();
  frame.motion.emplace();
  frame.motion->start = {1.0, 1.01, 2, "unix"};
  frame.motion->end = frame.nominal.stamp;
  frame.motion->epoch = frame.nominal.epoch;
  frame.motion->cross_covariance.setIdentity();
  frame.motion->joint_cross_covariance = Eigen::MatrixXd::Zero(18, 21);
  frame.motion->joint_cross_covariance.leftCols(18).setIdentity();
  frame.motion->sources = {{"imu", 11}, {"lidar", 4}};
  frame.previous_dimension = 18;
  frame.scan.emplace();
  frame.scan->reference = {1.05, 1.06, 4, "unix"};
  frame.scan->conditioning_state = frame.joint;
  frame.scan->lidar_revision = 2;
  frame.scan->points_in_reference_base = {{1, 2, 3}, {4, 5, 6}};
  frame.scan->point_jacobians = {
      Eigen::MatrixXd::Random(3, 21), Eigen::MatrixXd::Random(3, 21)};
  return frame;
}

NativeJointOdometryInput Packet(const NativeGraphFrame& frame) {
  NativePublication publication(0.2);
  const auto now = NativePublication::Clock::now();
  publication.Update(frame.nominal, {}, now);
  return EncodeNativeGraph(frame, publication.Sample(10, now), "native-joint-v1");
}

TEST(NativeGraphTransportTest, PreservesRectangularMotionAndOriginalDelayedScan) {
  const auto frame = Frame();
  const auto encoded = Packet(frame);
  NativeJointOdometryInput transported;
  ASSERT_TRUE(transported.ParseFromString(encoded.SerializeAsString()));
  const auto decoded = DecodeNativeGraph(transported, "native-joint-v1", 32);
  ASSERT_TRUE(decoded.motion);
  EXPECT_TRUE(decoded.motion->joint_cross_covariance.isApprox(
      frame.motion->joint_cross_covariance, 0));
  EXPECT_EQ(decoded.motion->sources[1].source, "lidar");
  ASSERT_TRUE(decoded.scan);
  EXPECT_DOUBLE_EQ(decoded.scan->reference.time, 1.05);
  EXPECT_DOUBLE_EQ(decoded.scan->reference.receive_time, 1.06);
  EXPECT_EQ(decoded.scan->conditioning_state.stamp.sequence, 12);
  EXPECT_EQ(decoded.scan->reference.sequence, 4);
  EXPECT_TRUE(decoded.scan->point_jacobians[1].isApprox(
      frame.scan->point_jacobians[1], 0));
  EXPECT_TRUE(decoded.odometry.joint_error->output_jacobian.isApprox(
      frame.joint.joint_error->output_jacobian, 0));
}

TEST(NativeGraphTransportTest, TerminalUnavailableDoesNotInventAnErrorModelOrSource) {
  NativePublication publication(0.2);
  publication.Invalidate({u::Reason::INVALID_INPUT, "terminal_source_gap"});
  const auto before_start = UnavailableNativeGraph(
      publication.Sample(10), "native-joint-v1", "terminal_source_gap");
  EXPECT_TRUE(before_start.IsInitialized());
  EXPECT_FALSE(before_start.model_available());
  EXPECT_FALSE(before_start.has_dimension());
  EXPECT_FALSE(before_start.odometry().has_odom_from_imu());
  EXPECT_EQ(before_start.measurement_ns(), 0);
  EXPECT_THROW(DecodeNativeGraph(before_start, "native-joint-v1", 32),
               std::invalid_argument);
  const auto frame = Frame();
  NativePublication active(0.2);
  const auto now = NativePublication::Clock::now();
  active.Update(frame.nominal, {}, now);
  active.Invalidate({u::Reason::INVALID_INPUT, "terminal_source_gap"});
  const auto after_start = UnavailableNativeGraph(
      active.Sample(11, now), "native-joint-v1", "terminal_source_gap");
  EXPECT_FALSE(after_start.odometry().usable());
  EXPECT_EQ(after_start.measurement_ns(), 1100000000);
  EXPECT_EQ(after_start.odometry().source_sequence(), 12);
  EXPECT_EQ(after_start.covariance_size(), 0);
}

TEST(NativeGraphTransportTest, RejectsTruncationFalseBasisAndChangedIdentity) {
  auto packet = Packet(Frame());
  packet.mutable_scan_jacobians()->RemoveLast();
  EXPECT_THROW(DecodeNativeGraph(packet, "native-joint-v1", 32), std::invalid_argument);
  packet = Packet(Frame());
  packet.set_unit_covariance_basis(false);
  EXPECT_THROW(DecodeNativeGraph(packet, "native-joint-v1", 32), std::invalid_argument);
  packet = Packet(Frame());
  packet.set_measurement_ns(packet.measurement_ns() + 1);
  EXPECT_THROW(DecodeNativeGraph(packet, "native-joint-v1", 32), std::invalid_argument);
  packet = Packet(Frame());
  EXPECT_THROW(DecodeNativeGraph(packet, "another-model", 32), std::invalid_argument);
  EXPECT_THROW(DecodeNativeGraph(packet, "native-joint-v1", 16), std::invalid_argument);
  packet.set_covariance(0, 2);
  EXPECT_THROW(DecodeNativeGraph(packet, "native-joint-v1", 32), std::invalid_argument);
}

}  // namespace
}  // namespace apollo::localization::openloc_diagnostic
