// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_graph_transport.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "Eigen/Cholesky"

namespace apollo::localization::openloc_diagnostic {
namespace loc = ::openloc::localization;
namespace u = loc::unified;
namespace {

int64_t Nanoseconds(double time) {
  const long double ns = static_cast<long double>(time) * 1e9L;
  if (!std::isfinite(ns) || ns <= 0 ||
      ns > std::numeric_limits<int64_t>::max() - 1.0L)
    throw std::invalid_argument("invalid native graph timestamp");
  return static_cast<int64_t>(std::round(ns));
}

template <typename Matrix>
void WriteMatrix(const Matrix& matrix,
                 google::protobuf::RepeatedField<double>* output) {
  if (!matrix.allFinite() ||
      matrix.size() > static_cast<Eigen::Index>(kMaximumNativeGraphBytes / sizeof(double)))
    throw std::invalid_argument("oversized or nonfinite native graph matrix");
  for (Eigen::Index row = 0; row < matrix.rows(); ++row)
    for (Eigen::Index col = 0; col < matrix.cols(); ++col)
      output->Add(matrix(row, col));
}

Eigen::MatrixXd ReadMatrix(const google::protobuf::RepeatedField<double>& input,
                           uint32_t rows, uint32_t cols) {
  if (rows == 0 || cols == 0 ||
      static_cast<uint64_t>(rows) * cols != static_cast<uint64_t>(input.size()))
    throw std::invalid_argument("invalid native graph matrix shape");
  Eigen::MatrixXd matrix(rows, cols);
  for (uint32_t row = 0; row < rows; ++row)
    for (uint32_t col = 0; col < cols; ++col) {
      const double value = input.Get(row * cols + col);
      if (!std::isfinite(value))
        throw std::invalid_argument("nonfinite native graph matrix");
      matrix(row, col) = value;
    }
  return matrix;
}

}  // namespace

NativeJointOdometryInput UnavailableNativeGraph(
    NativeOdometryDiagnostic odometry, const std::string& model_id,
    const std::string& detail) {
  if (model_id.empty() || model_id.size() > 128 ||
      detail.empty() || detail.size() > 1024 ||
      odometry.has_measurement_time() != odometry.has_receive_time())
    throw std::invalid_argument("invalid unavailable native graph declaration");
  NativeJointOdometryInput output;
  output.set_error_model_id(model_id);
  output.set_model_available(false);
  output.set_model_detail(detail);
  output.set_measurement_ns(
      odometry.has_measurement_time() ? Nanoseconds(odometry.measurement_time()) : 0);
  output.set_receive_ns(
      odometry.has_receive_time() ? Nanoseconds(odometry.receive_time()) : 0);
  *output.mutable_odometry() = std::move(odometry);
  return output;
}

NativeJointOdometryInput EncodeNativeGraph(
    const NativeGraphFrame& frame, NativeOdometryDiagnostic odometry,
    const std::string& model_id) {
  if (!frame.assessment.ok())
    return UnavailableNativeGraph(std::move(odometry), model_id, frame.assessment.message);
  NativeJointOdometryInput output;
  *output.mutable_odometry() = std::move(odometry);
  output.set_error_model_id(model_id);
  output.set_measurement_ns(Nanoseconds(frame.nominal.stamp.time));
  output.set_receive_ns(Nanoseconds(frame.nominal.stamp.receive_time));
  output.set_model_available(frame.assessment.ok());
  output.set_model_detail(frame.assessment.message);
  if (!frame.joint.joint_error || !frame.joint.covariance_model_valid ||
      frame.joint.uncertainty_qualified ||
      frame.joint.stamp.time != frame.nominal.stamp.time ||
      frame.joint.stamp.sequence != frame.nominal.stamp.sequence ||
      frame.joint.stamp.clock_id != "unix" || frame.nominal.stamp.clock_id != "unix" ||
      frame.joint.epoch != frame.nominal.epoch ||
      !frame.joint.position.isApprox(frame.nominal.position, 0) ||
      !frame.joint.orientation.isApprox(frame.nominal.orientation, 0))
    throw std::invalid_argument("native graph snapshot does not match nominal ODOM");
  const auto& model = *frame.joint.joint_error;
  const auto dimension = model.covariance.rows();
  if (dimension < 15 || dimension > kMaximumNativeGraphDimension ||
      model.covariance.cols() != dimension ||
      !model.covariance.isIdentity(0) ||
      model.output_jacobian.rows() != 15 ||
      model.output_jacobian.cols() != dimension)
    throw std::invalid_argument("native graph requires its actual bounded unit basis");
  output.set_dimension(dimension);
  output.set_unit_covariance_basis(true);
  WriteMatrix(frame.joint.covariance, output.mutable_covariance());
  WriteMatrix(model.output_jacobian, output.mutable_output_jacobian());
  if (frame.motion) {
    const auto& motion = *frame.motion;
    if (motion.epoch != frame.joint.epoch ||
        motion.end.time != frame.joint.stamp.time ||
        motion.end.sequence != frame.joint.stamp.sequence)
      throw std::invalid_argument("native graph motion/snapshot endpoint mismatch");
    output.set_previous_ns(Nanoseconds(motion.start.time));
    output.set_previous_sequence(motion.start.sequence);
    output.set_previous_dimension(frame.previous_dimension);
    WriteMatrix(motion.cross_covariance, output.mutable_cross_covariance());
    WriteMatrix(motion.joint_cross_covariance, output.mutable_joint_cross_covariance());
    for (const auto& source : motion.sources) {
      auto* item = output.add_sources();
      item->set_source(source.source);
      item->set_sequence(source.sequence);
    }
  }
  if (frame.scan) {
    const auto& scan = *frame.scan;
    output.set_scan_sequence(scan.reference.sequence);
    output.set_scan_reference_ns(Nanoseconds(scan.reference.time));
    output.set_scan_receive_ns(Nanoseconds(scan.reference.receive_time));
    output.set_lidar_revision(scan.lidar_revision);
    if (scan.reference.clock_id != "unix" ||
        scan.conditioning_state.stamp.clock_id != "unix" ||
        !scan.conditioning_state.joint_error ||
        !scan.conditioning_state.joint_error->covariance.isApprox(model.covariance, 0) ||
        !scan.conditioning_state.joint_error->output_jacobian.isApprox(
            model.output_jacobian, 0) ||
        scan.conditioning_state.stamp.sequence != frame.joint.stamp.sequence ||
        scan.conditioning_state.stamp.time != frame.joint.stamp.time ||
        scan.conditioning_state.epoch != frame.joint.epoch ||
        scan.points_in_reference_base.size() != scan.point_jacobians.size())
      throw std::invalid_argument("native scan support has another conditioning state");
    for (std::size_t i = 0; i < scan.point_jacobians.size(); ++i) {
      if (scan.point_jacobians[i].rows() != 3 ||
          scan.point_jacobians[i].cols() != dimension)
        throw std::invalid_argument("native scan loading has another point/basis shape");
      WriteMatrix(scan.points_in_reference_base[i], output.mutable_scan_points());
      WriteMatrix(scan.point_jacobians[i], output.mutable_scan_jacobians());
    }
  }
  DecodeNativeGraph(output, model_id, kMaximumNativeGraphDimension);
  return output;
}

NativeDecodedGraph DecodeNativeGraph(const NativeJointOdometryInput& input,
                                     const std::string& model_id,
                                     uint32_t maximum_dimension) {
  if (!input.IsInitialized() || input.ByteSizeLong() > kMaximumNativeGraphBytes ||
      model_id.empty() || input.error_model_id() != model_id ||
      !input.model_available() || !input.odometry().usable() ||
      !input.odometry().new_measurement() || input.odometry().trusted() ||
      !input.odometry().diagnostic_only() ||
      input.dimension() < 15 || input.dimension() > maximum_dimension ||
      input.dimension() > kMaximumNativeGraphDimension ||
      !input.has_unit_covariance_basis() || !input.unit_covariance_basis() ||
      input.measurement_ns() != Nanoseconds(input.odometry().measurement_time()) ||
      input.receive_ns() != Nanoseconds(input.odometry().receive_time()) ||
      input.odometry().session().empty() || input.odometry().generation() == 0 ||
      input.odometry().source_sequence() == 0 ||
      input.receive_ns() < input.measurement_ns() - 20000000 ||
      input.sources_size() > 16384)
    throw std::invalid_argument("invalid coherent native graph packet");
  NativeDecodedGraph output;
  auto& sample = output.odometry;
  sample.sequence = input.odometry().source_sequence();
  sample.odometry.timestamp = {input.measurement_ns(), loc::ClockDomain::kUnixEpoch};
  sample.receive_time = {input.receive_ns(), loc::ClockDomain::kUnixEpoch};
  sample.odometry.epoch = {input.odometry().session(), input.odometry().generation()};
  sample.odometry.parent_frame = {"native-odom"};
  sample.odometry.child_frame = {"imu"};
  const auto& stamped = input.odometry().odom_from_imu();
  const auto suffix = input.odometry().session() + "/" +
                      std::to_string(input.odometry().generation());
  if (!input.odometry().has_odom_from_imu() ||
      stamped.header().frame_id() != "openloc_diag/odom/" + suffix ||
      stamped.child_frame_id() != "openloc_diag/imu/" + suffix ||
      stamped.header().timestamp_sec() != input.odometry().measurement_time())
    throw std::invalid_argument("native graph ODOM frame/source identity mismatch");
  const auto& pose = stamped.transform();
  if (!pose.has_translation() || !pose.translation().has_x() ||
      !pose.translation().has_y() || !pose.translation().has_z() ||
      !pose.has_rotation() || !pose.rotation().has_qw() ||
      !pose.rotation().has_qx() || !pose.rotation().has_qy() || !pose.rotation().has_qz())
    throw std::invalid_argument("incomplete native graph pose");
  sample.odometry.pose.translation = {pose.translation().x(), pose.translation().y(),
                                       pose.translation().z()};
  sample.odometry.pose.rotation = Eigen::Quaterniond(
      pose.rotation().qw(), pose.rotation().qx(), pose.rotation().qy(), pose.rotation().qz());
  if (!sample.odometry.pose.translation.allFinite() ||
      !sample.odometry.pose.rotation.coeffs().allFinite() ||
      std::abs(sample.odometry.pose.rotation.norm() - 1.0) > 1e-9)
    throw std::invalid_argument("invalid native graph pose");
  sample.covariance = ReadMatrix(input.covariance(), 15, 15);
  sample.joint_error = u::JointLocalError{
      Eigen::MatrixXd::Identity(input.dimension(), input.dimension()),
      ReadMatrix(input.output_jacobian(), 15, input.dimension())};
  if (!sample.covariance.isApprox(sample.covariance.transpose(), 1e-9) ||
      Eigen::LLT<u::Matrix15d>(sample.covariance).info() != Eigen::Success ||
      !(sample.joint_error->output_jacobian *
        sample.joint_error->output_jacobian.transpose()).isApprox(sample.covariance, 1e-9))
    throw std::invalid_argument("native graph projection/marginal mismatch");
  sample.covariance_model_valid = true;
  sample.uncertainty_qualified = false;
  if (input.has_previous_ns()) {
    if (input.previous_ns() <= 0 || input.previous_ns() >= input.measurement_ns() ||
        input.previous_sequence() == 0 || input.previous_sequence() >= sample.sequence ||
        input.previous_dimension() < 15 ||
        input.previous_dimension() > maximum_dimension ||
        input.previous_dimension() > kMaximumNativeGraphDimension)
      throw std::invalid_argument("invalid native graph motion endpoints");
    output.motion.emplace();
    auto& motion = *output.motion;
    motion.epoch = sample.odometry.epoch;
    motion.start_time = {input.previous_ns(), loc::ClockDomain::kUnixEpoch};
    motion.end_time = sample.odometry.timestamp;
    motion.start_sequence = input.previous_sequence();
    motion.end_sequence = sample.sequence;
    motion.cross_covariance = ReadMatrix(input.cross_covariance(), 15, 15);
    motion.joint_cross_covariance = ReadMatrix(
        input.joint_cross_covariance(), input.previous_dimension(), input.dimension());
    for (const auto& source : input.sources()) {
      if (source.source().empty() || source.source().size() > 128 ||
          source.sequence() == 0)
        throw std::invalid_argument("invalid native graph primitive identity");
      motion.sources.push_back({source.source(), source.sequence()});
    }
  } else if (input.has_previous_sequence() || input.has_previous_dimension() ||
             input.cross_covariance_size() || input.joint_cross_covariance_size() ||
             input.sources_size()) {
    throw std::invalid_argument("partial native graph motion link");
  }
  if (input.has_scan_sequence()) {
    const auto count = input.scan_points_size() / 3;
    if (input.scan_sequence() == 0 || !input.has_scan_reference_ns() ||
        input.scan_reference_ns() <= 0 || input.scan_reference_ns() > input.measurement_ns() ||
        !input.has_scan_receive_ns() || input.scan_receive_ns() <= 0 ||
        input.scan_receive_ns() > input.receive_ns() ||
        input.scan_receive_ns() < input.scan_reference_ns() - 20000000 ||
        !input.has_lidar_revision() || input.lidar_revision() == 0 ||
        count == 0 || input.scan_points_size() % 3 ||
        static_cast<uint64_t>(count) * 3 * input.dimension() !=
            static_cast<uint64_t>(input.scan_jacobians_size()))
      throw std::invalid_argument("invalid shared native scan support shape/reference");
    output.scan.emplace();
    auto& scan = *output.scan;
    scan.reference = {input.scan_reference_ns() * 1e-9, input.scan_receive_ns() * 1e-9,
                       input.scan_sequence(), "unix"};
    scan.conditioning_state.stamp = {input.measurement_ns() * 1e-9,
        input.receive_ns() * 1e-9, sample.sequence, "unix"};
    scan.conditioning_state.epoch = {sample.odometry.epoch.producer_session,
                                     sample.odometry.epoch.generation};
    scan.conditioning_state.position = sample.odometry.pose.translation;
    scan.conditioning_state.orientation = sample.odometry.pose.rotation;
    scan.conditioning_state.covariance = sample.covariance;
    scan.conditioning_state.joint_error = sample.joint_error;
    scan.conditioning_state.covariance_model_valid = true;
    scan.conditioning_state.uncertainty_qualified = false;
    scan.conditioning_state.valid = true;
    scan.lidar_revision = input.lidar_revision();
    const auto points = ReadMatrix(input.scan_points(), count, 3);
    const auto loadings = ReadMatrix(input.scan_jacobians(), count * 3, input.dimension());
    for (int i = 0; i < count; ++i) {
      scan.points_in_reference_base.push_back(points.row(i).transpose());
      scan.point_jacobians.push_back(loadings.middleRows(i * 3, 3));
    }
  } else if (input.has_scan_reference_ns() || input.has_scan_receive_ns() ||
             input.has_lidar_revision() ||
             input.scan_points_size() || input.scan_jacobians_size()) {
    throw std::invalid_argument("partial shared native scan support");
  }
  return output;
}

}  // namespace apollo::localization::openloc_diagnostic
