// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/runtime/messages.h"

#include <algorithm>
#include <cmath>

#include "Eigen/Cholesky"

namespace apollo {
namespace localization {
namespace unified {
namespace {

void SetPoint(const Eigen::Vector3d& value, apollo::common::Point3D* point) {
  point->set_x(value.x());
  point->set_y(value.y());
  point->set_z(value.z());
}

void EncodeEpoch(const Epoch& epoch, OdomIdentity* message) {
  message->set_producer_session(epoch.session);
  message->set_generation(epoch.generation);
}

}  // namespace

LocalConfig LocalPolicy(const LocalEstimatorConfig& config,
                        const std::string& session) {
  LocalConfig policy;
  policy.session = session;
  policy.clock_id = config.clock_id();
  policy.gravity = config.gravity();
  policy.max_sample_age = config.max_sample_age();
  policy.future_tolerance = config.future_tolerance();
  policy.max_imu_gap = config.max_imu_gap();
  policy.wheel_timeout = config.wheel_timeout();
  policy.stationary_speed = config.stationary_speed();
  policy.stationary_gyro = config.stationary_gyro();
  policy.stationary_accel_tolerance = config.stationary_accel_tolerance();
  policy.stationary_samples = config.stationary_samples();
  policy.gyro_noise = config.gyro_noise();
  policy.accel_noise = config.accel_noise();
  policy.gyro_bias_noise = config.gyro_bias_noise();
  policy.accel_bias_noise = config.accel_bias_noise();
  policy.max_position_std = config.max_position_std();
  policy.max_velocity_std = config.max_velocity_std();
  policy.max_attitude_std = config.max_attitude_std();
  policy.use_nonholonomic_constraint = config.use_nonholonomic_constraint();
  policy.nonholonomic_variance = config.nonholonomic_variance();
  policy.innovation_gate = config.innovation_gate();
  policy.history_duration = config.history_duration();
  policy.max_history_states = config.max_history_states();
  policy.enable_lidar = config.enable_lidar();
  if (config.enable_lidar()) {
    policy.lidar_frame = config.lidar_frame();
    policy.lidar_calibration_id = config.lidar_calibration_id();
    policy.lidar_point_noise_std = config.lidar_point_noise_std();
    policy.lidar_plane_noise_std = config.lidar_plane_noise_std();
    policy.lidar_voxel_size = config.lidar_voxel_size();
    policy.lidar_max_voxels = config.lidar_max_voxels();
    policy.lidar_max_scans = config.lidar_max_scans();
    policy.lidar_min_points_per_plane =
        config.lidar_min_points_per_plane();
    policy.lidar_min_correspondences = config.lidar_min_correspondences();
    policy.lidar_max_points = config.lidar_max_points();
    policy.lidar_max_correspondence_distance =
        config.lidar_max_correspondence_distance();
    policy.lidar_observed_singular_value =
        config.lidar_observed_singular_value();
    policy.lidar_innovation_gate = config.lidar_innovation_gate();
    policy.lidar_max_scan_age = config.lidar_max_scan_age();
    policy.lidar_future_tolerance = config.lidar_future_tolerance();
    policy.lidar_time_tolerance = config.lidar_time_tolerance();
    policy.lidar_max_translation_correction_rate =
        config.lidar_max_translation_correction_rate();
    policy.lidar_max_rotation_correction_rate =
        config.lidar_max_rotation_correction_rate();
    policy.lidar_max_velocity_correction_acceleration =
        config.lidar_max_velocity_correction_acceleration();
    policy.lidar_anchor_variance_scale =
        config.lidar_anchor_variance_scale();
    policy.lidar_correction_variance_scale =
        config.lidar_correction_variance_scale();
    policy.lidar_max_translation_correction =
        config.lidar_max_translation_correction();
    policy.lidar_max_rotation_correction =
        config.lidar_max_rotation_correction();
    policy.lidar_max_velocity_correction =
        config.lidar_max_velocity_correction();
    policy.lidar_max_update_iterations =
        config.lidar_max_update_iterations();
    policy.lidar_process_noise_psd_tolerance =
        config.lidar_process_noise_psd_tolerance();
    policy.lidar_rotation_length = config.lidar_rotation_length();
  }
  return policy;
}

GlobalConfig GlobalPolicy(const GlobalEstimatorConfig& config) {
  GlobalConfig policy;
  policy.clock_id = config.clock_id();
  policy.map_id = config.map().map_id();
  policy.map_version = config.map().version();
  policy.calibration_id = config.map().calibration_id();
  policy.history_duration = config.history_duration();
  policy.max_interpolation_gap = config.max_interpolation_gap();
  policy.max_observation_age = config.max_observation_age();
  policy.local_timeout = config.local_timeout();
  policy.global_timeout = config.global_timeout();
  policy.future_tolerance = config.future_tolerance();
  policy.innovation_gate = config.innovation_gate();
  policy.max_position_std = config.max_position_std();
  policy.max_attitude_std = config.max_attitude_std();
  policy.verification_frames = config.verification_frames();
  policy.verification_position = config.verification_position();
  policy.verification_angle = config.verification_angle();
  policy.position_drift_variance_rate = config.position_drift_variance_rate();
  policy.attitude_drift_variance_rate = config.attitude_drift_variance_rate();
  policy.enable_fixed_lag_graph = config.enable_fixed_lag_graph();
  policy.motion_error_model_qualified = config.motion_error_model_qualified();
  policy.graph_solve_budget_ms = config.graph_solve_budget_ms();
  policy.graph_robust_threshold = config.graph_robust_threshold();
  policy.graph_max_states = config.graph_max_states();
  policy.graph_max_observations = config.graph_max_observations();
  policy.graph_max_iterations = config.graph_max_iterations();
  policy.graph_max_linearization_angle = config.graph_max_linearization_angle();
  return policy;
}

Result DecodePose(const LocalizationEstimate& message,
                  Eigen::Isometry3d* pose) {
  if (pose == nullptr || !message.has_measurement_time() ||
      !std::isfinite(message.measurement_time()) ||
      message.measurement_time() <= 0.0 || !message.has_pose() ||
      !message.pose().has_position() || !message.pose().has_orientation()) {
    return {Reason::INVALID_INPUT, "incomplete pose"};
  }
  const auto& position = message.pose().position();
  const auto& q = message.pose().orientation();
  if (!position.has_x() || !position.has_y() || !position.has_z() ||
      !q.has_qw() || !q.has_qx() || !q.has_qy() || !q.has_qz()) {
    return {Reason::INVALID_INPUT, "missing pose coordinates"};
  }
  Eigen::Quaterniond rotation(q.qw(), q.qx(), q.qy(), q.qz());
  Eigen::Vector3d translation(position.x(), position.y(), position.z());
  if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
      std::abs(rotation.norm() - 1.0) > 0.001) {
    return {Reason::INVALID_INPUT, "nonfinite pose/invalid quaternion"};
  }
  *pose = Eigen::Isometry3d::Identity();
  pose->linear() = rotation.normalized().toRotationMatrix();
  pose->translation() = translation;
  return {};
}

void EncodePose(const Eigen::Isometry3d& pose, const Matrix6d& covariance,
                double time, double now, const std::string& frame,
                LocalizationEstimate* message) {
  message->Clear();
  message->mutable_header()->set_timestamp_sec(now);
  message->mutable_header()->set_frame_id(frame);
  message->mutable_header()->set_module_name("localization");
  message->set_measurement_time(time);
  auto* output = message->mutable_pose();
  output->mutable_position()->set_x(pose.translation().x());
  output->mutable_position()->set_y(pose.translation().y());
  output->mutable_position()->set_z(pose.translation().z());
  const Eigen::Quaterniond rotation(pose.linear());
  auto* q = output->mutable_orientation();
  q->set_qw(rotation.w());
  q->set_qx(rotation.x());
  q->set_qy(rotation.y());
  q->set_qz(rotation.z());
  output->set_heading(std::atan2(pose.linear()(1, 0), pose.linear()(0, 0)));
  SetPoint(covariance.topLeftCorner<3, 3>().diagonal().cwiseSqrt(),
           message->mutable_uncertainty()->mutable_position_std_dev());
  const Eigen::Matrix3d quaternion_jacobian =
      0.5 * (rotation.w() * Eigen::Matrix3d::Identity() - Skew(rotation.vec()));
  const Eigen::Matrix3d quaternion_covariance =
      quaternion_jacobian * covariance.bottomRightCorner<3, 3>() *
      quaternion_jacobian.transpose();
  SetPoint(quaternion_covariance.diagonal().cwiseMax(0.0).cwiseSqrt(),
           message->mutable_uncertainty()->mutable_orientation_std_dev());
}

void EncodeLocal(const LocalState& state, const LocalEstimatorConfig& config,
                 double now, LocalOdometry* message) {
  message->Clear();
  EncodeEpoch(state.epoch, message->mutable_epoch());
  message->set_sequence(state.stamp.sequence);
  message->set_frame_id(config.odom_frame());
  message->set_child_frame_id(config.base_frame());
  message->set_clock_id(config.clock_id());
  message->set_publish_time(now);
  message->set_valid_until(std::min(state.stamp.time + config.max_sample_age(),
                                    now + config.max_sample_age()));
  Matrix6d covariance = PoseCovariance(state);
  const Eigen::Matrix3d rotation = state.orientation.toRotationMatrix();
  Matrix6d axes = Matrix6d::Identity();
  axes.bottomRightCorner<3, 3>() = rotation;
  covariance = axes * covariance * axes.transpose();
  EncodePose(Pose(state), covariance, state.stamp.time, now, config.odom_frame(),
             message->mutable_localization());
  auto* pose = message->mutable_localization()->mutable_pose();
  SetPoint(state.velocity, pose->mutable_linear_velocity());
  SetPoint(state.angular_velocity, pose->mutable_angular_velocity_vrf());
  SetPoint(rotation * state.angular_velocity, pose->mutable_angular_velocity());
  SetPoint(state.covariance.block<3, 3>(3, 3).diagonal().cwiseSqrt(),
           message->mutable_localization()->mutable_uncertainty()
               ->mutable_linear_velocity_std_dev());
  for (int row = 0; row < 15; ++row) {
    for (int col = 0; col < 15; ++col) {
      message->add_state_covariance(state.covariance(row, col));
    }
  }
  message->set_valid(state.valid);
  message->set_covariance_model_valid(state.covariance_model_valid);
  if (!state.covariance_model_valid) {
    message->mutable_localization()->clear_uncertainty();
  }
}

Result DecodeLocal(const LocalOdometry& message,
                   const GlobalEstimatorConfig& config, double now,
                   LocalState* state) {
  if (state == nullptr || !message.IsInitialized() || !message.valid() ||
      message.frame_id() != config.odom_frame() ||
      message.child_frame_id() != config.base_frame() ||
      message.clock_id() != config.clock_id() ||
      message.localization().header().frame_id() != config.odom_frame() ||
      message.epoch().producer_session().empty() ||
      message.epoch().generation() == 0 || message.sequence() == 0 ||
      !std::isfinite(now) || !std::isfinite(message.publish_time()) ||
      !std::isfinite(message.valid_until()) || now > message.valid_until() ||
      message.valid_until() < message.localization().measurement_time() ||
      message.publish_time() - now > config.future_tolerance() ||
      now - message.publish_time() > config.local_timeout() ||
      message.state_covariance_size() != 225) {
    return {Reason::INVALID_INPUT, "invalid local envelope/identity"};
  }
  if (now - message.localization().measurement_time() > config.local_timeout() ||
      message.localization().measurement_time() - now > config.future_tolerance()) {
    return {Reason::CLOCK_INVALID, "local measurement age exceeds contract"};
  }
  Eigen::Isometry3d pose;
  auto decoded = DecodePose(message.localization(), &pose);
  if (!decoded.ok()) {
    return decoded;
  }
  state->stamp = {message.localization().measurement_time(), now,
                  message.sequence(), message.clock_id()};
  state->epoch = {message.epoch().producer_session(), message.epoch().generation()};
  state->position = pose.translation();
  state->orientation = Eigen::Quaterniond(pose.linear());
  const auto& dynamics = message.localization().pose();
  if (!dynamics.has_linear_velocity() ||
      !dynamics.linear_velocity().has_x() ||
      !dynamics.linear_velocity().has_y() ||
      !dynamics.linear_velocity().has_z() ||
      !dynamics.has_angular_velocity_vrf() ||
      !dynamics.angular_velocity_vrf().has_x() ||
      !dynamics.angular_velocity_vrf().has_y() ||
      !dynamics.angular_velocity_vrf().has_z()) {
    return {Reason::INVALID_INPUT, "local dynamics incomplete"};
  }
  state->velocity = Eigen::Vector3d(dynamics.linear_velocity().x(),
                                   dynamics.linear_velocity().y(),
                                   dynamics.linear_velocity().z());
  state->angular_velocity = Eigen::Vector3d(
      dynamics.angular_velocity_vrf().x(), dynamics.angular_velocity_vrf().y(),
      dynamics.angular_velocity_vrf().z());
  for (int row = 0; row < 15; ++row) {
    for (int col = 0; col < 15; ++col) {
      state->covariance(row, col) = message.state_covariance(row * 15 + col);
    }
  }
  if (!state->velocity.allFinite() || !state->angular_velocity.allFinite() ||
      !state->covariance.allFinite() ||
      !state->covariance.isApprox(state->covariance.transpose(), 1e-9) ||
      state->covariance.llt().info() != Eigen::Success) {
    return {Reason::INVALID_INPUT, "local covariance/dynamics invalid"};
  }
  state->valid = true;
  state->covariance_model_valid = message.covariance_model_valid();
  return {};
}

void EncodeMotion(const MotionIncrement& motion,
                  const LocalEstimatorConfig& config, double now,
                  LocalMotionIncrement* message) {
  message->Clear();
  message->set_start_sequence(motion.start.sequence);
  message->set_correlation_group(motion.correlation_group);
  message->set_covariance_model(LocalMotionIncrement::JOINT_LOCAL_MARGINAL);
  LocalState start;
  start.covariance = motion.start_covariance;
  Matrix6d axes = Matrix6d::Identity();
  axes.bottomRightCorner<3, 3>() = motion.start_pose.linear();
  const Matrix6d covariance =
      axes * PoseCovariance(start) * axes.transpose();
  EncodePose(motion.start_pose, covariance, motion.start.time, now,
             config.odom_frame(), message->mutable_start_pose());
  for (int row = 0; row < 15; ++row) {
    for (int col = 0; col < 15; ++col) {
      message->add_start_covariance(motion.start_covariance(row, col));
      message->add_cross_covariance(motion.cross_covariance(row, col));
    }
  }
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      message->add_relative_covariance(motion.covariance(row, col));
    }
  }
  for (const auto& source : motion.sources) {
    auto* sample = message->add_source_samples();
    sample->set_source(source.source);
    sample->set_sequence(source.sequence);
  }
}

Result DecodeMotion(const LocalOdometry& message, const LocalState& end,
                    MotionIncrement* motion) {
  if (motion == nullptr || !message.IsInitialized() || !message.has_motion() ||
      !message.covariance_model_valid() || !end.covariance_model_valid ||
      !end.valid || message.sequence() != end.stamp.sequence ||
      message.localization().measurement_time() != end.stamp.time ||
      message.clock_id() != end.stamp.clock_id ||
      message.epoch().producer_session() != end.epoch.session ||
      message.epoch().generation() != end.epoch.generation) {
    return {Reason::INVALID_INPUT, "Missing/mismatched local motion envelope"};
  }
  const auto& input = message.motion();
  if (input.covariance_model() != LocalMotionIncrement::JOINT_LOCAL_MARGINAL ||
      input.start_covariance_size() != 225 ||
      input.cross_covariance_size() != 225 ||
      input.relative_covariance_size() != 36 ||
      input.source_samples_size() < 2 || input.source_samples_size() > 66 ||
      input.start_pose().header().frame_id() != message.frame_id()) {
    return {Reason::INVALID_INPUT, "Invalid motion covariance/provenance shape"};
  }
  Eigen::Isometry3d start_pose;
  const Result decoded = DecodePose(input.start_pose(), &start_pose);
  if (!decoded.ok()) {
    return decoded;
  }
  LocalState start;
  start.valid = true;
  start.epoch = end.epoch;
  start.stamp = {input.start_pose().measurement_time(), end.stamp.receive_time,
                 input.start_sequence(), end.stamp.clock_id};
  start.position = start_pose.translation();
  start.orientation = Eigen::Quaterniond(start_pose.linear());
  Matrix15d cross;
  Matrix6d relative;
  for (int row = 0; row < 15; ++row) {
    for (int col = 0; col < 15; ++col) {
      start.covariance(row, col) = input.start_covariance(row * 15 + col);
      cross(row, col) = input.cross_covariance(row * 15 + col);
    }
  }
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      relative(row, col) = input.relative_covariance(row * 6 + col);
    }
  }
  std::vector<SourceSampleId> sources;
  for (const auto& sample : input.source_samples()) {
    sources.push_back({sample.source(), sample.sequence()});
  }
  MotionIncrement next;
  const Result computed =
      ComputeMotionIncrement(start, end, cross, sources, &next);
  if (!computed.ok()) {
    return computed;
  }
  if (!relative.allFinite() ||
      !relative.isApprox(next.covariance, 1e-8) ||
      input.correlation_group() != next.correlation_group) {
    return {Reason::INVALID_INPUT, "Motion covariance/correlation mismatch"};
  }
  *motion = next;
  return {};
}

Result DecodeObservation(const GlobalObservationMessage& message,
                         const GlobalEstimatorConfig& config, double now,
                         GlobalObservation* observation) {
  if (observation == nullptr || !message.IsInitialized() ||
      message.frame_id() != config.map().frame_id() ||
      message.child_frame_id() != config.base_frame() ||
      (message.has_valid_until() &&
       (!std::isfinite(message.valid_until()) || now > message.valid_until()))) {
    return {Reason::INVALID_INPUT, "invalid global envelope"};
  }
  *observation = GlobalObservation();
  double time = message.measurement_time();
  if (message.kind() == GlobalObservationMessage::POSITION) {
    if (!message.has_measurement_time() || !std::isfinite(time) || time <= 0.0 ||
        !message.has_position() || !message.position().has_x() ||
        !message.position().has_y() || !message.position().has_z() ||
        !message.has_point_in_base() || !message.point_in_base().has_x() ||
        !message.point_in_base().has_y() || !message.point_in_base().has_z() ||
        message.observed_dimension() != 3 || message.observed_covariance_size() != 9 ||
        message.has_localization() || message.projection_size() != 0) {
      return {Reason::INVALID_INPUT, "position-only evidence needs point/reference/covariance"};
    }
    observation->kind = GlobalObservation::Kind::POSITION;
    observation->pose.translation() = Eigen::Vector3d(
        message.position().x(), message.position().y(), message.position().z());
    observation->point_in_base = Eigen::Vector3d(
        message.point_in_base().x(), message.point_in_base().y(), message.point_in_base().z());
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        observation->covariance(row, col) = message.observed_covariance(row * 3 + col);
      }
    }
  } else {
    if (!message.has_localization() ||
        message.localization().header().frame_id() != message.frame_id() ||
        (message.has_measurement_time() &&
         message.measurement_time() != message.localization().measurement_time())) {
      return {Reason::INVALID_INPUT, "pose evidence time/frame mismatch"};
    }
    const auto decoded = DecodePose(message.localization(), &observation->pose);
    if (!decoded.ok()) {
      return decoded;
    }
    time = message.localization().measurement_time();
    if (message.kind() == GlobalObservationMessage::PROJECTED_POSE) {
      const int rank = message.observed_dimension();
      if (rank < 1 || rank > 5 || message.projection_size() != rank * 6 ||
          message.observed_covariance_size() != rank * rank) {
        return {Reason::INVALID_INPUT, "invalid projected-pose dimensions"};
      }
      observation->kind = GlobalObservation::Kind::PROJECTED_POSE;
      observation->projection.resize(rank, 6);
      observation->projected_covariance.resize(rank, rank);
      for (int row = 0; row < rank; ++row) {
        for (int col = 0; col < 6; ++col) {
          observation->projection(row, col) = message.projection(row * 6 + col);
        }
        for (int col = 0; col < rank; ++col) {
          observation->projected_covariance(row, col) =
              message.observed_covariance(row * rank + col);
        }
      }
    } else if (message.kind() == GlobalObservationMessage::POSE &&
               message.covariance_size() == 36) {
      for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
          observation->covariance(row, col) = message.covariance(row * 6 + col);
        }
      }
    } else {
      return {Reason::INVALID_INPUT, "unsupported observation kind/covariance"};
    }
  }
  observation->stamp = {time, now, message.sequence(), message.clock_id()};
  observation->epoch =
      {message.epoch().producer_session(), message.epoch().generation()};
  observation->source = message.source();
  observation->map_id = message.map_id();
  observation->map_version = message.map_version();
  observation->calibration_id = message.calibration_id();
  observation->quality_valid = message.quality_valid();
  observation->georeferenced = message.georeferenced();
  observation->recovery = message.recovery();
  observation->ambiguous = message.ambiguous();
  observation->independent_of_local = message.independent_of_local();
  return {};
}

void EncodeObservation(const GlobalObservation& observation,
                       const GlobalEstimatorConfig& config, double now,
                       GlobalObservationMessage* message) {
  message->Clear();
  EncodeEpoch(observation.epoch, message->mutable_epoch());
  message->set_source(observation.source);
  message->set_sequence(observation.stamp.sequence);
  message->set_clock_id(observation.stamp.clock_id);
  message->set_frame_id(config.map().frame_id());
  message->set_child_frame_id(config.base_frame());
  message->set_map_id(observation.map_id);
  message->set_map_version(observation.map_version);
  message->set_calibration_id(observation.calibration_id);
  message->set_quality_valid(observation.quality_valid);
  message->set_georeferenced(observation.georeferenced);
  message->set_recovery(observation.recovery);
  message->set_ambiguous(observation.ambiguous);
  message->set_independent_of_local(observation.independent_of_local);
  message->set_measurement_time(observation.stamp.time);
  message->set_valid_until(observation.stamp.time + config.global_timeout());
  if (observation.kind == GlobalObservation::Kind::POSITION) {
    message->set_kind(GlobalObservationMessage::POSITION);
    SetPoint(observation.pose.translation(), message->mutable_position());
    SetPoint(observation.point_in_base, message->mutable_point_in_base());
    message->set_observed_dimension(3);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        message->add_observed_covariance(observation.covariance(row, col));
      }
    }
    return;
  }
  EncodePose(observation.pose, observation.covariance, observation.stamp.time,
             now, config.map().frame_id(), message->mutable_localization());
  if (observation.kind == GlobalObservation::Kind::PROJECTED_POSE) {
    message->set_kind(GlobalObservationMessage::PROJECTED_POSE);
    message->mutable_localization()->clear_uncertainty();
    const int rank = observation.projection.rows();
    message->set_observed_dimension(rank);
    for (int row = 0; row < rank; ++row) {
      for (int col = 0; col < 6; ++col) {
        message->add_projection(observation.projection(row, col));
      }
      for (int col = 0; col < rank; ++col) {
        message->add_observed_covariance(observation.projected_covariance(row, col));
      }
    }
  } else {
    message->set_kind(GlobalObservationMessage::POSE);
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        message->add_covariance(observation.covariance(row, col));
      }
    }
  }
}

void EncodeGlobal(const GlobalState& global, const LocalState& local,
                  const GlobalEstimatorConfig& config, double now,
                  GlobalLocalization* message) {
  message->Clear();
  EncodeEpoch(local.epoch, message->mutable_epoch());
  message->set_map_id(config.map().map_id());
  message->set_map_version(config.map().version());
  message->set_calibration_id(config.map().calibration_id());
  message->set_correction_id(global.correction_id);
  message->set_last_observation_time(global.last_full_observation);
  message->set_georeferenced(global.georeferenced &&
      now - global.last_georeference <= config.global_timeout());
  const Matrix6d covariance = GlobalPoseCovarianceBound(global, local);
  EncodePose(global.map_to_odom * Pose(local), covariance, local.stamp.time, now,
             config.map().frame_id(), message->mutable_localization());
  auto* pose = message->mutable_localization()->mutable_pose();
  SetPoint(global.map_to_odom.linear() * local.velocity,
           pose->mutable_linear_velocity());
  SetPoint(local.angular_velocity, pose->mutable_angular_velocity_vrf());
  EncodePose(global.map_to_odom, global.covariance, local.stamp.time, now,
             config.map().frame_id(), message->mutable_map_to_odom());
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      message->add_covariance(covariance(row, col));
      message->add_alignment_covariance(global.covariance(row, col));
    }
  }
}

void EncodeSource(const std::string& name, const SourceCounters& source,
                  double now, double timeout, SourceAssessment* assessment) {
  assessment->set_source(name);
  assessment->set_healthy(source.accepted > 0 &&
                          source.reason == Reason::NONE &&
                          now >= source.last_measurement &&
                          now - source.last_measurement <= timeout);
  assessment->set_last_measurement_time(source.last_measurement);
  assessment->set_last_receive_time(source.last_receive);
  assessment->set_accepted(source.accepted);
  assessment->set_rejected(source.rejected);
  assessment->set_duplicates(source.duplicates);
  assessment->set_regressions(source.regressions);
  assessment->set_evictions(source.evictions);
  assessment->set_reason_code(ReasonName(source.reason));
}

LocalizationAssessment HealthReporter::Evaluate(
    const std::string& owner, const LocalState* local, bool local_valid,
    bool global_valid, bool georeferenced, bool recovery_available,
    Reason reason, Recovery recovery, double now, bool global_owner,
    bool propagation_valid) {
  if (local != nullptr && !local->covariance_model_valid) {
    propagation_valid = propagation_valid || local_valid;
    local_valid = false;
    global_valid = false;
    if (reason == Reason::NONE) {
      reason = Reason::INVALID_INPUT;
    }
  }
  const Epoch epoch = local == nullptr ? Epoch() : local->epoch;
  Availability desired = INVALID;
  if (reason == Reason::WAITING_FOR_STATIONARY ||
      reason == Reason::WAITING_FOR_IMU ||
      (local == nullptr && reason == Reason::HISTORY_UNAVAILABLE)) {
    desired = INITIALIZING;
  } else if (local_valid || propagation_valid) {
    desired = !local_valid || (global_owner && !global_valid) ? DEGRADED : NOMINAL;
  }
  if (desired != state_ || epoch != epoch_ || recovery != recovery_) {
    event_.set_owner(owner);
    event_.set_session_id(epoch.session);
    event_.set_epoch_generation(epoch.generation);
    event_.set_transition_id(++transition_);
    event_.set_publish_time(now);
    event_.set_previous(state_);
    event_.set_current(desired);
    event_.set_primary_reason(ReasonName(reason));
    event_.set_recovery(recovery);
    has_event_ = true;
  }
  state_ = desired;
  recovery_ = recovery;
  epoch_ = epoch;
  LocalizationAssessment assessment;
  assessment.set_owner(owner);
  assessment.set_session_id(epoch.session);
  assessment.set_epoch_generation(epoch.generation);
  assessment.set_sequence(++sequence_);
  assessment.set_measurement_time(local == nullptr ? 0.0 : local->stamp.time);
  assessment.set_publish_time(now);
  assessment.set_availability(state_);
  uint64_t capabilities = 0;
  if (local_valid) {
    capabilities |= LOCAL_POSE_VALID | LOCAL_POSE_CONTINUOUS |
                    VELOCITY_VALID | HEADING_VALID |
                    SHORT_TERM_PREDICTION_VALID;
  }
  if ((local_valid || propagation_valid) && global_valid) {
    capabilities |= GLOBAL_POSE_VALID | MAP_ALIGNED;
    if (georeferenced) {
      capabilities |= GEOREFERENCE_VALID;
    }
  }
  if ((local_valid || propagation_valid) && recovery_available) {
    capabilities |= RELOCALIZATION_AVAILABLE;
  }
  assessment.set_capabilities(capabilities);
  assessment.set_primary_reason(ReasonName(reason));
  const bool covariance_valid =
      local != nullptr && local->covariance_model_valid &&
      ValidCovariance(PoseCovariance(*local));
  assessment.set_covariance_valid(covariance_valid);
  if (covariance_valid) {
    assessment.set_position_std(
        std::sqrt(local->covariance.block<3, 3>(0, 0).diagonal().maxCoeff()));
    assessment.set_attitude_std(
        std::sqrt(local->covariance.block<3, 3>(6, 6).diagonal().maxCoeff()));
  }
  assessment.set_propagation_valid(local_valid || propagation_valid);
  assessment.set_output_continuous(local_valid || propagation_valid);
  assessment.set_map_alignment_valid(global_valid && (local_valid || propagation_valid));
  // Lane containment needs independent road evidence, not RTK FIX or NDT score.
  assessment.set_lane_level_valid(false);
  assessment.set_recovery(recovery);
  assessment.set_transition_id(transition_);
  return assessment;
}

bool HealthReporter::TakeEvent(LocalizationHealthEvent* event) {
  if (!has_event_ || event == nullptr) {
    return false;
  }
  *event = event_;
  has_event_ = false;
  return true;
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
