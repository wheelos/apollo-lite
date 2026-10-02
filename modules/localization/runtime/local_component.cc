// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/runtime/local_component.h"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/localization/common/rigid_transform_helper.h"

namespace apollo {
namespace localization {
namespace unified {

LocalLocalizationComponent::~LocalLocalizationComponent() {
  if (health_timer_) {
    health_timer_->Stop();
  }
}

bool LocalLocalizationComponent::Init() {
  if (!GetProtoConfig(&config_) || !config_.IsInitialized() ||
      config_.odom_frame().empty() || config_.base_frame().empty() ||
      config_.imu_frame().empty() ||
      config_.odom_frame() == config_.base_frame() ||
      !std::isfinite(config_.imu_to_unix_offset()) ||
      !std::isfinite(config_.wheel_variance()) ||
      config_.wheel_variance() <= 0.0 ||
      !std::isfinite(config_.health_period_ms()) ||
      config_.health_period_ms() <= 0.0 ||
      config_.imu_topic().empty() || config_.chassis_topic().empty() ||
      config_.odometry_topic().empty() || config_.assessment_topic().empty() ||
      config_.event_topic().empty() || config_.reset_topic().empty()) {
    AERROR << "CONFIG_INVALID: incomplete local estimator configuration";
    return false;
  }
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::string session =
      node_->Name() + ":" + std::to_string(getpid()) + ":" + std::to_string(ticks);
  estimator_ = std::make_unique<LocalEstimator>(LocalPolicy(config_, session));
  const auto valid = estimator_->ValidateConfig();
  if (!valid.ok()) {
    AERROR << ReasonName(valid.reason) << ": " << valid.message;
    return false;
  }
  Eigen::Affine3d extrinsic;
  if (!apollo::localization::common::LookupStaticTransform(
          config_.base_frame(), config_.imu_frame(), &extrinsic, 3.0f)) {
    AERROR << "CONFIG_INVALID: missing static base-to-IMU calibration";
    return false;
  }
  base_from_imu_.matrix() = extrinsic.matrix();
  if (!base_from_imu_.matrix().allFinite() ||
      !(base_from_imu_.linear().transpose() * base_from_imu_.linear())
           .isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
      std::abs(base_from_imu_.linear().determinant() - 1.0) > 1e-6) {
    AERROR << "CONFIG_INVALID: invalid IMU extrinsic";
    return false;
  }
  if (config_.enable_lidar()) {
    const bool complete =
        config_.has_lidar_topic() && !config_.lidar_topic().empty() &&
        config_.has_lidar_frame() && !config_.lidar_frame().empty() &&
        config_.has_lidar_calibration_id() &&
        !config_.lidar_calibration_id().empty() &&
        config_.has_lidar_scan_time_offset() &&
        config_.has_lidar_point_time_offset() &&
        config_.has_lidar_point_noise_std() &&
        config_.has_lidar_plane_noise_std() &&
        config_.has_lidar_voxel_size() &&
        config_.has_lidar_max_voxels() &&
        config_.has_lidar_max_scans() &&
        config_.has_lidar_min_points_per_plane() &&
        config_.has_lidar_min_correspondences() &&
        config_.has_lidar_max_points() &&
        config_.has_lidar_max_correspondence_distance() &&
        config_.has_lidar_observed_singular_value() &&
        config_.has_lidar_innovation_gate() &&
        config_.has_lidar_max_scan_age() &&
        config_.has_lidar_future_tolerance() &&
        config_.has_lidar_time_tolerance() &&
        config_.has_lidar_max_translation_correction_rate() &&
        config_.has_lidar_max_rotation_correction_rate() &&
        config_.has_lidar_max_velocity_correction_acceleration() &&
        config_.has_lidar_anchor_variance_scale() &&
        config_.has_lidar_correction_variance_scale() &&
        config_.has_lidar_max_buffered_scans() &&
        config_.lidar_max_buffered_scans() > 0 &&
        config_.has_lidar_max_translation_correction() &&
        config_.has_lidar_max_rotation_correction() &&
        config_.has_lidar_max_velocity_correction() &&
        config_.has_lidar_max_update_iterations() &&
        config_.lidar_max_update_iterations() > 0 &&
        config_.has_lidar_input_is_raw() &&
        config_.lidar_input_is_raw() &&
        config_.has_lidar_point_time_is_unix_nanoseconds() &&
        config_.lidar_point_time_is_unix_nanoseconds() &&
        config_.has_lidar_process_noise_psd_tolerance() &&
        config_.has_lidar_rotation_length();
    if (!complete ||
        !std::isfinite(config_.lidar_scan_time_offset()) ||
        !std::isfinite(config_.lidar_point_time_offset())) {
      AERROR << "CONFIG_INVALID: LiDAR requires explicit frame/time/noise/"
                "storage/dynamics calibration";
      return false;
    }
    Eigen::Affine3d lidar_extrinsic;
    if (!apollo::localization::common::LookupStaticTransform(
            config_.base_frame(), config_.lidar_frame(),
            &lidar_extrinsic, 3.0f)) {
      AERROR << "CONFIG_INVALID: missing static base-to-LiDAR calibration";
      return false;
    }
    base_from_lidar_.matrix() = lidar_extrinsic.matrix();
    if (!base_from_lidar_.matrix().allFinite() ||
        !(base_from_lidar_.linear().transpose() *
          base_from_lidar_.linear())
             .isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
        std::abs(base_from_lidar_.linear().determinant() - 1.0) > 1e-6) {
      AERROR << "CONFIG_INVALID: invalid LiDAR extrinsic";
      return false;
    }
  }
  odometry_writer_ = node_->CreateWriter<LocalOdometry>(config_.odometry_topic());
  assessment_writer_ =
      node_->CreateWriter<LocalizationAssessment>(config_.assessment_topic());
  event_writer_ =
      node_->CreateWriter<LocalizationHealthEvent>(config_.event_topic());
  chassis_reader_ = node_->CreateReader<canbus::Chassis>(
      config_.chassis_topic(), [this](const auto& msg) { OnChassis(msg); });
  if (config_.enable_lidar()) {
    lidar_reader_ = node_->CreateReader<drivers::PointCloud>(
        config_.lidar_topic(), [this](const auto& msg) { OnLidar(msg); });
  }
  reset_reader_ = node_->CreateReader<OdomResetRequest>(
      config_.reset_topic(), [this](const auto& msg) { OnReset(msg); });
  if (!odometry_writer_ || !assessment_writer_ || !event_writer_ ||
      !chassis_reader_ || !reset_reader_ ||
      (config_.enable_lidar() && !lidar_reader_)) {
    AERROR << "FAULT: failed to initialize local localization IO";
    return false;
  }
  broadcaster_ = std::make_unique<transform::TransformBroadcaster>(node_);
  health_timer_ = std::make_unique<cyber::Timer>(
      config_.health_period_ms(), [this]() { PublishHealth(); }, false);
  health_timer_->Start();
  return true;
}

void LocalLocalizationComponent::OnLidar(
    const std::shared_ptr<drivers::PointCloud>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = cyber::Clock::NowInSeconds();
  if (!message || !message->has_measurement_time() ||
      !std::isfinite(message->measurement_time()) ||
      message->point_size() == 0 ||
      message->point_size() > static_cast<int>(config_.lidar_max_points()) ||
      apollo::localization::common::GetPointCloudFrameId(*message) !=
          config_.lidar_frame()) {
    RejectAdapter({Reason::INVALID_INPUT,
                   "invalid local raw LiDAR cloud/frame/time"}, now);
    return;
  }
  LidarScan scan;
  scan.stamp = {
      message->measurement_time() + config_.lidar_scan_time_offset(),
      now, ++lidar_sequence_, config_.clock_id()};
  scan.epoch = estimator_->state().epoch;
  scan.frame_id = config_.lidar_frame();
  scan.calibration_id = config_.lidar_calibration_id();
  scan.base_from_lidar = base_from_lidar_;
  scan.points.reserve(message->point_size());
  for (const auto& point : message->point()) {
    if (!point.has_x() || !point.has_y() || !point.has_z() ||
        !point.has_timestamp() || point.timestamp() == 0 ||
        !std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z())) {
      RejectAdapter({point.has_timestamp() && point.timestamp() != 0
                         ? Reason::INVALID_INPUT
                         : Reason::POINT_TIME_MISSING,
                     "raw local LiDAR needs finite points and per-point time"},
                    now);
      return;
    }
    TimedLidarPoint raw;
    raw.position = Eigen::Vector3d(point.x(), point.y(), point.z());
    raw.time =
        point.timestamp() * 1e-9 + config_.lidar_point_time_offset();
    scan.points.push_back(raw);
  }
  SubmitLidar(std::move(scan));
}

void LocalLocalizationComponent::SubmitLidar(LidarScan scan) {
  const double frontier = estimator_->internal_state().stamp.time;
  if (scan.stamp.time > frontier) {
    if (pending_lidar_.size() >= config_.lidar_max_buffered_scans()) {
      RejectAdapter({Reason::INVALID_INPUT,
                     "bounded future LiDAR queue is full"},
                    scan.stamp.receive_time);
      return;
    }
    if (!pending_lidar_.empty() &&
        scan.stamp.time <= pending_lidar_.back().stamp.time) {
      RejectAdapter(
          {scan.stamp.time == pending_lidar_.back().stamp.time
               ? Reason::DUPLICATE
               : Reason::TIMESTAMP_REGRESSION,
           "future LiDAR scans must arrive in strict measurement order"},
          scan.stamp.receive_time);
      return;
    }
    pending_lidar_.push_back(std::move(scan));
    return;
  }
  if (scan.stamp.time < frontier) {
    const Result result = estimator_->AddLidar(scan);
    if (!result.ok()) {
      AERROR << ReasonName(result.reason) << ": " << result.message;
    }
    return;
  }
  const Result result = estimator_->AddLidar(scan);
  if (!result.ok() && result.reason != Reason::GEOMETRY_UNAVAILABLE) {
    AERROR << ReasonName(result.reason) << ": " << result.message;
  }
}

void LocalLocalizationComponent::ProcessPendingLidar() {
  const double frontier = estimator_->internal_state().stamp.time;
  while (!pending_lidar_.empty() &&
         pending_lidar_.front().stamp.time <= frontier) {
    LidarScan scan = std::move(pending_lidar_.front());
    pending_lidar_.pop_front();
    const Result result = estimator_->AddLidar(scan);
    if (!result.ok() && result.reason != Reason::GEOMETRY_UNAVAILABLE) {
      AERROR << ReasonName(result.reason) << ": " << result.message;
    }
  }
}

void LocalLocalizationComponent::RejectAdapter(const Result& result,
                                                double now) {
  adapter_reason_ = result.reason;
  adapter_health_.reason = result.reason;
  adapter_health_.last_receive = now;
  ++adapter_health_.rejected;
  AERROR << ReasonName(result.reason) << ": " << result.message;
}

bool LocalLocalizationComponent::Proc(
    const std::shared_ptr<drivers::gnss::Imu>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = cyber::Clock::NowInSeconds();
  if (!message || !message->has_measurement_time() ||
      !message->has_linear_acceleration() || !message->has_angular_velocity() ||
      !message->linear_acceleration().has_x() ||
      !message->linear_acceleration().has_y() ||
      !message->linear_acceleration().has_z() ||
      !message->angular_velocity().has_x() ||
      !message->angular_velocity().has_y() ||
      !message->angular_velocity().has_z()) {
    RejectAdapter({Reason::INVALID_INPUT, "incomplete raw IMU"}, now);
    return false;
  }
  ImuSample sample;
  sample.stamp = {message->measurement_time() + config_.imu_to_unix_offset(),
                  now, ++imu_sequence_, config_.clock_id()};
  const auto& a = message->linear_acceleration();
  const auto& w = message->angular_velocity();
  Eigen::Vector3d acceleration(a.x(), a.y(), a.z());
  Eigen::Vector3d angular(w.x(), w.y(), w.z());
  if (config_.imu_rfu()) {
    acceleration = Eigen::Vector3d(a.y(), -a.x(), a.z());
    angular = Eigen::Vector3d(w.y(), -w.x(), w.z());
  }
  angular = base_from_imu_.linear() * angular;
  acceleration = base_from_imu_.linear() * acceleration;
  const double dt = sample.stamp.time - previous_imu_time_;
  Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();
  if (previous_imu_time_ > 0.0 && dt > 0.0 && dt <= config_.max_imu_gap()) {
    angular_acceleration = (angular - previous_angular_) / dt;
  } else if (base_from_imu_.translation().norm() > 1e-6 &&
             angular.norm() > config_.stationary_gyro()) {
    RejectAdapter({Reason::INVALID_INPUT,
                   "IMU lever-arm correction needs causal angular history"}, now);
    return false;
  }
  const Eigen::Vector3d corrected_angular =
      angular - estimator_->internal_state().gyro_bias;
  sample.angular_velocity = angular;
  sample.acceleration =
      acceleration - angular_acceleration.cross(base_from_imu_.translation()) -
      corrected_angular.cross(
          corrected_angular.cross(base_from_imu_.translation()));
  const auto result = estimator_->AddImu(sample);
  if (!result.ok()) {
    if (result.reason != Reason::WAITING_FOR_STATIONARY &&
        result.reason != Reason::WAITING_FOR_IMU) {
      AERROR << ReasonName(result.reason) << ": " << result.message;
    }
    // A rejected wheel update or exceeded precision budget can accompany a
    // successfully committed IMU propagation. Never replay the previous state.
    const auto& committed = estimator_->state();
    if (!committed.valid || committed.stamp.time != sample.stamp.time ||
        committed.stamp.sequence != sample.stamp.sequence) {
      return false;
    }
  }
  previous_imu_time_ = sample.stamp.time;
  previous_angular_ = angular;
  if (config_.enable_lidar()) {
    ProcessPendingLidar();
  }
  adapter_reason_ = Reason::NONE;
  adapter_health_.reason = Reason::NONE;
  adapter_health_.last_measurement = sample.stamp.time;
  adapter_health_.last_receive = now;
  ++adapter_health_.accepted;
  const auto integrity = estimator_->EvaluateIntegrity(now);
  if (!integrity.ok()) {
    if (integrity.reason == Reason::WAITING_FOR_STATIONARY ||
        integrity.reason == Reason::WAITING_FOR_IMU) {
      return true;
    }
    AERROR << ReasonName(integrity.reason) << ": " << integrity.message;
    return false;
  }
  LocalOdometry odometry;
  EncodeLocal(estimator_->state(), config_, now, &odometry);
  const auto* motion = estimator_->last_motion();
  if (motion != nullptr) {
    EncodeMotion(*motion, config_, now, odometry.mutable_motion());
  } else if (estimator_->motion_status().reason != Reason::HISTORY_UNAVAILABLE) {
    // Absence is fail-closed eligibility, not a request to synthesize an
    // independent relative covariance from the two published endpoints.
    const auto& status = estimator_->motion_status();
    AERROR << ReasonName(status.reason) << ": " << status.message;
  }
  const auto precision = estimator_->Evaluate(now);
  odometry.set_precision_valid(precision.ok());
  odometry.set_degradation_reason(ReasonName(precision.reason));
  if (!odometry_writer_->Write(odometry)) {
    RejectAdapter({Reason::INVALID_INPUT, "local odometry write failed"}, now);
    return false;
  }
  const auto& estimate = odometry.localization();
  transform::TransformStamped tf;
  tf.mutable_header()->set_timestamp_sec(estimate.measurement_time());
  tf.mutable_header()->set_frame_id(config_.odom_frame());
  tf.set_child_frame_id(config_.base_frame());
  const auto& position = estimate.pose().position();
  auto* translation = tf.mutable_transform()->mutable_translation();
  translation->set_x(position.x());
  translation->set_y(position.y());
  translation->set_z(position.z());
  *tf.mutable_transform()->mutable_rotation() = estimate.pose().orientation();
  broadcaster_->SendTransform(tf);
  return true;
}

void LocalLocalizationComponent::OnChassis(
    const std::shared_ptr<canbus::Chassis>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = cyber::Clock::NowInSeconds();
  if (!message || !message->has_header() ||
      !message->header().has_timestamp_sec() || !message->has_speed_mps() ||
      !std::isfinite(message->speed_mps()) ||
      (message->has_error_code() &&
       message->error_code() != canbus::Chassis::NO_ERROR)) {
    RejectAdapter({Reason::INVALID_INPUT, "invalid chassis velocity/time"}, now);
    return;
  }
  double speed = message->speed_mps();
  if (!config_.signed_chassis_speed()) {
    if (speed < 0.0 || !message->has_gear_location() ||
        message->gear_location() == canbus::Chassis::GEAR_INVALID ||
        message->gear_location() == canbus::Chassis::GEAR_NONE ||
        (std::abs(speed) > config_.stationary_speed() &&
         message->gear_location() != canbus::Chassis::GEAR_DRIVE &&
         message->gear_location() != canbus::Chassis::GEAR_LOW &&
         message->gear_location() != canbus::Chassis::GEAR_REVERSE)) {
      RejectAdapter({Reason::INVALID_INPUT, "unsigned wheel speed needs valid gear"}, now);
      return;
    }
    if (message->gear_location() == canbus::Chassis::GEAR_REVERSE) {
      speed = -speed;
    }
  }
  WheelSample sample;
  sample.stamp = {message->header().timestamp_sec(), now, ++wheel_sequence_,
                  config_.clock_id()};
  sample.speed = speed;
  sample.variance = config_.wheel_variance();
  const auto result = estimator_->AddWheel(sample);
  if (!result.ok()) {
    AERROR << ReasonName(result.reason) << ": " << result.message;
  }
}

void LocalLocalizationComponent::OnReset(
    const std::shared_ptr<OdomResetRequest>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = cyber::Clock::NowInSeconds();
  const auto& epoch = estimator_->state().epoch;
  if (!message || !message->IsInitialized() ||
      message->current_epoch().producer_session() != epoch.session ||
      message->current_epoch().generation() != epoch.generation ||
      !std::isfinite(message->request_time()) ||
      now - message->request_time() > config_.max_sample_age() ||
      message->request_time() - now > config_.future_tolerance()) {
    RejectAdapter({Reason::EPOCH_MISMATCH, "invalid/stale reset request"}, now);
    return;
  }
  const auto result = estimator_->ResetStopped(now);
  if (!result.ok()) {
    RejectAdapter(result, now);
  } else {
    previous_imu_time_ = 0.0;
    previous_angular_.setZero();
    pending_lidar_.clear();
  }
}

void LocalLocalizationComponent::PublishHealth() {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = cyber::Clock::NowInSeconds();
  const auto evaluated = estimator_->Evaluate(now);
  const Reason reason = !evaluated.ok() ? evaluated.reason : adapter_reason_;
  const bool propagation_valid = estimator_->EvaluateIntegrity(now).ok();
  auto assessment = health_.Evaluate(
      "local", &estimator_->state(), evaluated.ok(), false, false, false,
      reason, IDLE, now, false, propagation_valid);
  EncodeSource("imu", estimator_->imu_health(), now, config_.max_sample_age(),
               assessment.add_sources());
  EncodeSource("wheel", estimator_->wheel_health(), now, config_.wheel_timeout(),
               assessment.add_sources());
  if (config_.enable_lidar()) {
    EncodeSource("lidar", estimator_->lidar_health(), now,
                 config_.lidar_max_scan_age(), assessment.add_sources());
  }
  EncodeSource("input_adapter", adapter_health_, now, config_.max_sample_age(),
               assessment.add_sources());
  if (!assessment_writer_->Write(assessment)) {
    AERROR << "FAULT: local health write failed";
  }
  LocalizationHealthEvent event;
  if (health_.TakeEvent(&event) && !event_writer_->Write(event)) {
    AERROR << "FAULT: local health event write failed";
  }
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
