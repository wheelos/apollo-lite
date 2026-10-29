// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_publication.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace apollo::localization::openloc_diagnostic {
namespace u = ::openloc::localization::unified;
namespace {

bool ValidSession(const std::string& session) {
  return !session.empty() &&
         std::all_of(session.begin(), session.end(), [](unsigned char c) {
           return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '-';
         });
}

}  // namespace

NativePublication::NativePublication(double maximum_age, bool recorded_source_clock)
    : maximum_age_(maximum_age), recorded_source_clock_(recorded_source_clock) {
  if (!std::isfinite(maximum_age) || maximum_age <= 0 || maximum_age > 1)
    throw std::invalid_argument("publication age must be in (0,1] seconds");
}

void NativePublication::Update(const State& state, const Result& integrity,
                               Clock::time_point arrival) {
  if (!std::isfinite(state.stamp.time) || state.stamp.time <= 0 ||
      !std::isfinite(state.stamp.receive_time) ||
      state.stamp.receive_time <= 0 || state.stamp.clock_id != "unix" ||
      state.stamp.sequence == 0 || arrival > Clock::now() ||
      (state.valid &&
       (!ValidSession(state.epoch.session) || state.epoch.generation == 0 ||
        !state.position.allFinite() || !state.velocity.allFinite() ||
        !state.gyro_bias.allFinite() || !state.accel_bias.allFinite() ||
        !state.angular_velocity.allFinite() || !state.covariance.allFinite() ||
        !state.orientation.coeffs().allFinite() ||
        std::abs(state.orientation.norm() - 1.0) > 1e-9)))
    throw std::invalid_argument("invalid native publication snapshot");
  const auto next =
      std::make_shared<const Snapshot>(Snapshot{state, integrity, arrival});
  std::lock_guard<std::mutex> lock(mutex_);
  if (terminal_failure_)
    throw std::runtime_error("native publication is terminally invalidated");
  if (latest_ && latest_->state.epoch == state.epoch &&
      (state.stamp.time <= latest_->state.stamp.time ||
       state.stamp.sequence <= latest_->state.stamp.sequence ||
       state.stamp.receive_time < latest_->state.stamp.receive_time ||
       arrival < latest_->arrival))
    throw std::invalid_argument("native publication frontier regressed");
  if (latest_ && state.epoch.session == latest_->state.epoch.session &&
      state.epoch.generation < latest_->state.epoch.generation)
    throw std::invalid_argument("native publication epoch regressed");
  latest_ = next;
}

void NativePublication::Invalidate(const Result& failure) {
  if (failure.ok() || failure.message.empty())
    throw std::invalid_argument("publication invalidation needs an explicit failure");
  std::lock_guard<std::mutex> lock(mutex_);
  if (!terminal_failure_) terminal_failure_ = failure;
}

NativeOdometryDiagnostic NativePublication::Sample(
    double publication_time, std::optional<Clock::time_point> evaluation_time) {
  if (!std::isfinite(publication_time) || publication_time <= 0)
    throw std::invalid_argument("invalid publication clock");
  std::shared_ptr<const Snapshot> snapshot;
  std::optional<Result> failure;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot = latest_;
    failure = terminal_failure_;
  }
  // Capture the clock after the handoff, so a concurrent Update cannot make
  // the selected snapshot appear to have arrived in the future.
  const auto now = evaluation_time.value_or(Clock::now());
  if (last_publication_ && now < *last_publication_)
    throw std::invalid_argument("publication steady clock regressed");
  NativeOdometryDiagnostic output;
  output.set_publication_sequence(++publications_);
  output.set_publication_time(publication_time);
  output.set_publication_steady_ns(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          now.time_since_epoch())
          .count());
  output.set_publication_interval_ns(
      last_publication_ ? std::chrono::duration_cast<std::chrono::nanoseconds>(
                              now - *last_publication_)
                              .count()
                        : 0);
  output.set_requested_period_ns(10000000);
  last_publication_ = now;
  output.set_maximum_age(maximum_age_);
  output.set_diagnostic_only(true);
  output.set_trusted(false);
  if (recorded_source_clock_) {
    output.set_recorded_source_clock(true);
    output.set_recorded_input(true);
    output.set_processing_delay_s(0);
  }
  output.set_usable(false);
  output.set_new_measurement(false);
  output.set_epoch_changed(false);
  if (!snapshot) {
    output.set_reason(failure ? u::ReasonName(failure->reason) : "WAITING_FOR_IMU");
    output.set_detail(failure ? failure->message : "No native snapshot has arrived");
    return output;
  }
  const auto& state = snapshot->state;
  if (now < snapshot->arrival)
    throw std::invalid_argument("publication precedes snapshot arrival");
  const double delay = std::chrono::duration<double>(now - snapshot->arrival).count();
  if (recorded_source_clock_) output.set_processing_delay_s(delay);
  const double age =
      std::max(0.0, state.stamp.receive_time - state.stamp.time) +
      (recorded_source_clock_ ? 0 : delay);
  output.set_measurement_time(state.stamp.time);
  output.set_receive_time(state.stamp.receive_time);
  output.set_source_sequence(state.stamp.sequence);
  output.set_session(state.epoch.session);
  output.set_generation(state.epoch.generation);
  output.set_measurement_age(age);
  if (failure) {
    output.set_reason(u::ReasonName(failure->reason));
    output.set_detail(failure->message);
    return output;
  }
  if (!state.valid || !snapshot->integrity.ok() || age > maximum_age_) {
    output.set_reason(age > maximum_age_
                          ? "IMU_STALE"
                          : u::ReasonName(snapshot->integrity.reason));
    if (!state.valid && snapshot->integrity.ok())
      output.set_reason("WAITING_FOR_IMU");
    output.set_detail(age > maximum_age_ ? "Source snapshot exceeded freshness"
                                         : snapshot->integrity.message);
    return output;
  }
  const bool epoch_changed =
      last_emitted_ && state.epoch != last_emitted_->state.epoch;
  output.set_usable(true);
  output.set_epoch_changed(epoch_changed);
  output.set_new_measurement(!last_emitted_ || epoch_changed ||
                             state.stamp.sequence !=
                                 last_emitted_->state.stamp.sequence);
  output.set_reason("UNCERTAINTY_UNQUALIFIED");
  output.set_detail("Diagnostic local IMU-origin ODOM; not ENU/VRP or trusted");
  const std::string suffix =
      state.epoch.session + "/" + std::to_string(state.epoch.generation);
  auto* transform = output.mutable_odom_from_imu();
  transform->mutable_header()->set_timestamp_sec(state.stamp.time);
  transform->mutable_header()->set_frame_id("openloc_diag/odom/" + suffix);
  transform->set_child_frame_id("openloc_diag/imu/" + suffix);
  auto* translation = transform->mutable_transform()->mutable_translation();
  translation->set_x(state.position.x());
  translation->set_y(state.position.y());
  translation->set_z(state.position.z());
  auto* rotation = transform->mutable_transform()->mutable_rotation();
  rotation->set_qx(state.orientation.x());
  rotation->set_qy(state.orientation.y());
  rotation->set_qz(state.orientation.z());
  rotation->set_qw(state.orientation.w());
  auto* velocity = output.mutable_velocity_in_odom();
  velocity->set_x(state.velocity.x());
  velocity->set_y(state.velocity.y());
  velocity->set_z(state.velocity.z());
  auto* error = output.mutable_native_error_state();
  error->set_covariance_model_valid(state.covariance_model_valid);
  error->set_uncertainty_qualified(state.uncertainty_qualified);
  error->mutable_covariance()->Reserve(225);
  for (int row = 0; row < 15; ++row)
    for (int column = 0; column < 15; ++column)
      error->add_covariance(state.covariance(row, column));
  const auto set_vector = [](const Eigen::Vector3d& source,
                             apollo::common::Point3D* destination) {
    destination->set_x(source.x());
    destination->set_y(source.y());
    destination->set_z(source.z());
  };
  set_vector(state.gyro_bias, error->mutable_gyro_bias());
  set_vector(state.accel_bias, error->mutable_accel_bias());
  set_vector(state.angular_velocity, error->mutable_angular_velocity());
  last_emitted_ = snapshot;
  return output;
}

}  // namespace apollo::localization::openloc_diagnostic
