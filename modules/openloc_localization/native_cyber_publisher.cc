// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_cyber_publisher.h"

#include <stdexcept>

#include "cyber/time/clock.h"

namespace apollo::localization::openloc_diagnostic {

NativeCyberPublisher::NativeCyberPublisher(
    const std::shared_ptr<cyber::Node>& node, double maximum_age,
    bool recorded_input)
    : publication_(maximum_age),
      recorded_input_(recorded_input),
      timer_(
          10, [this] { Publish(); }, false) {
  if (!node) throw std::invalid_argument("diagnostic publisher needs a node");
  odometry_ = node->CreateWriter<NativeOdometryDiagnostic>(
      "/openloc/diagnostic/odometry");
  transforms_ = node->CreateWriter<transform::TransformStampeds>(
      "/openloc/diagnostic/tf");
  if (!odometry_ || !transforms_)
    throw std::runtime_error("cannot create diagnostic Cyber writers");
  timer_.Start();
}

NativeCyberPublisher::~NativeCyberPublisher() { timer_.Stop(); }

void NativeCyberPublisher::Update(
    const NativePublication::State& state,
    const NativePublication::Result& integrity,
    NativePublication::Clock::time_point arrival) {
  Check();
  publication_.Update(state, integrity, arrival);
}

void NativeCyberPublisher::Check() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  if (error_) std::rethrow_exception(error_);
}

void NativeCyberPublisher::Invalidate(const NativePublication::Result& failure) {
  {
    std::lock_guard<std::mutex> serial(publication_mutex_);
    Check();
    publication_.Invalidate(failure);
  }
  Publish();
  Check();
}

void NativeCyberPublisher::Finish() {
  timer_.Stop();
  Check();
}

void NativeCyberPublisher::Publish() {
  try {
    std::lock_guard<std::mutex> serial(publication_mutex_);
    Check();
    auto output = publication_.Sample(cyber::Clock::NowInSeconds());
    output.set_recorded_input(recorded_input_);
    if (!odometry_->Write(output))
      throw std::runtime_error("diagnostic ODOM transport rejected submission");
    if (output.usable() && output.new_measurement()) {
      transform::TransformStampeds transforms;
      transforms.mutable_header()->set_timestamp_sec(output.measurement_time());
      *transforms.add_transforms() = output.odom_from_imu();
      if (!transforms_->Write(transforms))
        throw std::runtime_error("diagnostic TF transport rejected submission");
    }
  } catch (...) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    if (!error_) {
      error_ = std::current_exception();
      AERROR << "OpenLoc diagnostic publication stopped by an explicit error";
    }
  }
}

}  // namespace apollo::localization::openloc_diagnostic
