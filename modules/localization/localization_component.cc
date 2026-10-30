// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "modules/localization/localization_component.h"

#include <memory>
#include <stdexcept>

#include "cyber/time/clock.h"
#include "modules/localization/localization_config.h"
#include "modules/transform/buffer.h"

namespace apollo::localization::openloc {
namespace {

template <typename Message>
auto Callback(const std::shared_ptr<LocalizationRuntime>& runtime,
              void (LocalizationRuntime::*method)(const Message&, double)) {
  return [runtime, method](const std::shared_ptr<Message>& message) {
    try {
      (runtime.get()->*method)(*message, cyber::Clock::NowInSeconds());
    } catch (const std::exception& error) {
      runtime->Invalidate(error.what());
      AERROR << "OpenLoc input stopped: " << error.what();
    }
  };
}

}  // namespace

OpenLocLocalizationComponent::~OpenLocLocalizationComponent() {
  if (imu_) imu_->Shutdown();
  if (lidar_) lidar_->Shutdown();
  if (gnss_) gnss_->Shutdown();
  if (heading_) heading_->Shutdown();
}

bool OpenLocLocalizationComponent::Init() {
  try {
    const auto deployment = LoadDeploymentConfiguration(ConfigFilePath());
    if (!deployment.deployment_configured()) {
      throw std::invalid_argument("localization deployment is not configured");
    }
    common::vehicle_calibration::Registry registry;
    if (!registry.LoadFromEnvironment()) {
      throw std::invalid_argument(
          "cannot load selected common vehicle calibration");
    }
    const transform::TransformQuery query(transform::Buffer::Instance());
    const auto config = ResolveConfiguration(deployment, registry, query);
    runtime_ = std::make_shared<LocalizationRuntime>(config);
    pose_ =
        node_->CreateWriter<LocalizationEstimate>(config.localization_topic());
    status_ = node_->CreateWriter<LocalizationStatus>(config.status_topic());
    if (!pose_ || !status_)
      throw std::runtime_error("cannot create output writers");
    broadcaster_ = std::make_unique<transform::TransformBroadcaster>(node_);
    cyber::ReaderConfig reader;
    reader.channel_name = config.local().imu_topic();
    reader.pending_queue_size = 256;
    imu_ = node_->CreateReader<drivers::gnss::Imu>(
        reader, Callback(runtime_, &LocalizationRuntime::AddImu));
    reader.channel_name = config.local().lidar_topic();
    reader.pending_queue_size = 2;
    lidar_ = node_->CreateReader<drivers::PointCloud>(
        reader, Callback(runtime_, &LocalizationRuntime::AddCloud));
    reader.channel_name = config.global().gnss_topic();
    reader.pending_queue_size = 128;
    gnss_ = node_->CreateReader<drivers::gnss::GnssBestPose>(
        reader, Callback(runtime_, &LocalizationRuntime::AddBestPose));
    if (config.global().structured_gnss().has_heading()) {
      reader.channel_name = config.global().structured_gnss().heading().topic();
      heading_ = node_->CreateReader<drivers::gnss::Heading>(
          reader, Callback(runtime_, &LocalizationRuntime::AddHeading));
      if (!heading_) throw std::runtime_error("cannot create heading reader");
    }
    if (!imu_ || !lidar_ || !gnss_) {
      throw std::runtime_error("cannot create sensor readers");
    }
    AWARN << "OpenLoc absolute output is diagnostic-only, not qualified for "
             "driving";
    return true;
  } catch (const std::exception& error) {
    AERROR << "OpenLoc initialization failed: " << error.what();
    return false;
  }
}

bool OpenLocLocalizationComponent::Proc() {
  const auto output =
      runtime_->Sample(cyber::Clock::NowInSeconds(), ++sequence_);
  if (!status_->Write(output.status)) {
    runtime_->Invalidate("localization status publication failed");
    return false;
  }
  if (output.pose) {
    if (!pose_->Write(*output.pose)) {
      runtime_->Invalidate("localization pose publication failed");
      return false;
    }
    broadcaster_->SendTransform(*output.transform);
  }
  return true;
}

}  // namespace apollo::localization::openloc
