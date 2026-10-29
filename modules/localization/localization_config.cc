// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "modules/localization/localization_config.h"

#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <stdexcept>
#include <string>

#include "google/protobuf/text_format.h"

#include "modules/common/configs/config_gflags.h"
#include "modules/common/map/map_selection.h"
#include "modules/localization/localization_adapter.h"

namespace apollo::localization::openloc {
namespace {

namespace calibration = common::vehicle_calibration;

std::string SensorFrame(const calibration::Registry& registry,
                        const std::string& selector,
                        calibration::SensorType type) {
  const auto* sensor = registry.FindSensor(selector);
  if (selector.empty() || sensor == nullptr ||
      sensor->calibration.sensor_type() != type) {
    throw std::invalid_argument("missing/wrong selected calibration sensor: " +
                                selector);
  }
  return sensor->calibration.frame_id();
}

host::NativeGlobalPose StaticPose(const transform::TransformQuery& query,
                                  const std::string& target,
                                  const std::string& source, float timeout) {
  Eigen::Affine3d pose;
  if (target.empty() || source.empty() ||
      !query.GetLatestStaticTransformToAffine(target, source, &pose, timeout)) {
    throw std::invalid_argument("missing static calibration: " + target +
                                " <- " + source);
  }
  const Eigen::Quaterniond rotation(pose.linear());
  host::NativeGlobalPose result;
  result.set_x(pose.translation().x());
  result.set_y(pose.translation().y());
  result.set_z(pose.translation().z());
  result.set_qx(rotation.x());
  result.set_qy(rotation.y());
  result.set_qz(rotation.z());
  result.set_qw(rotation.w());
  host::NativeMapMatcher::Pose(result);
  return result;
}

std::string MapFile(const common::SelectedMap& map, const std::string& input) {
  const std::filesystem::path relative(input);
  if (relative.empty() || relative.is_absolute()) {
    throw std::invalid_argument("map_pcd must be relative to the selected map");
  }
  for (const auto& part : relative) {
    if (part == "..")
      throw std::invalid_argument("map_pcd escapes selected map");
  }
  const auto root = std::filesystem::canonical(map.directory);
  const auto file = std::filesystem::canonical(root / relative);
  const auto within = file.lexically_relative(root);
  if (within.empty() || *within.begin() == ".." ||
      !std::filesystem::is_regular_file(file)) {
    throw std::invalid_argument(
        "map_pcd is not a file inside the selected map");
  }
  return file.string();
}

void RejectInlineCalibration(const LocalizationConfig& input) {
  const auto& local = input.local();
  const auto& global = input.global();
  const auto& gnss = global.structured_gnss();
  if (input.has_imu_frame() || input.has_imu_from_base() ||
      input.has_imu_from_vrp_flu() || local.has_lidar_frame() ||
      local.has_calibration_id() || local.has_imu_from_lidar_translation() ||
      local.has_imu_from_lidar_rotation() || global.has_calibration_id() ||
      global.has_reference_id() || global.online_map().has_lidar_frame() ||
      global.online_map().has_imu_from_lidar() || gnss.has_lever_x() ||
      gnss.has_lever_y() || gnss.has_lever_z() ||
      gnss.heading().has_baseline_x() || gnss.heading().has_baseline_y() ||
      gnss.heading().has_baseline_z()) {
    throw std::invalid_argument(
        "remove inline calibration/map identity; use common selection and "
        "static TF");
  }
}

}  // namespace

LocalizationConfig LoadDeploymentConfiguration(const std::string& path) {
  if (!std::filesystem::is_regular_file(path) ||
      std::filesystem::file_size(path) > 1024 * 1024) {
    throw std::invalid_argument(
        "missing or oversized localization configuration");
  }
  std::ifstream file(path);
  const std::string text((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
  LocalizationConfig config;
  google::protobuf::TextFormat::Parser parser;
  // Host profiles require resolved extrinsics. Only the deployment parse is
  // partial; ResolveConfiguration fills shared values and validates
  // completeness.
  parser.AllowPartialMessage(true);
  if (!file || !parser.ParseFromString(text, &config)) {
    throw std::invalid_argument(
        "cannot parse localization deployment configuration");
  }
  RejectInlineCalibration(config);
  return config;
}

LocalizationConfig ResolveConfiguration(
    const LocalizationConfig& deployment, const calibration::Registry& registry,
    const transform::TransformQuery& query) {
  RejectInlineCalibration(deployment);
  if (!deployment.deployment_configured() || registry.sensors().empty() ||
      !std::isfinite(deployment.static_tf_timeout_s()) ||
      deployment.static_tf_timeout_s() < 0) {
    throw std::invalid_argument(
        "configured deployment and selected calibration required");
  }
  common::SelectedMap selected;
  if (!common::MapSelection::GetSelectedMap(&selected)) {
    throw std::invalid_argument("cannot resolve common selected map");
  }
  LocalizationConfig config = deployment;
  if (!config.has_parent_frame()) {
    config.set_parent_frame(FLAGS_localization_tf2_frame_id);
  }
  if (!config.has_child_frame()) {
    config.set_child_frame(FLAGS_localization_tf2_child_frame_id);
  }
  if (config.child_frame() != "base_link") {
    throw std::invalid_argument(
        "selected vehicle calibration is rooted at base_link");
  }
  config.set_imu_frame(
      SensorFrame(registry, config.imu_sensor(), calibration::IMU));
  const auto lidar_frame =
      SensorFrame(registry, config.lidar_sensor(), calibration::LIDAR);
  const auto gnss_frame =
      SensorFrame(registry, config.gnss_sensor(), calibration::GNSS);
  const auto timeout = config.static_tf_timeout_s();
  const auto lidar =
      StaticPose(query, config.imu_frame(), lidar_frame, timeout);
  *config.mutable_imu_from_base() =
      StaticPose(query, config.imu_frame(), config.child_frame(), timeout);
  *config.mutable_imu_from_vrp_flu() =
      StaticPose(query, config.imu_frame(), config.vrp_frame(), timeout);
  const auto antenna =
      StaticPose(query, config.imu_frame(), gnss_frame, timeout);
  const auto calibration_id =
      registry.manifest_sha256().empty()
          ? registry.profile().vehicle_type() + "-" +
                registry.manifest().calibration_version()
          : registry.manifest_sha256();
  auto* local = config.mutable_local();
  local->set_lidar_frame(lidar_frame);
  local->set_calibration_id(calibration_id);
  local->mutable_imu_from_lidar_translation()->set_x(lidar.x());
  local->mutable_imu_from_lidar_translation()->set_y(lidar.y());
  local->mutable_imu_from_lidar_translation()->set_z(lidar.z());
  local->mutable_imu_from_lidar_rotation()->set_qx(lidar.qx());
  local->mutable_imu_from_lidar_rotation()->set_qy(lidar.qy());
  local->mutable_imu_from_lidar_rotation()->set_qz(lidar.qz());
  local->mutable_imu_from_lidar_rotation()->set_qw(lidar.qw());
  auto* global = config.mutable_global();
  global->set_reference_id(selected.id);
  global->set_calibration_id(calibration_id);
  global->mutable_online_map()->set_map_pcd(
      MapFile(selected, global->online_map().map_pcd()));
  global->mutable_online_map()->set_lidar_frame(lidar_frame);
  *global->mutable_online_map()->mutable_imu_from_lidar() = lidar;
  auto* gnss = global->mutable_structured_gnss();
  gnss->set_lever_x(antenna.x());
  gnss->set_lever_y(antenna.y());
  gnss->set_lever_z(antenna.z());
  if (gnss->has_heading()) {
    const auto secondary_frame = SensorFrame(
        registry, config.secondary_gnss_sensor(), calibration::GNSS);
    const auto secondary =
        StaticPose(query, config.imu_frame(), secondary_frame, timeout);
    gnss->mutable_heading()->set_baseline_x(secondary.x() - antenna.x());
    gnss->mutable_heading()->set_baseline_y(secondary.y() - antenna.y());
    gnss->mutable_heading()->set_baseline_z(secondary.z() - antenna.z());
  } else if (config.has_secondary_gnss_sensor()) {
    throw std::invalid_argument(
        "secondary GNSS selection requires heading configuration");
  }
  ValidateConfiguration(config);
  return config;
}

}  // namespace apollo::localization::openloc
