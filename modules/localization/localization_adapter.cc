// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "modules/localization/localization_adapter.h"

#include <cmath>
#include <set>
#include <stdexcept>
#include <string>

#include "modules/common/math/quaternion.h"

namespace apollo::localization::openloc {
namespace {

template <typename Vector>
Eigen::Vector3d ReadVector(const Vector& value) {
  const Eigen::Vector3d result(value.x(), value.y(), value.z());
  if (!value.has_x() || !value.has_y() || !value.has_z() ||
      !result.allFinite()) {
    throw std::invalid_argument("missing or nonfinite vector");
  }
  return result;
}

template <typename Vector>
void SetVector(const Eigen::Vector3d& value, Vector* output) {
  output->set_x(value.x());
  output->set_y(value.y());
  output->set_z(value.z());
}

void CheckTime(double measurement, double receive) {
  host::NativeMapMatcher::Nanoseconds(measurement);
  host::NativeMapMatcher::Nanoseconds(receive);
  if (measurement > receive + 0.02) {
    throw std::invalid_argument(
        "measurement is ahead of the Unix receive clock");
  }
}

}  // namespace

void ValidateConfiguration(const LocalizationConfig& config) {
  if (!config.IsInitialized() || !config.deployment_configured() ||
      config.imu_frame().empty() || config.vertical_datum().empty() ||
      !std::isfinite(config.utm_origin_height_m()) ||
      config.parent_frame().empty() || config.child_frame().empty() ||
      config.parent_frame() == config.child_frame()) {
    throw std::invalid_argument(
        "complete deployment/map/vehicle calibration required");
  }
  const auto& local = config.local();
  const auto& global = config.global();
  const auto& qualification =
      global.structured_gnss().position_graph_qualification();
  host::LocalConfiguration(local);
  host::NativeMapMatcher::Pose(config.imu_from_base());
  host::NativeMapMatcher::Pose(config.imu_from_vrp_flu());
  if (config.child_frame() != "base_link" || config.vrp_frame().empty()) {
    throw std::invalid_argument(
        "base_link and a declared rear-axle frame required");
  }
  if (!local.has_graph() || !global.has_target_graph() ||
      !global.has_online_map() || !global.has_structured_gnss() ||
      global.has_raw_gga() || global.has_map_topic() ||
      global.has_reset_topic() || global.has_alignment_session() ||
      global.reset_consumers_size() != 0 ||
      local.recorded_receive_time_from_header() || global.recorded_input() ||
      local.graph().recorded_source_clock() ||
      global.target_graph().recorded_source_clock() ||
      !global.target_graph().motion_error_model_qualified() ||
      !qualification.IsInitialized() ||
      qualification.uncertainty_model_id().empty() ||
      qualification.independence_evidence_id().empty() ||
      !qualification.quality_valid() || !qualification.independent_of_local() ||
      !qualification.independent_of_graph_sources() ||
      qualification.has_local_correlation() ||
      local.graph().motion_error_model_id() !=
          global.target_graph().motion_error_model_id() ||
      local.graph().maximum_error_dimension() !=
          global.target_graph().maximum_error_dimension() ||
      local.graph().output_topic() != global.odometry_topic() ||
      local.lidar_topic() != global.online_map().cloud_topic() ||
      local.lidar_frame() != global.online_map().lidar_frame() ||
      local.calibration_id() != global.calibration_id() ||
      local.session() != global.online_map().hypothesis_odometry_session() ||
      global.online_map().hypothesis_odometry_generation() != 1 ||
      local.journal() == global.journal() ||
      !host::LidarExtrinsic(local).matrix().isApprox(
          host::NativeMapMatcher::Pose(global.online_map().imu_from_lidar())
              .matrix(),
          1e-9)) {
    throw std::invalid_argument(
        "inconsistent live local/map/GNSS graph configuration");
  }
  std::set<std::string> topics;
  for (const auto& topic :
       {local.imu_topic(), local.lidar_topic(), global.gnss_topic(),
        global.odometry_topic(), global.output_topic(),
        config.localization_topic(), config.status_topic()}) {
    if (topic.empty() || !topics.insert(topic).second) {
      throw std::invalid_argument("empty or conflicting localization channel");
    }
  }
  if (global.structured_gnss().has_heading() &&
      (global.structured_gnss().heading().topic().empty() ||
       !topics.insert(global.structured_gnss().heading().topic()).second)) {
    throw std::invalid_argument("conflicting heading channel");
  }
}

native::ImuSample DecodeImu(const drivers::gnss::Imu& input,
                            const LocalizationConfig& config, double receive,
                            uint64_t sequence) {
  if (!input.has_measurement_time() ||
      input.header().frame_id() != config.imu_frame() ||
      !input.has_linear_acceleration() || !input.has_angular_velocity() ||
      !std::isfinite(input.measurement_span()) ||
      input.measurement_span() != 0 || sequence == 0) {
    throw std::invalid_argument("instantaneous calibrated FLU IMU required");
  }
  native::ImuSample sample;
  sample.stamp = {input.measurement_time() +
                      config.local().imu_measurement_to_unix_offset(),
                  receive, sequence, "unix"};
  CheckTime(sample.stamp.time, receive);
  sample.acceleration = ReadVector(input.linear_acceleration());
  sample.angular_velocity = ReadVector(input.angular_velocity());
  return sample;
}

native::LidarScan DecodeCloud(const drivers::PointCloud& input,
                              const LocalizationConfig& config,
                              uint32_t maximum_points, double receive,
                              uint64_t sequence) {
  if (!input.has_measurement_time() ||
      input.frame_id() != config.local().lidar_frame() ||
      input.point_size() == 0 || input.point_size() > 1000000 ||
      input.ByteSizeLong() > 32 * 1024 * 1024 || maximum_points == 0 ||
      sequence == 0) {
    throw std::invalid_argument("bounded timed LiDAR cloud required");
  }
  native::LidarScan scan;
  const double offset = config.local().lidar_measurement_to_unix_offset();
  scan.stamp = {input.measurement_time() + offset, receive, sequence, "unix"};
  CheckTime(scan.stamp.time, receive);
  scan.frame_id = config.local().lidar_frame();
  scan.calibration_id = config.local().calibration_id();
  scan.base_from_lidar = host::LidarExtrinsic(config.local());
  size_t returns = 0;
  double previous = 0;
  for (const auto& point : input.point()) {
    const auto xyz = ReadVector(point);
    const double time = static_cast<double>(point.timestamp()) / 1e9 + offset;
    if (!point.has_timestamp() || point.timestamp() == 0 || time <= 0 ||
        time < previous || time > scan.stamp.time) {
      throw std::invalid_argument(
          "LiDAR requires ordered absolute point timestamps");
    }
    previous = time;
    if (!xyz.isZero(0)) ++returns;
  }
  if (returns == 0) throw std::invalid_argument("LiDAR has no valid returns");
  const size_t stride = (returns + maximum_points - 1) / maximum_points;
  size_t index = 0;
  for (const auto& point : input.point()) {
    const auto xyz = ReadVector(point);
    if (xyz.isZero(0) || index++ % stride != 0) continue;
    scan.points.push_back(
        {xyz, static_cast<double>(point.timestamp()) / 1e9 + offset});
  }
  return scan;
}

Publication MakePublication(const LocalizationConfig& config,
                            const host::NativeGlobalDiagnostic& global,
                            const native::LocalState* local, double now,
                            uint32_t sequence) {
  Publication output;
  auto* header = output.status.mutable_header();
  header->set_module_name("openloc_localization");
  header->set_timestamp_sec(now);
  header->set_sequence_num(sequence);
  header->set_frame_id(config.parent_frame());
  output.status.set_fusion_status(MeasureState::ERROR);
  output.status.set_state_message(global.reason() + " " +
                                  global.graph_assessment_reason());
  if (global.has_measurement_ns()) {
    output.status.set_measurement_time(global.measurement_ns() / 1e9);
  }
  if (!global.usable() || !global.full_pose_available() ||
      !global.target_graph_available() ||
      global.selected_owner() != "fixed_lag" || global.reset_pending() ||
      global.recorded_input() || !global.has_map_from_imu() ||
      !global.has_map_from_odom()) {
    return output;
  }
  if (local == nullptr || !local->valid ||
      local->epoch.session != global.session() ||
      local->epoch.generation != global.generation() ||
      local->stamp.sequence != global.odometry_sequence() ||
      host::NativeMapMatcher::Nanoseconds(local->stamp.time) !=
          global.measurement_ns() ||
      !local->velocity.allFinite() || !local->angular_velocity.allFinite()) {
    output.status.set_state_message(
        "No exact local kinematics for global pose");
    return output;
  }
  if (!std::isfinite(now) || now < local->stamp.time - 0.02 ||
      now - local->stamp.time >
          config.global().maximum_odometry_age_ms() / 1000.0) {
    output.status.set_state_message("Absolute localization source is stale");
    return output;
  }
  const auto map_from_imu = host::NativeMapMatcher::Pose(global.map_from_imu());
  const auto map_from_odom =
      host::NativeMapMatcher::Pose(global.map_from_odom());
  const auto imu_from_vrp =
      host::NativeMapMatcher::Pose(config.imu_from_vrp_flu());
  Eigen::Isometry3d world_from_base =
      map_from_imu * host::NativeMapMatcher::Pose(config.imu_from_base());
  Eigen::Isometry3d world_from_vrp = map_from_imu * imu_from_vrp;
  const Eigen::Vector3d origin(
      config.global().structured_gnss().origin_easting_m(),
      config.global().structured_gnss().origin_northing_m(),
      config.utm_origin_height_m());
  world_from_vrp.translation() += origin;
  world_from_base.translation() += origin;
  // Apollo Pose uses RFU; the calibrated body TF remains FLU.
  const Eigen::Quaterniond q =
      Eigen::Quaterniond(world_from_vrp.linear()) *
      Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d angular =
      map_from_imu.linear() * local->angular_velocity;
  const Eigen::Vector3d velocity =
      map_from_odom.linear() * local->velocity +
      map_from_imu.linear() *
          local->angular_velocity.cross(imu_from_vrp.translation());
  output.status.set_fusion_status(MeasureState::WARNNING);
  output.status.set_state_message(
      "OpenLoc absolute pose; diagnostic-only, uncertainty unqualified");
  if (!global.new_measurement()) return output;
  output.pose.emplace();
  *output.pose->mutable_header() = *header;
  output.pose->set_measurement_time(local->stamp.time);
  auto* quality = output.pose->mutable_msf_status();
  quality->set_local_lidar_status(LocalLidarStatus::MSF_LOCAL_LIDAR_NOT_GOOD);
  quality->set_local_lidar_quality(LocalLidarQuality::MSF_LOCAL_LIDAR_BAD);
  quality->set_local_lidar_consistency(
      LocalLidarConsistency::MSF_LOCAL_LIDAR_CONSISTENCY_03);
  quality->set_gnss_consistency(GnssConsistency::MSF_GNSS_CONSISTENCY_03);
  quality->set_msf_running_status(MsfRunningStatus::MSF_NOSOL_X_XXX);
  auto* pose = output.pose->mutable_pose();
  SetVector(world_from_vrp.translation(), pose->mutable_position());
  auto* rotation = pose->mutable_orientation();
  rotation->set_qw(q.w());
  rotation->set_qx(q.x());
  rotation->set_qy(q.y());
  rotation->set_qz(q.z());
  pose->set_heading(common::math::QuaternionToHeading(q));
  common::math::EulerAnglesZXYd euler(q.w(), q.x(), q.y(), q.z());
  SetVector({euler.pitch(), euler.roll(), euler.yaw()},
            pose->mutable_euler_angles());
  SetVector(velocity, pose->mutable_linear_velocity());
  SetVector(angular, pose->mutable_angular_velocity());
  SetVector(q.conjugate() * angular, pose->mutable_angular_velocity_vrf());
  output.transform.emplace();
  output.transform->mutable_header()->set_timestamp_sec(local->stamp.time);
  output.transform->mutable_header()->set_frame_id(config.parent_frame());
  output.transform->set_child_frame_id(config.child_frame());
  SetVector(world_from_base.translation(),
            output.transform->mutable_transform()->mutable_translation());
  const Eigen::Quaterniond base_rotation(world_from_base.linear());
  auto* tf_rotation = output.transform->mutable_transform()->mutable_rotation();
  tf_rotation->set_qw(base_rotation.w());
  tf_rotation->set_qx(base_rotation.x());
  tf_rotation->set_qy(base_rotation.y());
  tf_rotation->set_qz(base_rotation.z());
  return output;
}

}  // namespace apollo::localization::openloc
