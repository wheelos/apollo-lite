// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <fstream>
#include <iomanip>

#include "native_global_test_input.h"

namespace apollo::localization::openloc_diagnostic::test {

inline std::vector<Eigen::Vector3d> MapGeometry() {
  std::vector<Eigen::Vector3d> points;
  for (int x = 0; x <= 40; ++x) {
    for (int y = 0; y <= 30; ++y) {
      const double px = -4 + 0.2 * x, py = -3 + 0.2 * y;
      points.emplace_back(px, py, 0.05 * px + 0.02 * py * py + 0.002 * px * py);
    }
  }
  for (int y = 0; y <= 20; ++y) {
    for (int z = 1; z <= 15; ++z) {
      const double py = -2 + 0.2 * y, pz = 0.2 * z;
      points.emplace_back(3.5 + 0.03 * std::sin(py) + 0.01 * pz, py, pz);
    }
  }
  for (int x = 0; x <= 25; ++x) {
    for (int z = 1; z <= 13; ++z) {
      const double px = -3 + 0.2 * x, pz = 0.2 * z;
      points.emplace_back(px, 3.1 + 0.025 * std::sin(px) + 0.015 * pz, pz);
    }
  }
  return points;
}

inline void EnableOnlineMap(NativeGlobalHostConfig* config, const std::string& path) {
  const auto points = MapGeometry();
  std::ofstream file(path);
  file << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
       << "WIDTH " << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
       << "POINTS " << points.size() << "\nDATA ascii\n" << std::setprecision(9);
  for (const auto& p : points) file << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
  file.close();
  if (!file) throw std::runtime_error("cannot write synthetic map");
  config->set_maximum_odometry_age_ms(1000);
  auto* online = config->mutable_online_map();
  online->set_cloud_topic("/openloc/diagnostic/map-cloud/" +
                         config->reference_id());
  online->set_lidar_frame("lidar");
  online->set_map_pcd(path);
  online->set_map_sha256(NativeMapMatcher::Sha256(path));
  online->set_preparation_id("synthetic-conditional-map");
  online->set_hypothesis_minimum_ns(100000000000LL);
  online->set_hypothesis_maximum_ns(101000000000LL);
  online->set_window_radius_m(10);
  online->set_window_cell_m(2);
  online->set_task_deadline_ms(500);
  online->set_hypothesis_odometry_session("synthetic-global-test");
  online->set_hypothesis_odometry_generation(1);
  NativeGlobalPose identity;
  identity.set_x(0); identity.set_y(0); identity.set_z(0);
  identity.set_qx(0); identity.set_qy(0); identity.set_qz(0); identity.set_qw(1);
  *online->mutable_imu_from_lidar() = identity;
  *online->mutable_initial_map_from_odom() = identity;
}

inline NativeOdometryDiagnostic MapOdometry(int index, uint64_t generation = 1) {
  auto source = Odometry(index, generation);
  const double seconds = 100 + 0.01 * index;
  source.set_measurement_time(seconds);
  source.set_receive_time(seconds);
  source.mutable_odom_from_imu()->mutable_header()->set_timestamp_sec(seconds);
  source.mutable_odom_from_imu()->mutable_transform()->mutable_translation()->set_x(0);
  return source;
}

inline NativeGlobalWorker::RawMapPacket MapCloud(int index) {
  NativeGlobalWorker::RawMapPacket packet;
  packet.receive_ns = 100000000000LL + index * 10000000LL;
  auto& cloud = packet.cloud;
  cloud.mutable_header()->set_timestamp_sec(packet.receive_ns / 1e9);
  cloud.mutable_header()->set_sequence_num(index + 1);
  cloud.set_measurement_time(packet.receive_ns / 1e9);
  cloud.set_frame_id("lidar");
  const auto points = MapGeometry();
  for (const auto& p : points) {
    auto* point = cloud.add_point();
    point->set_x(p.x() - 0.35);
    point->set_y(p.y() + 0.22);
    point->set_z(p.z() - 0.18);
    point->set_timestamp(packet.receive_ns);
  }
  return packet;
}

}  // namespace apollo::localization::openloc_diagnostic::test
