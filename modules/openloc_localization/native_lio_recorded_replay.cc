// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "google/protobuf/text_format.h"
#include "native_cyber_publisher.h"
#include "native_lio_config.h"
#include "native_lio_worker.h"
#include "native_recorded_input.h"

namespace host = apollo::localization::openloc_diagnostic;
namespace u = ::openloc::localization::unified;
namespace {
using host::recorded::Number;
using host::recorded::Read;
using host::recorded::Time;

void Replay(const std::filesystem::path& input,
            const host::NativeLioHostConfig& profile,
            const std::filesystem::path& output) {
  if (!std::filesystem::create_directory(output))
    throw std::runtime_error("raw replay output must be a new directory");
  const auto config = host::LocalConfiguration(profile);
  const auto extrinsic = host::LidarExtrinsic(profile);
  if (profile.imu_measurement_to_unix_offset() != 0 ||
      profile.lidar_measurement_to_unix_offset() != 0)
    throw std::runtime_error("prepared timestamps must already use Unix epoch");
  const auto imu =
      Read(input / "imu.csv", "timestamp_ns,receive_ns,ax,ay,az,gx,gy,gz", 8);
  const auto scans = Read(input / "scans.csv", "end_ns,receive_ns,points", 3);
  if (imu.empty() || scans.empty())
    throw std::runtime_error("raw replay requires IMU and LiDAR");
  std::shared_ptr<apollo::cyber::Node> node =
      apollo::cyber::CreateNode("openloc_recorded_lio_diagnostic");
  if (!node) throw std::runtime_error("cannot create raw replay Cyber node");
  host::NativeCyberPublisher publisher(node, config.max_sample_age, true);
  std::ofstream states(output / "odometry.csv");
  if (!states) throw std::runtime_error("cannot open replay source journal");
  states << std::setprecision(17)
         << "timestamp_ns,receive_ns,sequence,valid,px,py,pz,qx,qy,qz,qw,"
            "vx,vy,vz,covariance_model_valid,integrity_reason\n";
  std::size_t state_count = 0;
  uint64_t valid_count = 0;
  host::NativeLioWorker worker(
      config, profile.bootstrap_voxel_m(), (output / "lidar.csv").string(),
      [&](const auto& state, const auto& integrity, auto arrival) {
        // Integer source identities come from the original input row, not a
        // double-to-integer round trip or a wall-clock rebasing operation.
        const auto& row = imu.at(state.stamp.sequence - 1);
        states << row[0] << ',' << row[1] << ',' << state.stamp.sequence << ','
               << state.valid;
        for (int index = 0; index < 10; ++index)
          states << ','
                 << (index < 3   ? state.position[index]
                     : index < 7 ? state.orientation.coeffs()[index - 3]
                                 : state.velocity[index - 7]);
        states << ',' << state.covariance_model_valid << ','
               << u::ReasonName(integrity.reason) << '\n';
        if (!states) throw std::runtime_error("incomplete replay state write");
        ++state_count;
        valid_count += state.valid;
        publisher.Update(state, integrity, arrival);
      },
      std::chrono::milliseconds(profile.lidar_commit_delay_ms()),
      std::chrono::milliseconds(profile.startup_task_timeout_ms()));
  const auto origin = host::NativePublication::Clock::now();
  const int64_t first_receive = Time(imu.front()[1]);
  std::size_t next_scan = 0;
  int64_t last_measurement = 0;
  int64_t last_receive = 0;
  for (std::size_t index = 0; index < imu.size(); ++index) {
    const auto& row = imu[index];
    const int64_t measurement = Time(row[0]);
    const int64_t receive = Time(row[1]);
    if (measurement <= last_measurement || receive < last_receive)
      throw std::runtime_error("raw IMU frontier regressed");
    last_measurement = measurement;
    last_receive = receive;
    const auto arrival =
        origin + std::chrono::nanoseconds(receive - first_receive);
    std::this_thread::sleep_until(arrival);
    while (next_scan < scans.size() &&
           Time(scans[next_scan][0]) <= measurement &&
           Time(scans[next_scan][1]) <= receive) {
      const auto& source = scans[next_scan];
      const std::filesystem::path relative(source[2]);
      if (relative.is_absolute())
        throw std::runtime_error(
            "scan path must be relative to prepared input");
      for (const auto& component : relative)
        if (component == "..")
          throw std::runtime_error("scan path escaped prepared input");
      const auto points = Read(input / relative, "timestamp_ns,x,y,z", 4);
      u::LidarScan scan;
      scan.stamp = {Time(source[0]) / 1e9, Time(source[1]) / 1e9, ++next_scan,
                    "unix"};
      scan.frame_id = config.lidar_frame;
      scan.calibration_id = config.lidar_calibration_id;
      scan.base_from_lidar = extrinsic;
      for (const auto& point : points)
        scan.points.push_back(
            {{Number(point[1]), Number(point[2]), Number(point[3])},
             Time(point[0]) / 1e9});
      const auto source_arrival =
          origin + std::chrono::nanoseconds(Time(source[1]) - first_receive);
      worker.AddLidar(std::move(scan), source_arrival);
    }
    u::ImuSample sample;
    sample.stamp = {measurement / 1e9, receive / 1e9, index + 1, "unix"};
    for (int axis = 0; axis < 3; ++axis) {
      sample.acceleration[axis] = Number(row[axis + 2]);
      sample.angular_velocity[axis] = Number(row[axis + 5]);
    }
    worker.AddImu(std::move(sample), arrival);
    publisher.Check();
  }
  worker.Finish();
  publisher.Finish();
  states.flush();
  if (!states || state_count != imu.size() || valid_count == 0)
    throw std::runtime_error(
        "raw host replay did not produce all source states");
  std::ofstream summary(output / "summary.csv");
  summary << "imu_samples,valid_states,scans_enqueued,scans_unconsumed\n"
          << state_count << ',' << valid_count << ',' << next_scan << ','
          << scans.size() - next_scan << '\n';
  summary.close();
  if (!summary) throw std::runtime_error("incomplete raw host replay summary");
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: native_lio_recorded_replay INPUT PROFILE.pb.txt "
                 "NEW_OUTPUT\n";
    return 1;
  }
  bool initialized = false;
  try {
    std::ifstream file(argv[2]);
    const std::string text((std::istreambuf_iterator<char>(file)), {});
    host::NativeLioHostConfig profile;
    if (!file || !google::protobuf::TextFormat::ParseFromString(text, &profile))
      throw std::runtime_error("unreadable recorded host profile");
    if (!apollo::cyber::Init(argv[0]))
      throw std::runtime_error("Cyber initialization failed");
    initialized = true;
    Replay(argv[1], profile, argv[3]);
    apollo::cyber::Clear();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "OpenLoc recorded LIO host failed: " << error.what() << '\n';
    if (initialized) apollo::cyber::Clear();
    return 1;
  }
}
