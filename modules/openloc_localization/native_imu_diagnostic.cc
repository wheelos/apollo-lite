// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <cmath>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "native_cyber_publisher.h"
#include "openloc/localization/local/native_odometry.h"

#include "wheelos_msgs/sensor_msgs/imu.pb.h"

namespace u = ::openloc::localization::unified;
namespace host = apollo::localization::openloc_diagnostic;

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: native_imu_diagnostic IMU_TOPIC "
                 "MEASUREMENT_TO_UNIX_OFFSET_SECONDS SESSION\n"
                 "Explicit diagnostic IMU-only quiet-window startup; no "
                 "LiDAR, wheel, map, GNSS or production output.\n";
    return 1;
  }
  bool initialized = false;
  try {
    std::size_t end = 0;
    const double offset = std::stod(argv[2], &end);
    if (end != std::string(argv[2]).size() || !std::isfinite(offset) ||
        std::string(argv[1]).empty() || std::string(argv[3]).empty())
      throw std::invalid_argument("invalid explicit sensor time/topic/session");
    if (!apollo::cyber::Init(argv[0]))
      throw std::runtime_error("Cyber initialization failed");
    initialized = true;
    {
      std::shared_ptr<apollo::cyber::Node> node =
          apollo::cyber::CreateNode("openloc_native_imu_diagnostic");
      if (!node) throw std::runtime_error("cannot create diagnostic node");
      u::LocalConfig config;
      config.session = argv[3];
      config.allow_imu_only_initialization = true;
      ::openloc::localization::NativeOdometry owner(config);
      const auto configured = owner.ValidateConfig();
      if (!configured.ok()) throw std::runtime_error(configured.message);
      host::NativeCyberPublisher publisher(node, config.max_sample_age);
      std::mutex mutex;
      std::exception_ptr error;
      uint64_t sequence = 0;
      apollo::cyber::ReaderConfig reader_config;
      reader_config.channel_name = argv[1];
      reader_config.pending_queue_size = 256;
      auto reader = node->CreateReader<apollo::drivers::gnss::Imu>(
          reader_config,
          [&](const std::shared_ptr<apollo::drivers::gnss::Imu>& message) {
            const auto arrival = host::NativePublication::Clock::now();
            const double receive = apollo::cyber::Time::Now().ToSecond();
            std::lock_guard<std::mutex> lock(mutex);
            if (error) return;
            try {
              if (!message->has_measurement_time() ||
                  !message->has_linear_acceleration() ||
                  !message->has_angular_velocity())
                throw std::runtime_error("incomplete raw IMU message");
              u::ImuSample sample;
              sample.stamp = {message->measurement_time() + offset, receive,
                              ++sequence, "unix"};
              const auto& acceleration = message->linear_acceleration();
              const auto& rate = message->angular_velocity();
              if (!acceleration.has_x() || !acceleration.has_y() ||
                  !acceleration.has_z() || !rate.has_x() || !rate.has_y() ||
                  !rate.has_z())
                throw std::runtime_error("missing calibrated FLU IMU axes");
              sample.acceleration = {acceleration.x(), acceleration.y(),
                                     acceleration.z()};
              sample.angular_velocity = {rate.x(), rate.y(), rate.z()};
              const auto result = owner.AddImu(sample);
              const auto integrity = owner.EvaluateIntegrity(receive);
              if (!result.ok() &&
                  !(integrity.ok() &&
                    (result.reason == u::Reason::WHEEL_STALE ||
                     result.reason == u::Reason::CORRELATION_UNQUALIFIED)))
                throw std::runtime_error(result.message);
              if (owner.state().stamp.sequence)
                publisher.Update(owner.state(), integrity, arrival);
            } catch (...) {
              error = std::current_exception();
              AERROR << "OpenLoc diagnostic IMU input failed";
            }
          });
      if (!reader) throw std::runtime_error("cannot subscribe to raw IMU");
      while (apollo::cyber::OK()) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          if (error) std::rethrow_exception(error);
        }
        publisher.Check();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      reader->Shutdown();
      publisher.Finish();
    }
    apollo::cyber::Clear();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "OpenLoc diagnostic host failed: " << error.what() << '\n';
    if (initialized) apollo::cyber::Clear();
    return 1;
  }
}
