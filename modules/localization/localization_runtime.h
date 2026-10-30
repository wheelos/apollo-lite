// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "modules/localization/localization_adapter.h"
#include "modules/openloc_localization/native_lio_worker.h"
#include "modules/openloc_localization/native_source_ingress.h"

namespace apollo::localization::openloc {

class LocalizationRuntime {
 public:
  explicit LocalizationRuntime(const LocalizationConfig& config);
  ~LocalizationRuntime();
  void AddImu(const drivers::gnss::Imu& input, double receive);
  void AddCloud(const drivers::PointCloud& input, double receive);
  void AddBestPose(const drivers::gnss::GnssBestPose& input, double receive);
  void AddHeading(const drivers::gnss::Heading& input, double receive);
  void Invalidate(const std::string& reason);
  Publication Sample(double now, uint32_t sequence);

 private:
  void Check() const;
  bool AddGraph(const host::NativeGraphFrame& frame,
                host::NativePublication::Clock::time_point arrival);
  const LocalizationConfig config_;
  native::LocalConfig local_config_;
  host::NativePublication publication_;
  mutable std::mutex mutex_;
  std::string failure_;
  std::deque<native::LocalState> kinematics_;
  std::mutex input_mutex_;
  std::mutex lidar_mutex_;
  uint64_t imu_sequence_ = 0;
  uint64_t lidar_sequence_ = 0;
  std::unique_ptr<host::NativeSourceIngress> ingress_;
  std::unique_ptr<host::NativeGlobalWorker> global_;
  // Destroy/join the local producer before the global consumer.
  std::unique_ptr<host::NativeLioWorker> local_;
};

}  // namespace apollo::localization::openloc
