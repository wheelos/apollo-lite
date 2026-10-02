// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "cyber/component/component.h"
#include "cyber/timer/timer.h"
#include "modules/localization/core/local_estimator.h"
#include "modules/localization/runtime/messages.h"
#include "modules/transform/transform_broadcaster.h"
#include "wheelos_msgs/chassis_msgs/chassis.pb.h"
#include "wheelos_msgs/sensor_msgs/imu.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace localization {
namespace unified {

class LocalLocalizationComponent final
    : public cyber::Component<drivers::gnss::Imu> {
 public:
  ~LocalLocalizationComponent() override;
  bool Init() override;
  bool Proc(const std::shared_ptr<drivers::gnss::Imu>& message) override;

 private:
  void OnChassis(const std::shared_ptr<canbus::Chassis>& message);
  void OnLidar(const std::shared_ptr<drivers::PointCloud>& message);
  void SubmitLidar(LidarScan scan);
  void ProcessPendingLidar();
  void OnReset(const std::shared_ptr<OdomResetRequest>& message);
  void PublishHealth();
  void RejectAdapter(const Result& result, double now);
  LocalEstimatorConfig config_;
  std::unique_ptr<LocalEstimator> estimator_;
  HealthReporter health_;
  std::mutex mutex_;
  Eigen::Isometry3d base_from_imu_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d base_from_lidar_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d previous_angular_ = Eigen::Vector3d::Zero();
  double previous_imu_time_ = 0.0;
  uint64_t imu_sequence_ = 0;
  uint64_t wheel_sequence_ = 0;
  uint64_t lidar_sequence_ = 0;
  std::deque<LidarScan> pending_lidar_;
  Reason adapter_reason_ = Reason::NONE;
  SourceCounters adapter_health_;
  std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<drivers::PointCloud>> lidar_reader_;
  std::shared_ptr<cyber::Reader<OdomResetRequest>> reset_reader_;
  std::shared_ptr<cyber::Writer<LocalOdometry>> odometry_writer_;
  std::shared_ptr<cyber::Writer<LocalizationAssessment>> assessment_writer_;
  std::shared_ptr<cyber::Writer<LocalizationHealthEvent>> event_writer_;
  std::unique_ptr<transform::TransformBroadcaster> broadcaster_;
  std::unique_ptr<cyber::Timer> health_timer_;
};

CYBER_REGISTER_COMPONENT(LocalLocalizationComponent);

}  // namespace unified
}  // namespace localization
}  // namespace apollo
