// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <memory>
#include <future>
#include <mutex>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/timer/timer.h"
#include "modules/localization/map/map_matcher.h"
#include "modules/localization/runtime/gnss_adapter.h"
#include "modules/localization/runtime/messages.h"
#include "modules/transform/transform_broadcaster.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace localization {
namespace unified {

class GlobalLocalizationComponent final : public cyber::Component<LocalOdometry> {
 public:
  ~GlobalLocalizationComponent() override;
  bool Init() override;
  bool Proc(const std::shared_ptr<LocalOdometry>& message) override;

 private:
  bool InitMap();
  bool InitGnss();
  void OnObservation(const std::shared_ptr<GlobalObservationMessage>& message);
  void OnLocalHealth(const std::shared_ptr<LocalizationAssessment>& message);
  void OnCloud(const std::shared_ptr<drivers::PointCloud>& message);
  void OnGnss(const std::shared_ptr<drivers::gnss::GnssBestPose>& message);
  void OnHeading(const std::shared_ptr<drivers::gnss::Heading>& message);
  void PublishHealth();
  void Report(const Result& result);
  void OnMapSwitch(const std::shared_ptr<MapSwitchRequest>& request);
  void FinishMapSwitch(double now);
  bool PublishMapSwitch(uint64_t request_id, MapSwitchStatus::Phase phase,
                        const Result& result, double now);
  struct PendingMap {
    GlobalEstimatorConfig config;
    std::shared_ptr<MapMatcher> matcher;
    std::unique_ptr<GnssAdapter> gnss;
    std::vector<Eigen::Isometry3d> seeds;
    Result result;
  };
  GlobalEstimatorConfig config_;
  std::unique_ptr<GlobalAlignment> alignment_;
  std::shared_ptr<MapMatcher> matcher_;
  std::unique_ptr<GnssAdapter> gnss_;
  std::vector<Eigen::Isometry3d> recovery_seeds_;
  HealthReporter health_;
  bool local_valid_ = false;
  bool local_precision_valid_ = false;
  Reason input_reason_ = Reason::NONE;
  SourceCounters cloud_health_;
  uint64_t cloud_sequence_ = 0;
  uint64_t map_generation_ = 1;
  uint64_t map_request_sequence_ = 0;
  MapSwitchRequest pending_map_request_;
  std::future<PendingMap> pending_map_;
  Eigen::Vector3d antenna_in_base_ = Eigen::Vector3d::Zero();
  std::mutex mutex_;
  std::mutex cloud_mutex_;
  std::shared_ptr<cyber::Reader<GlobalObservationMessage>> observation_reader_;
  std::shared_ptr<cyber::Reader<MapSwitchRequest>> map_switch_reader_;
  std::shared_ptr<cyber::Writer<MapSwitchStatus>> map_switch_writer_;
  std::shared_ptr<cyber::Reader<LocalizationAssessment>> local_health_reader_;
  std::shared_ptr<cyber::Reader<drivers::PointCloud>> cloud_reader_;
  std::shared_ptr<cyber::Reader<drivers::gnss::GnssBestPose>> gnss_reader_;
  std::shared_ptr<cyber::Reader<drivers::gnss::Heading>> heading_reader_;
  std::shared_ptr<cyber::Writer<GlobalLocalization>> global_writer_;
  std::shared_ptr<cyber::Writer<GlobalConstraintSet>> constraints_writer_;
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> pose_writer_;
  std::shared_ptr<cyber::Writer<LocalizationStatus>> status_writer_;
  std::shared_ptr<cyber::Writer<LocalizationAssessment>> assessment_writer_;
  std::shared_ptr<cyber::Writer<LocalizationHealthEvent>> event_writer_;
  std::unique_ptr<transform::TransformBroadcaster> broadcaster_;
  std::unique_ptr<cyber::Timer> health_timer_;
};

CYBER_REGISTER_COMPONENT(GlobalLocalizationComponent);

}  // namespace unified
}  // namespace localization
}  // namespace apollo
