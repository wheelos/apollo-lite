// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <set>

#include "cyber/component/component.h"
#include "cyber/timer/timer.h"
#include "modules/localization/core/lane_relative.h"
#include "modules/localization/runtime/messages.h"
#include "modules/localization_health/proto/localization_health.pb.h"
#include "wheelos_msgs/perception_msgs/perception_lane.pb.h"

namespace apollo {
namespace localization {
namespace unified {

class LocalizationHealthBridge final : public cyber::Component<LocalOdometry> {
 public:
  ~LocalizationHealthBridge() override;
  bool Init() override;
  bool Proc(const std::shared_ptr<LocalOdometry>& message) override;

 private:
  void OnGlobal(const std::shared_ptr<GlobalLocalization>& message);
  void OnLocalAssessment(const std::shared_ptr<LocalizationAssessment>& message);
  void OnGlobalAssessment(const std::shared_ptr<LocalizationAssessment>& message);
  void OnLanes(const std::shared_ptr<perception::PerceptionLanes>& message);
  Result Lookup(double time, LocalState* state) const;
  void Tick();
  bool LocalUsable(double now) const;
  bool LocalIntegrity(double now) const;
  LaneRelativeState EvaluateLaneNow(double now) const;
  HealthBridgeConfig config_;
  GlobalEstimatorConfig decoding_config_;
  LanePolicy lane_policy_;
  std::deque<LocalState> history_;
  std::set<std::string> retired_sessions_;
  LocalOdometry latest_;
  GlobalLocalization global_;
  LocalizationAssessment local_assessment_;
  LocalizationAssessment global_assessment_;
  LaneGeometry lane_geometry_;
  LocalState lane_local_;
  double lane_time_ = 0.0;
  double lane_frontier_ = 0.0;
  uint64_t lane_sequence_ = 0;
  uint64_t lane_reference_generation_ = 0;
  uint64_t assessment_sequence_ = 0;
  std::string lane_source_;
  Result lane_result_{Reason::HISTORY_UNAVAILABLE, "no lane evidence"};
  std::mutex mutex_;
  std::shared_ptr<cyber::Reader<GlobalLocalization>> global_reader_;
  std::shared_ptr<cyber::Reader<LocalizationAssessment>> local_assessment_reader_;
  std::shared_ptr<cyber::Reader<LocalizationAssessment>> global_assessment_reader_;
  std::shared_ptr<cyber::Reader<perception::PerceptionLanes>> lanes_reader_;
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> pose_writer_;
  std::shared_ptr<cyber::Writer<apollo::localization::LocalizationAssessment>>
      assessment_writer_;
  std::shared_ptr<cyber::Writer<LaneRelativeState>> lane_writer_;
  std::unique_ptr<cyber::Timer> timer_;
};

CYBER_REGISTER_COMPONENT(LocalizationHealthBridge);

}  // namespace unified
}  // namespace localization
}  // namespace apollo
