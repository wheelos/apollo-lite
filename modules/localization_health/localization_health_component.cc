// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2026-09-06
//  Author: daohu527

#include "modules/localization_health/localization_health_component.h"

#include <memory>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"

namespace apollo {
namespace localization {

bool LocalizationHealthComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Unable to load localization health conf file: "
           << ConfigFilePath();
    return false;
  }

  localization_health_.Init(config_);

  pose_reader_ = node_->CreateReader<LocalizationEstimate>(
      config_.localization_pose_topic(),
      [this](const std::shared_ptr<LocalizationEstimate>& pose_msg) {
        OnPose(pose_msg);
      });

  assessment_reader_ = node_->CreateReader<LocalizationAssessment>(
      config_.localization_assessment_topic(),
      [this](const std::shared_ptr<LocalizationAssessment>& assessment_msg) {
        OnAssessment(assessment_msg);
      });

  status_writer_ = node_->CreateWriter<LocalizationHealthStatus>(
      config_.localization_status_topic());

  event_writer_ = node_->CreateWriter<LocalizationHealthEvent>(
      config_.localization_event_topic());

  return true;
}

void LocalizationHealthComponent::OnPose(
    const std::shared_ptr<LocalizationEstimate>& pose_msg) {
  if (pose_msg != nullptr) {
    double now = cyber::Clock::NowInSeconds();
    localization_health_.UpdatePose(*pose_msg, now);
  }
}

void LocalizationHealthComponent::OnAssessment(
    const std::shared_ptr<LocalizationAssessment>& assessment_msg) {
  if (assessment_msg != nullptr) {
    double now = cyber::Clock::NowInSeconds();
    localization_health_.UpdateAssessment(*assessment_msg, now);
  }
}

bool LocalizationHealthComponent::Proc() {
  double now = cyber::Clock::NowInSeconds();
  LocalizationHealthStatus status = localization_health_.Evaluate(now);
  status_writer_->Write(status);

  LocalizationHealthEvent event;
  if (localization_health_.PopTransitionEvent(&event)) {
    event_writer_->Write(event);
  }
  return true;
}

}  // namespace localization
}  // namespace apollo
