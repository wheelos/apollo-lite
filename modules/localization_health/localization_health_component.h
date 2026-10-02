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

#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/localization_health/localization_health.h"
#include "modules/localization_health/proto/localization_health.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"

namespace apollo {
namespace localization {

class LocalizationHealthComponent final : public cyber::TimerComponent {
 public:
  LocalizationHealthComponent() = default;
  ~LocalizationHealthComponent() override = default;

  bool Init() override;
  bool Proc() override;

 private:
  void OnPose(const std::shared_ptr<LocalizationEstimate>& pose_msg);
  void OnAssessment(
      const std::shared_ptr<LocalizationAssessment>& assessment_msg);

 private:
  LocalizationHealthConfig config_;
  LocalizationHealth localization_health_;

  std::shared_ptr<cyber::Reader<LocalizationEstimate>> pose_reader_;
  std::shared_ptr<cyber::Reader<LocalizationAssessment>> assessment_reader_;
  std::shared_ptr<cyber::Writer<LocalizationHealthStatus>> status_writer_;
  std::shared_ptr<cyber::Writer<LocalizationHealthEvent>> event_writer_;
};

CYBER_REGISTER_COMPONENT(LocalizationHealthComponent)

}  // namespace localization
}  // namespace apollo
