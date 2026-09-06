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

#include <cmath>
#include <string>

#include "modules/localization_health/proto/localization_health.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"

namespace apollo {
namespace localization {

/**
 * @class IndependentChecker
 * @brief Independent low-complexity validation to reduce common-mode failures.
 * Evaluates time, numerical, and kinematic/continuity integrity.
 */
class IndependentChecker {
 public:
  IndependentChecker() = default;
  ~IndependentChecker() = default;

  void Reset();

  // Run all independent checks and return a bitmask of detected reasons.
  uint64_t RunAllChecks(double now_sec, const LocalizationHealthConfig& config,
                        bool has_pose, const LocalizationEstimate& latest_pose,
                        double latest_pose_receive_time_sec, bool has_prev_pose,
                        const LocalizationEstimate& prev_pose,
                        double prev_pose_time_sec, bool has_assessment,
                        const LocalizationAssessment& latest_assessment,
                        double latest_assessment_receive_time_sec,
                        const std::string& session_id);

  static double NormalizeAngle(double angle);

 private:
  void CheckTimeIntegrity(double now_sec,
                          const LocalizationHealthConfig& config, bool has_pose,
                          const LocalizationEstimate& latest_pose,
                          double latest_pose_receive_time_sec,
                          bool has_prev_pose, double prev_pose_time_sec,
                          bool has_assessment,
                          const LocalizationAssessment& latest_assessment,
                          double latest_assessment_receive_time_sec,
                          const std::string& session_id, uint64_t* reasons);

  void CheckNumericalIntegrity(const LocalizationHealthConfig& config,
                               bool has_pose,
                               const LocalizationEstimate& latest_pose,
                               uint64_t* reasons);

  void CheckKinematicsAndContinuity(
      double now_sec, const LocalizationHealthConfig& config, bool has_pose,
      const LocalizationEstimate& latest_pose, bool has_prev_pose,
      const LocalizationEstimate& prev_pose, bool has_assessment,
      const LocalizationAssessment& latest_assessment, uint64_t* reasons);

  uint64_t last_pose_sequence_ = 0;
  uint64_t last_assessment_sequence_ = 0;
};

}  // namespace localization
}  // namespace apollo
