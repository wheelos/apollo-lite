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

#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "modules/localization_health/proto/localization_health.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"

namespace apollo {
namespace localization {

class DirectionalConstraintTracker {
 public:
  void Reset();

  std::vector<DirectionalConstraint> Evaluate(
      const LocalizationAssessment& assessment,
      const LocalizationEstimate* pose, double now_sec,
      uint64_t active_reasons, const LocalizationHealthConfig& config);

  static bool HasLaneLateralAndHeading(
      const std::vector<DirectionalConstraint>& constraints);

 private:
  struct RecoveryState {
    uint64_t last_sequence = 0;
    double first_observation_time = 0.0;
    double last_observation_time = 0.0;
    uint32_t observation_count = 0;
    std::array<double, 6> projection{};
  };

  static bool IsStructurallyValid(const DirectionalConstraint& constraint);
  static bool IsProjection(const DirectionalConstraint& constraint,
                           int coefficient_index, const std::string& unit);
  static std::string MakeKey(const std::string& session_id,
                             uint64_t odom_generation,
                             const DirectionalConstraint& constraint);

  std::map<std::string, RecoveryState> recovery_states_;
};

}  // namespace localization
}  // namespace apollo
