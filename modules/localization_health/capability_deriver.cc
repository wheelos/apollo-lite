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

#include "modules/localization_health/capability_deriver.h"

#include <cmath>

#include "modules/localization_health/reason_aggregator.h"

namespace apollo {
namespace localization {

uint64_t CapabilityDeriver::DeriveCapabilities(
    const LocalizationAssessment& assessment, const LocalizationEstimate* pose,
    uint64_t active_reasons, const LocalizationHealthConfig& config,
    bool* out_c_min_met, bool* out_c_nominal_met) {
  uint64_t caps = 0;

  bool local_pose_valid =
      assessment.pose_valid() &&
      !ReasonAggregator::HasReason(active_reasons, REASON_LOCAL_POSE_INVALID) &&
      !ReasonAggregator::HasReason(active_reasons, REASON_NON_FINITE_OUTPUT) &&
      !ReasonAggregator::HasReason(active_reasons, REASON_INVALID_QUATERNION);
  if (local_pose_valid) {
    caps |= LOCAL_POSE_VALID;
  }

  bool local_pose_continuous =
      assessment.output_continuous() &&
      !ReasonAggregator::HasReason(active_reasons,
                                   REASON_UNDECLARED_POSE_JUMP) &&
      !ReasonAggregator::HasReason(active_reasons, REASON_KINEMATIC_VIOLATION);
  if (local_pose_continuous) {
    caps |= LOCAL_POSE_CONTINUOUS;
  }

  bool velocity_valid =
      assessment.velocity_valid() &&
      !ReasonAggregator::HasReason(active_reasons,
                                   REASON_KINEMATIC_VIOLATION) &&
      !ReasonAggregator::HasReason(active_reasons, REASON_NON_FINITE_OUTPUT);
  if (velocity_valid) {
    caps |= VELOCITY_VALID;
  }

  bool heading_valid =
      assessment.heading_valid() &&
      !ReasonAggregator::HasReason(active_reasons, REASON_INVALID_QUATERNION);
  if (heading_valid) {
    caps |= HEADING_VALID;
  }

  bool fresh =
      !ReasonAggregator::HasReason(active_reasons, REASON_POSE_TIMEOUT) &&
      !ReasonAggregator::HasReason(active_reasons, REASON_ASSESSMENT_TIMEOUT) &&
      !ReasonAggregator::HasReason(active_reasons, REASON_DATA_TOO_OLD);

  bool c_min = local_pose_valid && local_pose_continuous && velocity_valid &&
               heading_valid && fresh;
  if (out_c_min_met) {
    *out_c_min_met = c_min;
  }

  bool map_match_good = !assessment.has_map_match_score() ||
                        assessment.map_match_score() >=
                            config.nominal_map_match_score_threshold();

  bool map_aligned =
      assessment.map_alignment_valid() && map_match_good &&
      !ReasonAggregator::HasReason(active_reasons, REASON_MAP_MATCH_FAILED) &&
      !ReasonAggregator::HasReason(active_reasons, REASON_MAP_MISMATCH);
  if (map_aligned) {
    caps |= MAP_ALIGNED;
  }

  bool pos_std_good = !assessment.covariance_valid() ||
                      (assessment.position_std_x() <=
                           config.nominal_horizontal_uncertainty_threshold() &&
                       assessment.position_std_y() <=
                           config.nominal_horizontal_uncertainty_threshold());

  bool global_pose_valid =
      local_pose_valid && map_aligned && pos_std_good &&
      !ReasonAggregator::HasReason(active_reasons, REASON_GLOBAL_POSE_INVALID);
  if (global_pose_valid) {
    caps |= GLOBAL_POSE_VALID;
  }

  bool yaw_std_good =
      !assessment.covariance_valid() ||
      assessment.yaw_std() <= config.nominal_heading_uncertainty_threshold();

  bool lane_level_valid = assessment.lane_level_valid() && map_aligned &&
                          pos_std_good && yaw_std_good &&
                          !ReasonAggregator::HasReason(
                              active_reasons, REASON_LANE_LEVEL_UNAVAILABLE);
  if (lane_level_valid) {
    caps |= LANE_LEVEL_VALID;
  }

  if (local_pose_valid && velocity_valid && heading_valid &&
      local_pose_continuous) {
    caps |= SHORT_TERM_PREDICTION_VALID;
  }

  bool reloc_avail = assessment.relocalization_phase() != RECOVERY_FAILED &&
                     !ReasonAggregator::HasReason(active_reasons,
                                                  REASON_RELOCALIZATION_FAILED);
  if (reloc_avail) {
    caps |= RELOCALIZATION_AVAILABLE;
  }

  // Quality budget check for nominal operations
  bool quality_budget = true;
  if (assessment.has_degeneracy_level() && assessment.degeneracy_level() > 0) {
    quality_budget = false;
  }
  if (assessment.has_innovation_test_valid() &&
      assessment.innovation_test_valid() && !assessment.innovation_passed()) {
    quality_budget = false;
  }
  if (pose && pose->has_uncertainty() &&
      pose->uncertainty().has_position_std_dev()) {
    double sx = pose->uncertainty().position_std_dev().x();
    double sy = pose->uncertainty().position_std_dev().y();
    if (std::sqrt(sx * sx + sy * sy) >
        config.nominal_horizontal_uncertainty_threshold() * 1.5) {
      quality_budget = false;
    }
  }

  bool c_nominal = c_min && global_pose_valid && map_aligned &&
                   lane_level_valid && quality_budget;
  if (out_c_nominal_met) {
    *out_c_nominal_met = c_nominal;
  }

  return caps;
}

}  // namespace localization
}  // namespace apollo
