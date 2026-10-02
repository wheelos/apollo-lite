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
    double now_sec, uint64_t active_reasons,
    const LocalizationHealthConfig& config,
    bool directional_lane_valid,
    bool* out_c_min_met, bool* out_c_nominal_met) {
  uint64_t caps = 0;

  bool fresh =
      !ReasonAggregator::HasReason(active_reasons, REASON_POSE_TIMEOUT) &&
      !ReasonAggregator::HasReason(active_reasons, REASON_ASSESSMENT_TIMEOUT) &&
      !ReasonAggregator::HasReason(active_reasons, REASON_DATA_TOO_OLD) &&
      !ReasonAggregator::HasReason(active_reasons,
                                   REASON_TIMESTAMP_REGRESSION) &&
      !ReasonAggregator::HasReason(active_reasons, REASON_SEQUENCE_ERROR) &&
      !ReasonAggregator::HasReason(active_reasons, REASON_SESSION_CHANGED) &&
      !ReasonAggregator::HasReason(active_reasons, REASON_TIME_SYNC_ERROR);
  bool numerically_valid =
      !ReasonAggregator::HasReason(active_reasons, REASON_NON_FINITE_OUTPUT) &&
      !ReasonAggregator::HasReason(active_reasons,
                                   REASON_INVALID_QUATERNION) &&
      !ReasonAggregator::HasReason(active_reasons,
                                   REASON_INVALID_COVARIANCE) &&
      !ReasonAggregator::HasReason(active_reasons,
                                   REASON_KINEMATIC_VIOLATION) &&
      !ReasonAggregator::HasReason(active_reasons,
                                   REASON_UNDECLARED_POSE_JUMP);
  bool local_pose_valid =
      pose != nullptr && assessment.estimator_running() &&
      assessment.pose_valid() && assessment.covariance_valid() && fresh &&
      numerically_valid &&
      !ReasonAggregator::HasReason(active_reasons, REASON_LOCAL_POSE_INVALID) &&
      !ReasonAggregator::HasReason(active_reasons,
                                   REASON_ESTIMATOR_NOT_RUNNING);
  if (local_pose_valid) {
    caps |= LOCAL_POSE_VALID;
  }

  bool local_pose_continuous =
      local_pose_valid && assessment.output_continuous();
  if (local_pose_continuous) {
    caps |= LOCAL_POSE_CONTINUOUS;
  }

  bool velocity_valid =
      local_pose_valid && assessment.velocity_valid() &&
      pose->pose().has_linear_velocity();
  if (velocity_valid) {
    caps |= VELOCITY_VALID;
  }

  bool heading_valid =
      local_pose_valid && assessment.heading_valid() &&
      pose->pose().has_orientation();
  if (heading_valid) {
    caps |= HEADING_VALID;
  }

  bool c_min = local_pose_valid && local_pose_continuous && velocity_valid &&
               heading_valid && fresh;
  if (out_c_min_met) {
    *out_c_min_met = c_min;
  }

  bool map_aligned =
      local_pose_valid && assessment.global_pose_valid() &&
      assessment.georeference_valid() && assessment.map_alignment_valid() &&
      !ReasonAggregator::HasReason(active_reasons, REASON_MAP_MATCH_FAILED) &&
      !ReasonAggregator::HasReason(active_reasons, REASON_MAP_MISMATCH);
  if (map_aligned) {
    caps |= MAP_ALIGNED;
  }

  bool assessment_pos_std_good =
      assessment.covariance_valid() && assessment.has_position_std_x() &&
      assessment.has_position_std_y() &&
      std::isfinite(assessment.position_std_x()) &&
      std::isfinite(assessment.position_std_y()) &&
      assessment.position_std_x() >= 0.0 &&
      assessment.position_std_y() >= 0.0 &&
      assessment.position_std_x() <=
          config.nominal_horizontal_uncertainty_threshold() &&
      assessment.position_std_y() <=
          config.nominal_horizontal_uncertainty_threshold();
  bool pose_pos_std_good =
      pose != nullptr && pose->has_uncertainty() &&
      pose->uncertainty().has_position_std_dev() &&
      std::isfinite(pose->uncertainty().position_std_dev().x()) &&
      std::isfinite(pose->uncertainty().position_std_dev().y()) &&
      pose->uncertainty().position_std_dev().x() >= 0.0 &&
      pose->uncertainty().position_std_dev().y() >= 0.0 &&
      pose->uncertainty().position_std_dev().x() <=
          config.nominal_horizontal_uncertainty_threshold() &&
      pose->uncertainty().position_std_dev().y() <=
          config.nominal_horizontal_uncertainty_threshold();

  bool global_pose_valid =
      map_aligned && assessment_pos_std_good && pose_pos_std_good &&
      !ReasonAggregator::HasReason(active_reasons, REASON_GLOBAL_POSE_INVALID);
  if (global_pose_valid) {
    caps |= GLOBAL_POSE_VALID;
  }

  bool lane_evidence_fresh =
      std::isfinite(assessment.lane_evidence_time()) &&
      std::isfinite(assessment.lane_evidence_valid_until()) &&
      assessment.lane_evidence_time() > 0.0 &&
      assessment.lane_evidence_time() <= now_sec &&
      assessment.lane_evidence_valid_until() >= now_sec;
  bool lane_epoch_valid =
      assessment.odom_generation() > 0 &&
      std::isfinite(assessment.local_measurement_time()) &&
      assessment.local_measurement_time() > 0.0 && pose != nullptr &&
      std::abs(assessment.local_measurement_time() -
               pose->measurement_time()) <= config.pose_timeout_threshold();
  bool legacy_lane_valid =
      local_pose_valid && assessment.lane_level_valid() &&
      assessment.lane_evidence_independent() &&
      !assessment.lane_source().empty() && lane_evidence_fresh &&
      lane_epoch_valid;
  bool lane_level_valid =
      (legacy_lane_valid || directional_lane_valid) &&
      !ReasonAggregator::HasReason(active_reasons,
                                   REASON_LANE_LEVEL_UNAVAILABLE);
  if (lane_level_valid) {
    caps |= LANE_LEVEL_VALID;
  }

  if (local_pose_valid && velocity_valid && heading_valid &&
      local_pose_continuous) {
    caps |= SHORT_TERM_PREDICTION_VALID;
  }

  bool reloc_avail =
      fresh && assessment.recovery_available() &&
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

  bool global_profile_met =
      !config.require_global_for_nominal() || global_pose_valid;
  bool lane_profile_met =
      !config.require_lane_for_nominal() || lane_level_valid;
  bool lane_directional_quality_override =
      config.require_lane_for_nominal() && directional_lane_valid;
  bool c_nominal =
      c_min && global_profile_met && lane_profile_met &&
      (quality_budget || lane_directional_quality_override);
  if (out_c_nominal_met) {
    *out_c_nominal_met = c_nominal;
  }

  return caps;
}

}  // namespace localization
}  // namespace apollo
