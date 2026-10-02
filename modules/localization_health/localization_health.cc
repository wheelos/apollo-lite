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

#include "modules/localization_health/localization_health.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <unistd.h>

namespace apollo {
namespace localization {

LocalizationHealth::LocalizationHealth()
    : supervisor_session_id_(GenerateSupervisorSessionId()) {
  LocalizationHealthConfig default_config;
  Init(default_config);
}

LocalizationHealth::LocalizationHealth(const LocalizationHealthConfig &config)
    : supervisor_session_id_(GenerateSupervisorSessionId()) {
  Init(config);
}

std::string LocalizationHealth::GenerateSupervisorSessionId() {
  const auto ticks =
      std::chrono::steady_clock::now().time_since_epoch().count();
  std::ostringstream id;
  id << "localization_health:" << getpid() << ":" << ticks;
  return id.str();
}

void LocalizationHealth::Init(const LocalizationHealthConfig &config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;

  if (config_.assessment_timeout_threshold() <= 0.0) {
    config_.set_assessment_timeout_threshold(0.2);
  }
  if (config_.pose_timeout_threshold() <= 0.0) {
    config_.set_pose_timeout_threshold(0.1);
  }
  if (config_.max_receive_age_threshold() <= 0.0) {
    config_.set_max_receive_age_threshold(0.2);
  }
  if (config_.data_too_old_threshold() <= 0.0) {
    config_.set_data_too_old_threshold(0.5);
  }
  if (config_.failure_debounce_duration() <= 0.0) {
    config_.set_failure_debounce_duration(0.1);
  }
  if (config_.degraded_debounce_duration() <= 0.0) {
    config_.set_degraded_debounce_duration(0.2);
  }
  if (config_.recovery_duration_threshold() <= 0.0) {
    config_.set_recovery_duration_threshold(1.0);
  }
  if (config_.max_linear_velocity() <= 0.0) {
    config_.set_max_linear_velocity(45.0);
  }
  if (config_.max_linear_acceleration() <= 0.0) {
    config_.set_max_linear_acceleration(10.0);
  }
  if (config_.max_angular_velocity() <= 0.0) {
    config_.set_max_angular_velocity(2.0);
  }
  if (config_.pose_jump_threshold() <= 0.0) {
    config_.set_pose_jump_threshold(1.0);
  }
  if (config_.yaw_jump_threshold() <= 0.0) {
    config_.set_yaw_jump_threshold(0.35);
  }
  if (config_.nominal_horizontal_uncertainty_threshold() <= 0.0) {
    config_.set_nominal_horizontal_uncertainty_threshold(0.3);
  }
  if (config_.nominal_heading_uncertainty_threshold() <= 0.0) {
    config_.set_nominal_heading_uncertainty_threshold(0.05);
  }
  if (config_.nominal_map_match_score_threshold() <= 0.0) {
    config_.set_nominal_map_match_score_threshold(0.6);
  }
  if (config_.quaternion_norm_tolerance() <= 0.0) {
    config_.set_quaternion_norm_tolerance(0.01);
  }
  if (config_.direction_recovery_frames() < 3) {
    config_.set_direction_recovery_frames(3);
  }
  if (config_.direction_recovery_min_span() < 0.1) {
    config_.set_direction_recovery_min_span(0.1);
  }
  if (config_.max_retired_producer_sessions() == 0) {
    config_.set_max_retired_producer_sessions(16);
  }

  checker_.Reset();
  directional_constraint_tracker_.Reset();
  reason_aggregator_.Reset();
  state_machine_.Reset();

  status_sequence_ = 0;
  current_session_id_.clear();
  current_capabilities_ = 0;
  current_directional_constraints_.clear();
  retired_session_order_.clear();
  retired_sessions_.clear();
  committed_assessment_time_sec_ = 0.0;
  committed_local_measurement_time_sec_ = 0.0;
  rejected_input_reasons_ = 0;
  evaluation_start_time_sec_ = -1.0;
  has_assessment_ = false;
  has_pose_ = false;
  has_prev_pose_ = false;
}

void LocalizationHealth::ResetSession(const std::string& session_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_session_id_ = session_id;
  retired_session_order_.clear();
  retired_sessions_.clear();
  committed_assessment_time_sec_ = 0.0;
  committed_local_measurement_time_sec_ = 0.0;
  rejected_input_reasons_ = 0;
  checker_.Reset();
  directional_constraint_tracker_.Reset();
  reason_aggregator_.Reset();
  state_machine_.ResetSession(current_session_id_, 0.0);

  has_assessment_ = false;
  has_pose_ = false;
  has_prev_pose_ = false;
  current_capabilities_ = 0;
  current_directional_constraints_.clear();
  evaluation_start_time_sec_ = -1.0;
}

void LocalizationHealth::RetireProducerSession(
    const std::string& session_id) {
  if (session_id.empty() || retired_sessions_.count(session_id) != 0) {
    return;
  }
  retired_sessions_.insert(session_id);
  retired_session_order_.push_back(session_id);
  while (retired_session_order_.size() >
         config_.max_retired_producer_sessions()) {
    retired_sessions_.erase(retired_session_order_.front());
    retired_session_order_.pop_front();
  }
}

bool LocalizationHealth::UpdateAssessment(
    const LocalizationAssessment& assessment, double receive_time_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  double effective_receive_time =
      (receive_time_sec > 0.0) ? receive_time_sec : assessment.timestamp_sec();
  const std::string producer_session =
      assessment.has_session_id() ? assessment.session_id() : "";
  bool assessment_time_valid =
      assessment.has_timestamp_sec() &&
      std::isfinite(assessment.timestamp_sec()) &&
      assessment.timestamp_sec() > 0.0;
  bool local_time_valid =
      assessment.has_local_measurement_time() &&
      std::isfinite(assessment.local_measurement_time()) &&
      assessment.local_measurement_time() > 0.0;

  if (producer_session.empty() ||
      retired_sessions_.count(producer_session) != 0) {
    rejected_input_reasons_ |=
        ReasonAggregator::ReasonToBit(REASON_SESSION_CHANGED);
    return false;
  }

  if (producer_session != current_session_id_) {
    bool is_first_session = current_session_id_.empty();
    bool advances_frontier =
        assessment_time_valid && local_time_valid &&
        (is_first_session ||
         (assessment.timestamp_sec() > committed_assessment_time_sec_ &&
          assessment.local_measurement_time() >
              committed_local_measurement_time_sec_));
    if (!advances_frontier) {
      rejected_input_reasons_ |=
          ReasonAggregator::ReasonToBit(REASON_SESSION_CHANGED);
      return false;
    }

    RetireProducerSession(current_session_id_);
    current_session_id_ = producer_session;
    checker_.Reset();
    directional_constraint_tracker_.Reset();
    reason_aggregator_.Reset();
    state_machine_.ResetSession(current_session_id_, effective_receive_time);
    has_prev_pose_ = false;
    current_capabilities_ = 0;
    current_directional_constraints_.clear();
    evaluation_start_time_sec_ = effective_receive_time;
  }

  latest_assessment_ = assessment;
  latest_assessment_receive_time_sec_ = effective_receive_time;
  has_assessment_ = true;
  if (assessment_time_valid) {
    committed_assessment_time_sec_ =
        std::max(committed_assessment_time_sec_, assessment.timestamp_sec());
  }
  if (local_time_valid) {
    committed_local_measurement_time_sec_ =
        std::max(committed_local_measurement_time_sec_,
                 assessment.local_measurement_time());
  }
  rejected_input_reasons_ = 0;
  return true;
}

void LocalizationHealth::UpdatePose(const LocalizationEstimate& pose,
                                    double receive_time_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (has_pose_) {
    prev_pose_ = latest_pose_;
    prev_pose_time_sec_ = latest_pose_.measurement_time();
    has_prev_pose_ = true;
  }
  latest_pose_ = pose;
  latest_pose_receive_time_sec_ =
      (receive_time_sec > 0.0) ? receive_time_sec : pose.measurement_time();
  has_pose_ = true;
}

LocalizationHealthStatus LocalizationHealth::Evaluate(double now_sec) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (now_sec <= 0.0) {
    if (has_pose_) {
      now_sec = latest_pose_.measurement_time();
    } else if (has_assessment_) {
      now_sec = latest_assessment_.timestamp_sec();
    } else {
      now_sec = 0.0;
    }
  }
  if (evaluation_start_time_sec_ < 0.0) {
    evaluation_start_time_sec_ = now_sec;
  }
  bool absence_is_fault =
      now_sec - evaluation_start_time_sec_ >= config_.data_too_old_threshold();

  // 1. Run independent checks
  uint64_t independent_reasons = checker_.RunAllChecks(
      now_sec, config_, has_pose_, latest_pose_, latest_pose_receive_time_sec_,
      has_prev_pose_, prev_pose_, prev_pose_time_sec_, has_assessment_,
      latest_assessment_, latest_assessment_receive_time_sec_,
      current_session_id_, absence_is_fault);

  // 2. Ingest algorithm faults from assessment
  uint64_t algo_faults = 0;
  if (has_assessment_) {
    if (!latest_assessment_.estimator_running()) {
      ReasonAggregator::AddReason(&algo_faults, REASON_ESTIMATOR_NOT_RUNNING);
    }
    if (!latest_assessment_.estimator_converged()) {
      ReasonAggregator::AddReason(&algo_faults, REASON_ESTIMATOR_NOT_CONVERGED);
    }
    if (!latest_assessment_.pose_valid()) {
      ReasonAggregator::AddReason(&algo_faults, REASON_LOCAL_POSE_INVALID);
    }
    bool assessment_covariance_valid =
        latest_assessment_.covariance_valid() &&
        latest_assessment_.has_position_std_x() &&
        latest_assessment_.has_position_std_y() &&
        latest_assessment_.has_position_std_z() &&
        latest_assessment_.has_yaw_std() &&
        std::isfinite(latest_assessment_.position_std_x()) &&
        std::isfinite(latest_assessment_.position_std_y()) &&
        std::isfinite(latest_assessment_.position_std_z()) &&
        std::isfinite(latest_assessment_.yaw_std()) &&
        latest_assessment_.position_std_x() >= 0.0 &&
        latest_assessment_.position_std_y() >= 0.0 &&
        latest_assessment_.position_std_z() >= 0.0 &&
        latest_assessment_.yaw_std() >= 0.0;
    if (!assessment_covariance_valid) {
      ReasonAggregator::AddReason(&algo_faults, REASON_INVALID_COVARIANCE);
    }
    if (config_.require_global_for_nominal() &&
        latest_assessment_.has_global_pose_valid() &&
        !latest_assessment_.global_pose_valid()) {
      ReasonAggregator::AddReason(&algo_faults, REASON_GLOBAL_POSE_INVALID);
    }
    if (config_.require_global_for_nominal() &&
        latest_assessment_.has_map_alignment_valid() &&
        !latest_assessment_.map_alignment_valid()) {
      ReasonAggregator::AddReason(&algo_faults, REASON_MAP_MATCH_FAILED);
    }
    if (config_.require_global_for_nominal() &&
        latest_assessment_.has_map_match_score() &&
        latest_assessment_.map_match_score() <
        config_.nominal_map_match_score_threshold()) {
      ReasonAggregator::AddReason(&algo_faults, REASON_MAP_MISMATCH);
    }
    if (config_.require_lane_for_nominal() &&
        !latest_assessment_.lane_level_valid()) {
      ReasonAggregator::AddReason(&algo_faults, REASON_LANE_LEVEL_UNAVAILABLE);
    }
    algo_faults |= latest_assessment_.active_algorithm_faults();
  }

  // 3. Aggregate reasons
  reason_aggregator_.Update(independent_reasons | rejected_input_reasons_,
                            algo_faults);

  // 4. Derive capabilities
  bool c_min_met = false;
  bool c_nominal_met = false;
  LocalizationAssessment eval_assessment =
      has_assessment_ ? latest_assessment_ : LocalizationAssessment();
  const LocalizationEstimate* eval_pose = has_pose_ ? &latest_pose_ : nullptr;
  current_directional_constraints_ = directional_constraint_tracker_.Evaluate(
      eval_assessment, eval_pose, now_sec, reason_aggregator_.active_reasons(),
      config_);
  bool directional_lane_valid =
      eval_assessment.lane_level_valid() &&
      DirectionalConstraintTracker::HasLaneLateralAndHeading(
          current_directional_constraints_);
  current_capabilities_ = capability_deriver_.DeriveCapabilities(
      eval_assessment, eval_pose, now_sec,
      reason_aggregator_.active_reasons(), config_, directional_lane_valid,
      &c_min_met, &c_nominal_met);

  // 5. Update state machine
  bool est_running = has_assessment_ && latest_assessment_.estimator_running();
  bool est_converged =
      has_assessment_ && latest_assessment_.estimator_converged();
  AvailabilityState desired = state_machine_.DetermineDesiredState(
      has_assessment_, est_running, est_converged, has_pose_, c_min_met,
      c_nominal_met, reason_aggregator_.active_reasons());

  RecoveryPhase rec_phase = RECOVERY_IDLE;
  if (has_assessment_ && latest_assessment_.has_relocalization_phase()) {
    rec_phase = latest_assessment_.relocalization_phase();
  }
  state_machine_.Update(desired, rec_phase, now_sec, current_session_id_,
                        reason_aggregator_.primary_reason(),
                        reason_aggregator_.active_reasons(), config_);

  // 6. Build status output
  LocalizationHealthStatus status;
  PopulateStatus(now_sec, &status);
  return status;
}

void LocalizationHealth::PopulateStatus(double now_sec,
                                        LocalizationHealthStatus* status) {
  auto* header = status->mutable_header();
  header->set_timestamp_sec(now_sec);
  header->set_module_name("localization_health");
  header->set_sequence_num(++status_sequence_);

  status->set_sequence(status_sequence_);
  status->set_session_id(current_session_id_);
  status->set_supervisor_session_id(supervisor_session_id_);
  status->set_transition_id(state_machine_.transition_id());
  status->set_interface_version(1);

  status->set_availability_state(state_machine_.current_state());
  status->set_recovery_phase(state_machine_.current_recovery_phase());

  status->set_capabilities(current_capabilities_);
  status->set_active_reasons(reason_aggregator_.active_reasons());
  status->set_latched_reasons(reason_aggregator_.latched_reasons());
  status->set_primary_reason(reason_aggregator_.primary_reason());
  for (const auto& constraint : current_directional_constraints_) {
    *status->add_directional_constraints() = constraint;
  }

  if (has_pose_) {
    double pose_t = latest_pose_.measurement_time();
    status->set_measurement_stamp(pose_t);
    status->set_publish_stamp(now_sec);
    status->set_pose_age(static_cast<float>(now_sec - pose_t));

    const auto& unc = latest_pose_.uncertainty();
    if (unc.has_position_std_dev()) {
      float sx = static_cast<float>(unc.position_std_dev().x());
      float sy = static_cast<float>(unc.position_std_dev().y());
      float sz = static_cast<float>(unc.position_std_dev().z());
      status->set_horizontal_uncertainty(std::sqrt(sx * sx + sy * sy));
      status->set_vertical_uncertainty(sz);
    }
  } else {
    status->set_measurement_stamp(0.0);
    status->set_publish_stamp(now_sec);
    status->set_pose_age(999.0f);
  }

  if (has_assessment_) {
    status->set_assessment_age(
        static_cast<float>(now_sec - latest_assessment_.timestamp_sec()));
    status->set_correction_in_progress(
        latest_assessment_.correction_in_progress());
    if (latest_assessment_.has_yaw_std() &&
        std::isfinite(latest_assessment_.yaw_std()) &&
        latest_assessment_.yaw_std() >= 0.0) {
      status->set_heading_uncertainty(
          static_cast<float>(latest_assessment_.yaw_std()));
    }
  } else {
    status->set_assessment_age(999.0f);
    status->set_correction_in_progress(false);
  }
}

bool LocalizationHealth::PopTransitionEvent(LocalizationHealthEvent* event) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_machine_.PopPendingEvent(event)) {
    return false;
  }
  event->set_supervisor_session_id(supervisor_session_id_);
  return true;
}

AvailabilityState LocalizationHealth::current_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_machine_.current_state();
}

RecoveryPhase LocalizationHealth::current_recovery_phase() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_machine_.current_recovery_phase();
}

uint64_t LocalizationHealth::current_capabilities() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_capabilities_;
}

uint64_t LocalizationHealth::active_reasons() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return reason_aggregator_.active_reasons();
}

uint64_t LocalizationHealth::latched_reasons() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return reason_aggregator_.latched_reasons();
}

HealthReason LocalizationHealth::primary_reason() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return reason_aggregator_.primary_reason();
}

uint64_t LocalizationHealth::transition_id() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_machine_.transition_id();
}

LocalizationHealthConfig LocalizationHealth::config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

std::string LocalizationHealth::supervisor_session_id() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return supervisor_session_id_;
}

}  // namespace localization
}  // namespace apollo
