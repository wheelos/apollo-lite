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

#include "modules/localization_health/independent_checker.h"

#include <algorithm>
#include <string>

#include "modules/localization_health/reason_aggregator.h"

namespace apollo {
namespace localization {

void IndependentChecker::Reset() {
  last_pose_sequence_ = 0;
  last_assessment_sequence_ = 0;
}

double IndependentChecker::NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

uint64_t IndependentChecker::RunAllChecks(
    double now_sec, const LocalizationHealthConfig& config, bool has_pose,
    const LocalizationEstimate& latest_pose,
    double latest_pose_receive_time_sec, bool has_prev_pose,
    const LocalizationEstimate& prev_pose, double prev_pose_time_sec,
    bool has_assessment, const LocalizationAssessment& latest_assessment,
    double latest_assessment_receive_time_sec, const std::string& session_id) {
  uint64_t reasons = 0;

  CheckTimeIntegrity(now_sec, config, has_pose, latest_pose,
                     latest_pose_receive_time_sec, has_prev_pose,
                     prev_pose_time_sec, has_assessment, latest_assessment,
                     latest_assessment_receive_time_sec, session_id, &reasons);

  CheckNumericalIntegrity(config, has_pose, latest_pose, &reasons);

  CheckKinematicsAndContinuity(now_sec, config, has_pose, latest_pose,
                               has_prev_pose, prev_pose, has_assessment,
                               latest_assessment, &reasons);

  return reasons;
}

void IndependentChecker::CheckTimeIntegrity(
    double now_sec, const LocalizationHealthConfig& config, bool has_pose,
    const LocalizationEstimate& latest_pose,
    double latest_pose_receive_time_sec, bool has_prev_pose,
    double prev_pose_time_sec, bool has_assessment,
    const LocalizationAssessment& latest_assessment,
    double latest_assessment_receive_time_sec, const std::string& session_id,
    uint64_t* reasons) {
  if (!has_assessment) {
    ReasonAggregator::AddReason(reasons, REASON_ASSESSMENT_TIMEOUT);
  } else {
    double assessment_time = latest_assessment.timestamp_sec();
    double assessment_age = now_sec - assessment_time;
    double assessment_recv_age = now_sec - latest_assessment_receive_time_sec;

    if (assessment_age > config.assessment_timeout_threshold() ||
        assessment_recv_age > config.max_receive_age_threshold()) {
      ReasonAggregator::AddReason(reasons, REASON_ASSESSMENT_TIMEOUT);
    }
    if (assessment_age > config.data_too_old_threshold()) {
      ReasonAggregator::AddReason(reasons, REASON_DATA_TOO_OLD);
    }
    if (latest_assessment.has_sequence()) {
      if (last_assessment_sequence_ > 0 &&
          latest_assessment.sequence() < last_assessment_sequence_) {
        ReasonAggregator::AddReason(reasons, REASON_SEQUENCE_ERROR);
      }
      last_assessment_sequence_ = latest_assessment.sequence();
    }
    if (latest_assessment.has_session_id() && !session_id.empty() &&
        latest_assessment.session_id() != session_id) {
      ReasonAggregator::AddReason(reasons, REASON_SESSION_CHANGED);
    }
  }

  if (!has_pose) {
    ReasonAggregator::AddReason(reasons, REASON_POSE_TIMEOUT);
  } else {
    double pose_time = latest_pose.measurement_time();
    double pose_age = now_sec - pose_time;
    double pose_recv_age = now_sec - latest_pose_receive_time_sec;

    if (pose_age > config.pose_timeout_threshold() ||
        pose_recv_age > config.max_receive_age_threshold()) {
      ReasonAggregator::AddReason(reasons, REASON_POSE_TIMEOUT);
    }
    if (pose_age > config.data_too_old_threshold()) {
      ReasonAggregator::AddReason(reasons, REASON_DATA_TOO_OLD);
    }
    if (has_prev_pose && pose_time < prev_pose_time_sec) {
      ReasonAggregator::AddReason(reasons, REASON_TIMESTAMP_REGRESSION);
    }
    if (latest_pose.header().has_sequence_num()) {
      uint64_t seq = latest_pose.header().sequence_num();
      if (last_pose_sequence_ > 0 && seq < last_pose_sequence_) {
        ReasonAggregator::AddReason(reasons, REASON_SEQUENCE_ERROR);
      }
      last_pose_sequence_ = seq;
    }
  }

  if (has_assessment && has_pose) {
    double dt_sync = std::abs(latest_assessment.timestamp_sec() -
                              latest_pose.measurement_time());
    if (dt_sync > config.pose_timeout_threshold()) {
      ReasonAggregator::AddReason(reasons, REASON_TIME_SYNC_ERROR);
    }
  }
}

void IndependentChecker::CheckNumericalIntegrity(
    const LocalizationHealthConfig& config, bool has_pose,
    const LocalizationEstimate& latest_pose, uint64_t* reasons) {
  if (!has_pose) {
    return;
  }

  const auto& pose = latest_pose.pose();

  if (pose.has_position()) {
    double x = pose.position().x();
    double y = pose.position().y();
    double z = pose.position().z();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      ReasonAggregator::AddReason(reasons, REASON_NON_FINITE_OUTPUT);
    }
  }

  if (pose.has_orientation()) {
    double qx = pose.orientation().qx();
    double qy = pose.orientation().qy();
    double qz = pose.orientation().qz();
    double qw = pose.orientation().qw();
    if (!std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) ||
        !std::isfinite(qw)) {
      ReasonAggregator::AddReason(reasons, REASON_NON_FINITE_OUTPUT);
    }
    double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (std::abs(norm - 1.0) > config.quaternion_norm_tolerance()) {
      ReasonAggregator::AddReason(reasons, REASON_INVALID_QUATERNION);
    }
  }

  if (pose.has_linear_velocity()) {
    double vx = pose.linear_velocity().x();
    double vy = pose.linear_velocity().y();
    double vz = pose.linear_velocity().z();
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)) {
      ReasonAggregator::AddReason(reasons, REASON_NON_FINITE_OUTPUT);
    }
  }

  if (pose.has_linear_acceleration()) {
    double ax = pose.linear_acceleration().x();
    double ay = pose.linear_acceleration().y();
    double az = pose.linear_acceleration().z();
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
      ReasonAggregator::AddReason(reasons, REASON_NON_FINITE_OUTPUT);
    }
  }

  if (pose.has_angular_velocity()) {
    double wx = pose.angular_velocity().x();
    double wy = pose.angular_velocity().y();
    double wz = pose.angular_velocity().z();
    if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz)) {
      ReasonAggregator::AddReason(reasons, REASON_NON_FINITE_OUTPUT);
    }
  }

  if (latest_pose.uncertainty().has_position_std_dev()) {
    double sx = latest_pose.uncertainty().position_std_dev().x();
    double sy = latest_pose.uncertainty().position_std_dev().y();
    double sz = latest_pose.uncertainty().position_std_dev().z();
    if (!std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz) ||
        sx < 0.0 || sy < 0.0 || sz < 0.0) {
      ReasonAggregator::AddReason(reasons, REASON_INVALID_COVARIANCE);
    }
  }
}

void IndependentChecker::CheckKinematicsAndContinuity(
    double now_sec, const LocalizationHealthConfig& config, bool has_pose,
    const LocalizationEstimate& latest_pose, bool has_prev_pose,
    const LocalizationEstimate& prev_pose, bool has_assessment,
    const LocalizationAssessment& latest_assessment, uint64_t* reasons) {
  if (!has_pose) {
    return;
  }

  const auto& cur_p = latest_pose.pose();

  if (cur_p.has_linear_velocity()) {
    double v_sq = cur_p.linear_velocity().x() * cur_p.linear_velocity().x() +
                  cur_p.linear_velocity().y() * cur_p.linear_velocity().y() +
                  cur_p.linear_velocity().z() * cur_p.linear_velocity().z();
    if (v_sq > config.max_linear_velocity() * config.max_linear_velocity()) {
      ReasonAggregator::AddReason(reasons, REASON_KINEMATIC_VIOLATION);
    }
  }

  if (cur_p.has_linear_acceleration()) {
    double a_sq =
        cur_p.linear_acceleration().x() * cur_p.linear_acceleration().x() +
        cur_p.linear_acceleration().y() * cur_p.linear_acceleration().y() +
        cur_p.linear_acceleration().z() * cur_p.linear_acceleration().z();
    if (a_sq >
        config.max_linear_acceleration() * config.max_linear_acceleration()) {
      ReasonAggregator::AddReason(reasons, REASON_KINEMATIC_VIOLATION);
    }
  }

  if (cur_p.has_angular_velocity()) {
    double w_sq = cur_p.angular_velocity().x() * cur_p.angular_velocity().x() +
                  cur_p.angular_velocity().y() * cur_p.angular_velocity().y() +
                  cur_p.angular_velocity().z() * cur_p.angular_velocity().z();
    if (w_sq > config.max_angular_velocity() * config.max_angular_velocity()) {
      ReasonAggregator::AddReason(reasons, REASON_KINEMATIC_VIOLATION);
    }
  }

  if (!has_prev_pose) {
    return;
  }

  const auto& prev_p = prev_pose.pose();
  double dt = latest_pose.measurement_time() - prev_pose.measurement_time();
  if (dt <= 0.0 || dt > 1.0) {
    return;
  }

  double dx = cur_p.position().x() - prev_p.position().x();
  double dy = cur_p.position().y() - prev_p.position().y();
  double dz = cur_p.position().z() - prev_p.position().z();
  double delta_pos = std::sqrt(dx * dx + dy * dy + dz * dz);

  double max_delta_p = config.max_linear_velocity() * dt +
                       0.5 * config.max_linear_acceleration() * dt * dt + 0.3;
  if (delta_pos > max_delta_p) {
    ReasonAggregator::AddReason(reasons, REASON_KINEMATIC_VIOLATION);
  }

  if (cur_p.has_heading() && prev_p.has_heading()) {
    double delta_yaw =
        std::abs(NormalizeAngle(cur_p.heading() - prev_p.heading()));
    double max_delta_yaw = config.max_angular_velocity() * dt + 0.1;
    if (delta_yaw > max_delta_yaw) {
      ReasonAggregator::AddReason(reasons, REASON_KINEMATIC_VIOLATION);
    }
  }

  double expected_dx = cur_p.linear_velocity().x() * dt;
  double expected_dy = cur_p.linear_velocity().y() * dt;
  double unexpected_jump = std::sqrt((dx - expected_dx) * (dx - expected_dx) +
                                     (dy - expected_dy) * (dy - expected_dy));
  bool correction_in_progress =
      has_assessment && latest_assessment.correction_in_progress();
  if (unexpected_jump > config.pose_jump_threshold() &&
      !correction_in_progress) {
    ReasonAggregator::AddReason(reasons, REASON_UNDECLARED_POSE_JUMP);
  }
}

}  // namespace localization
}  // namespace apollo
