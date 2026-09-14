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

#include <cmath>
#include <limits>

#include "gtest/gtest.h"

namespace apollo {
namespace localization {

class LocalizationHealthTest : public ::testing::Test {
 protected:
  void SetUp() override {
    LocalizationHealthConfig config;
    config.set_pose_timeout_threshold(0.1);
    config.set_assessment_timeout_threshold(0.1);
    config.set_data_too_old_threshold(0.5);
    config.set_max_receive_age_threshold(0.2);
    config.set_recovery_duration_threshold(0.5);
    config.set_failure_debounce_duration(0.05);
    config.set_degraded_debounce_duration(0.1);
    config.set_nominal_horizontal_uncertainty_threshold(0.2);
    config.set_nominal_map_match_score_threshold(0.6);

    health_.Init(config);
    health_.ResetSession("session-test-01");
  }

  LocalizationEstimate CreateValidPose(double time_sec, double x = 0.0,
                                       double y = 0.0, double z = 0.0,
                                       double yaw = 0.0) {
    LocalizationEstimate est;
    est.set_measurement_time(time_sec);
    est.mutable_header()->set_timestamp_sec(time_sec);
    est.mutable_header()->set_sequence_num(1);

    auto* pose = est.mutable_pose();
    pose->mutable_position()->set_x(x);
    pose->mutable_position()->set_y(y);
    pose->mutable_position()->set_z(z);

    // Orientation around z
    pose->mutable_orientation()->set_qw(std::cos(yaw * 0.5));
    pose->mutable_orientation()->set_qx(0.0);
    pose->mutable_orientation()->set_qy(0.0);
    pose->mutable_orientation()->set_qz(std::sin(yaw * 0.5));
    pose->set_heading(yaw);

    pose->mutable_linear_velocity()->set_x(5.0);
    pose->mutable_linear_velocity()->set_y(0.0);
    pose->mutable_linear_velocity()->set_z(0.0);

    pose->mutable_linear_acceleration()->set_x(0.5);
    pose->mutable_linear_acceleration()->set_y(0.0);
    pose->mutable_linear_acceleration()->set_z(0.0);

    pose->mutable_angular_velocity()->set_x(0.0);
    pose->mutable_angular_velocity()->set_y(0.0);
    pose->mutable_angular_velocity()->set_z(0.05);

    return est;
  }

  LocalizationAssessment CreateValidAssessment(double time_sec) {
    LocalizationAssessment ass;
    ass.set_timestamp_sec(time_sec);
    ass.mutable_header()->set_timestamp_sec(time_sec);
    ass.set_sequence(1);
    ass.set_session_id("session-test-01");

    ass.set_estimator_running(true);
    ass.set_estimator_converged(true);
    ass.set_pose_valid(true);
    ass.set_velocity_valid(true);
    ass.set_heading_valid(true);

    ass.set_covariance_valid(true);
    ass.set_position_std_x(0.05);
    ass.set_position_std_y(0.05);
    ass.set_position_std_z(0.1);
    ass.set_yaw_std(0.01);

    ass.set_local_consistency_valid(true);
    ass.set_map_alignment_valid(true);
    ass.set_lane_level_valid(true);
    ass.set_output_continuous(true);

    ass.set_innovation_test_valid(true);
    ass.set_innovation_passed(true);
    ass.set_map_match_score(0.95);
    ass.set_degeneracy_level(0);

    ass.set_correction_in_progress(false);
    ass.set_relocalization_phase(RECOVERY_IDLE);

    return ass;
  }

  LocalizationHealth health_;
};

TEST_F(LocalizationHealthTest, InitialStateAndReset) {
  LocalizationHealth uninit_health;
  EXPECT_EQ(uninit_health.current_state(), AVAILABILITY_UNKNOWN);
  EXPECT_EQ(uninit_health.current_capabilities(), 0);

  health_.ResetSession("session-new");
  EXPECT_EQ(health_.current_state(), AVAILABILITY_INITIALIZING);

  LocalizationHealthEvent event;
  EXPECT_TRUE(health_.PopTransitionEvent(&event));
  EXPECT_EQ(event.current_state(), AVAILABILITY_INITIALIZING);
  EXPECT_EQ(event.session_id(), "session-new");
}

TEST_F(LocalizationHealthTest, ReachNominalAfterRecoveryDuration) {
  double t = 1000.0;
  auto pose = CreateValidPose(t);
  auto ass = CreateValidAssessment(t);

  health_.UpdatePose(pose, t);
  health_.UpdateAssessment(ass, t);

  // First eval: desired is NOMINAL, but recovery hysteresis requires 0.5s
  auto status1 = health_.Evaluate(t);
  EXPECT_EQ(status1.availability_state(), AVAILABILITY_INITIALIZING);

  // At t + 0.3s (less than 0.5s recovery threshold)
  t += 0.3;
  pose = CreateValidPose(t, 1.5, 0.0);
  ass = CreateValidAssessment(t);
  health_.UpdatePose(pose, t);
  health_.UpdateAssessment(ass, t);
  auto status2 = health_.Evaluate(t);
  EXPECT_EQ(status2.availability_state(), AVAILABILITY_INITIALIZING);

  // At t + 0.3s (total 0.6s >= 0.5s)
  t += 0.3;
  pose = CreateValidPose(t, 3.0, 0.0);
  ass = CreateValidAssessment(t);
  health_.UpdatePose(pose, t);
  health_.UpdateAssessment(ass, t);
  auto status3 = health_.Evaluate(t);
  EXPECT_EQ(status3.availability_state(), AVAILABILITY_NOMINAL);
  EXPECT_TRUE(LocalizationHealth::HasCapability(status3.capabilities(),
                                                Capability::LOCAL_POSE_VALID));
  EXPECT_TRUE(LocalizationHealth::HasCapability(status3.capabilities(),
                                                Capability::GLOBAL_POSE_VALID));
  EXPECT_TRUE(LocalizationHealth::HasCapability(status3.capabilities(),
                                                Capability::LANE_LEVEL_VALID));

  LocalizationHealthEvent event;
  EXPECT_TRUE(health_.PopTransitionEvent(&event));
  EXPECT_EQ(event.previous_state(), AVAILABILITY_INITIALIZING);
  EXPECT_EQ(event.current_state(), AVAILABILITY_NOMINAL);
}

TEST_F(LocalizationHealthTest, HardFaultImmediateInvalid) {
  // First reach NOMINAL
  double t = 1000.0;
  for (int i = 0; i < 7; ++i) {
    auto pose = CreateValidPose(t, i * 0.1, 0.0);
    auto ass = CreateValidAssessment(t);
    health_.UpdatePose(pose, t);
    health_.UpdateAssessment(ass, t);
    health_.Evaluate(t);
    t += 0.1;
  }
  EXPECT_EQ(health_.current_state(), AVAILABILITY_NOMINAL);

  // Inject hard fault: NaN position
  auto bad_pose = CreateValidPose(t);
  bad_pose.mutable_pose()->mutable_position()->set_x(
      std::numeric_limits<double>::quiet_NaN());
  auto ass = CreateValidAssessment(t);
  health_.UpdatePose(bad_pose, t);
  health_.UpdateAssessment(ass, t);

  auto status = health_.Evaluate(t);
  // Must immediately transition to INVALID (fail fast)
  EXPECT_EQ(status.availability_state(), AVAILABILITY_INVALID);
  EXPECT_EQ(status.primary_reason(), REASON_NON_FINITE_OUTPUT);
  EXPECT_TRUE(LocalizationHealth::HasReason(status.latched_reasons(),
                                            REASON_NON_FINITE_OUTPUT));

  LocalizationHealthEvent event;
  EXPECT_TRUE(health_.PopTransitionEvent(&event));
  EXPECT_EQ(event.previous_state(), AVAILABILITY_NOMINAL);
  EXPECT_EQ(event.current_state(), AVAILABILITY_INVALID);
  EXPECT_EQ(event.primary_reason(), REASON_NON_FINITE_OUTPUT);
}

TEST_F(LocalizationHealthTest, TimeoutImmediateInvalid) {
  // Reach NOMINAL
  double t = 1000.0;
  for (int i = 0; i < 7; ++i) {
    health_.UpdatePose(CreateValidPose(t, i * 0.1, 0.0), t);
    health_.UpdateAssessment(CreateValidAssessment(t), t);
    health_.Evaluate(t);
    t += 0.1;
  }
  EXPECT_EQ(health_.current_state(), AVAILABILITY_NOMINAL);

  // Advance time without new messages -> timeout
  t += 0.3;  // exceeds pose_timeout_threshold (0.1)
  auto status = health_.Evaluate(t);
  EXPECT_EQ(status.availability_state(), AVAILABILITY_INVALID);
  EXPECT_EQ(status.primary_reason(), REASON_ASSESSMENT_TIMEOUT);
}

TEST_F(LocalizationHealthTest, DegradedOnMapMismatch) {
  // Reach NOMINAL
  double t = 1000.0;
  for (int i = 0; i < 7; ++i) {
    health_.UpdatePose(CreateValidPose(t, i * 0.1, 0.0), t);
    health_.UpdateAssessment(CreateValidAssessment(t), t);
    health_.Evaluate(t);
    t += 0.1;
  }
  EXPECT_EQ(health_.current_state(), AVAILABILITY_NOMINAL);

  // Map match score drops below threshold (0.6)
  for (int i = 0; i < 3; ++i) {
    auto pose = CreateValidPose(t, 1.0 + i * 0.1, 0.0);
    auto ass = CreateValidAssessment(t);
    ass.set_map_match_score(0.4);  // degraded
    ass.set_map_alignment_valid(false);

    health_.UpdatePose(pose, t);
    health_.UpdateAssessment(ass, t);
    health_.Evaluate(t);
    t += 0.1;  // total 0.2s, exceeds degraded_debounce_duration (0.1s)
  }

  EXPECT_EQ(health_.current_state(), AVAILABILITY_DEGRADED);
  EXPECT_TRUE(LocalizationHealth::HasCapability(health_.current_capabilities(),
                                                Capability::LOCAL_POSE_VALID));
  EXPECT_FALSE(LocalizationHealth::HasCapability(health_.current_capabilities(),
                                                 Capability::MAP_ALIGNED));
}

TEST_F(LocalizationHealthTest, UndeclaredPoseJumpTriggersFault) {
  double t = 1000.0;
  health_.UpdatePose(CreateValidPose(t, 0.0, 0.0), t);
  health_.UpdateAssessment(CreateValidAssessment(t), t);
  health_.Evaluate(t);

  t += 0.02;  // 20ms
  // Jump by 10 meters without declared correction
  health_.UpdatePose(CreateValidPose(t, 10.0, 0.0), t);
  health_.UpdateAssessment(CreateValidAssessment(t), t);
  auto status = health_.Evaluate(t);

  EXPECT_EQ(status.availability_state(), AVAILABILITY_INVALID);
  EXPECT_TRUE(LocalizationHealth::HasReason(status.active_reasons(),
                                            REASON_UNDECLARED_POSE_JUMP));
}

TEST_F(LocalizationHealthTest, DeclaredCorrectionToleratesJump) {
  // Reach NOMINAL
  double t = 1000.0;
  for (int i = 0; i < 7; ++i) {
    health_.UpdatePose(CreateValidPose(t, i * 0.1, 0.0), t);
    health_.UpdateAssessment(CreateValidAssessment(t), t);
    health_.Evaluate(t);
    t += 0.1;
  }
  EXPECT_EQ(health_.current_state(), AVAILABILITY_NOMINAL);

  // Jump by 2.0 meters, BUT correction_in_progress is TRUE
  t += 0.02;
  auto ass = CreateValidAssessment(t);
  ass.set_correction_in_progress(true);
  ass.set_relocalization_phase(RECOVERY_ALIGNING);

  health_.UpdatePose(CreateValidPose(t, 5.0, 0.0), t);
  health_.UpdateAssessment(ass, t);
  auto status = health_.Evaluate(t);

  // Should NOT be flagged as undeclared jump
  EXPECT_FALSE(LocalizationHealth::HasReason(status.active_reasons(),
                                             REASON_UNDECLARED_POSE_JUMP));
  EXPECT_EQ(status.recovery_phase(), RECOVERY_ALIGNING);
  EXPECT_TRUE(status.correction_in_progress());
}

TEST_F(LocalizationHealthTest, QuaternionNormCheck) {
  double t = 1000.0;
  auto pose = CreateValidPose(t);
  // Set unnormalized quaternion
  pose.mutable_pose()->mutable_orientation()->set_qw(2.0);
  pose.mutable_pose()->mutable_orientation()->set_qx(0.0);
  pose.mutable_pose()->mutable_orientation()->set_qy(0.0);
  pose.mutable_pose()->mutable_orientation()->set_qz(0.0);

  health_.UpdatePose(pose, t);
  health_.UpdateAssessment(CreateValidAssessment(t), t);
  auto status = health_.Evaluate(t);

  EXPECT_TRUE(LocalizationHealth::HasReason(status.active_reasons(),
                                            REASON_INVALID_QUATERNION));
}

TEST_F(LocalizationHealthTest, TimestampRegressionFault) {
  double t = 1000.0;
  health_.UpdatePose(CreateValidPose(t), t);
  health_.UpdateAssessment(CreateValidAssessment(t), t);
  health_.Evaluate(t);

  // Regress timestamp
  double t_regress = 999.0;
  health_.UpdatePose(CreateValidPose(t_regress), t);
  health_.UpdateAssessment(CreateValidAssessment(t_regress), t);
  auto status = health_.Evaluate(t);

  EXPECT_TRUE(LocalizationHealth::HasReason(status.active_reasons(),
                                            REASON_TIMESTAMP_REGRESSION));
  EXPECT_EQ(status.availability_state(), AVAILABILITY_INVALID);
}

}  // namespace localization
}  // namespace apollo
