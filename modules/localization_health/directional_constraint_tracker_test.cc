// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization_health/directional_constraint_tracker.h"

#include <cmath>

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace {

LocalizationAssessment Assessment(double time, uint64_t sequence,
                                  double scale) {
  LocalizationAssessment assessment;
  assessment.set_session_id("session");
  assessment.set_odom_generation(1);
  assessment.set_estimator_running(true);
  assessment.set_pose_valid(true);
  assessment.set_output_continuous(true);
  assessment.set_local_measurement_time(time);
  auto* direction = assessment.add_directional_constraints();
  direction->set_id("lane-lateral");
  direction->set_reference_frame("vehicle");
  direction->set_reference_id("lane");
  direction->set_source("camera");
  direction->set_unit("meters");
  for (int axis = 0; axis < 6; ++axis) {
    direction->add_projection(axis == 1 ? scale : 0.0);
  }
  direction->set_standard_deviation(0.02 * std::abs(scale));
  direction->set_error_budget(0.1 * std::abs(scale));
  direction->set_last_observation_time(time);
  direction->set_evaluation_time(time);
  direction->set_valid_until(time + 0.2);
  direction->set_observation_sequence(sequence);
  direction->set_independent(true);
  direction->set_mode(DIRECTION_CONSTRAINED);
  return assessment;
}

TEST(DirectionalConstraintTrackerTest, SignAndScaleDoNotRestartRecovery) {
  DirectionalConstraintTracker tracker;
  LocalizationHealthConfig config;
  LocalizationEstimate pose;
  for (int frame = 0; frame < 3; ++frame) {
    const double time = 10.0 + frame * 0.1;
    pose.set_measurement_time(time);
    auto assessment = Assessment(time, frame + 1, frame == 1 ? -2.0 : 1.0);
    const auto output = tracker.Evaluate(assessment, &pose, time, 0, config);
    ASSERT_EQ(1U, output.size());
    EXPECT_EQ(frame == 2 ? DIRECTION_CONSTRAINED : DIRECTION_UNAVAILABLE,
              output[0].mode());
    EXPECT_DOUBLE_EQ(assessment.directional_constraints(0).standard_deviation(),
                     output[0].standard_deviation());
  }
}

TEST(DirectionalConstraintTrackerTest, DifferentDirectionOrUnitRestartsRecovery) {
  DirectionalConstraintTracker tracker;
  LocalizationHealthConfig config;
  LocalizationEstimate pose;
  for (int frame = 0; frame < 4; ++frame) {
    const double time = 10.0 + frame * 0.1;
    pose.set_measurement_time(time);
    auto assessment = Assessment(time, frame + 1, 1.0);
    if (frame >= 2) {
      auto* row = assessment.mutable_directional_constraints(0);
      row->set_projection(1, 0.0);
      row->set_projection(5, 1.0);
      if (frame == 3) {
        row->set_unit("radians");
      }
    }
    const auto output = tracker.Evaluate(assessment, &pose, time, 0, config);
    ASSERT_EQ(1U, output.size());
    EXPECT_EQ(DIRECTION_UNAVAILABLE, output[0].mode());
  }
}

TEST(DirectionalConstraintTrackerTest, DuplicateRowsCannotCountAsNewFrames) {
  DirectionalConstraintTracker tracker;
  LocalizationHealthConfig config;
  LocalizationEstimate pose;
  pose.set_measurement_time(10.0);
  auto assessment = Assessment(10.0, 1, 1.0);
  const DirectionalConstraint original = assessment.directional_constraints(0);
  *assessment.add_directional_constraints() = original;
  const auto output = tracker.Evaluate(assessment, &pose, 10.0, 0, config);
  ASSERT_EQ(2U, output.size());
  EXPECT_EQ(DIRECTION_UNAVAILABLE, output[0].mode());
  EXPECT_EQ(DIRECTION_UNAVAILABLE, output[1].mode());
}

}  // namespace
}  // namespace localization
}  // namespace apollo
