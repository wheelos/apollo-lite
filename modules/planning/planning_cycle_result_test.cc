/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/planning_cycle_result.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(PlanningCycleResultTest, NotStartedDoesNotPublishOutputs) {
  PlanningCycleResult result;

  EXPECT_FALSE(result.ShouldPublishMotion());
  EXPECT_FALSE(result.ShouldPublishTrajectory());
  EXPECT_FALSE(result.ShouldRecordHistory());
}

TEST(PlanningCycleResultTest, InputHoldPublishesStatusWithoutMotionOrHistory) {
  PlanningCycleResult result;
  result.outcome = PlanningCycleOutcome::kInputHold;

  EXPECT_FALSE(result.ShouldPublishMotion());
  EXPECT_TRUE(result.ShouldPublishTrajectory());
  EXPECT_FALSE(result.ShouldRecordHistory());
}

TEST(PlanningCycleResultTest,
     ValidationHoldPublishesMotionAndTrajectoryWithoutHistory) {
  PlanningCycleResult result;
  result.outcome = PlanningCycleOutcome::kValidationHold;

  EXPECT_TRUE(result.ShouldPublishMotion());
  EXPECT_TRUE(result.ShouldPublishTrajectory());
  EXPECT_FALSE(result.ShouldRecordHistory());
}

TEST(PlanningCycleResultTest, PlannedCyclePublishesAndRecordsHistory) {
  PlanningCycleResult result;
  result.outcome = PlanningCycleOutcome::kPlanned;

  EXPECT_TRUE(result.ShouldPublishMotion());
  EXPECT_TRUE(result.ShouldPublishTrajectory());
  EXPECT_TRUE(result.ShouldRecordHistory());
}

TEST(PlanningCycleResultTest, LearningOnlyDoesNotPublishPlanningOutputs) {
  PlanningCycleResult result;
  result.outcome = PlanningCycleOutcome::kLearningOnly;

  EXPECT_FALSE(result.ShouldPublishMotion());
  EXPECT_FALSE(result.ShouldPublishTrajectory());
  EXPECT_FALSE(result.ShouldRecordHistory());
}

}  // namespace planning
}  // namespace apollo
