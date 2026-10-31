#include "modules/planning/scenarios/scenario.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace {

TEST(ScenarioGradeTest, ParentMissionGradeIsNotALocalScenarioGrade) {
  EXPECT_TRUE(IsLocalScenarioGrade(ScenarioGrade::CRUISE));
  EXPECT_TRUE(IsLocalScenarioGrade(ScenarioGrade::MANEUVER));
  EXPECT_TRUE(IsLocalScenarioGrade(ScenarioGrade::CRITICAL));
  EXPECT_FALSE(IsLocalScenarioGrade(ScenarioGrade::MISSION));
}

}  // namespace
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
