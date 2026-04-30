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

#include "modules/planning/planner/planner_selector.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(PlannerSelectorTest, CreatesPublicRoadForStandardShell) {
  PlanningConfig config;
  config.mutable_standard_planning_config()->add_planner_type(PUBLIC_ROAD);
  config.mutable_standard_planning_config()->mutable_planner_public_road_config();

  auto injector = std::make_shared<DependencyInjector>();
  std::unique_ptr<Planner> planner;
  const auto status =
      PlannerSelector::CreateStandardPlanner(config, injector, &planner);

  ASSERT_TRUE(status.ok());
  ASSERT_NE(planner, nullptr);
  EXPECT_EQ(planner->Name(), "PUBLIC_ROAD");
}

TEST(PlannerSelectorTest, CreatesNaviForNavigationShell) {
  PlanningConfig config;
  config.mutable_navigation_planning_config()->add_planner_type(NAVI);

  auto injector = std::make_shared<DependencyInjector>();
  std::unique_ptr<Planner> planner;
  const auto status =
      PlannerSelector::CreateNavigationPlanner(config, injector, &planner);

  ASSERT_TRUE(status.ok());
  ASSERT_NE(planner, nullptr);
  EXPECT_EQ(planner->Name(), "NAVI");
}

TEST(PlannerSelectorTest, RejectsNaviInStandardShell) {
  PlanningConfig config;
  config.mutable_standard_planning_config()->add_planner_type(NAVI);

  auto injector = std::make_shared<DependencyInjector>();
  std::unique_ptr<Planner> planner;
  const auto status =
      PlannerSelector::CreateStandardPlanner(config, injector, &planner);

  EXPECT_FALSE(status.ok());
  EXPECT_EQ(planner, nullptr);
}

TEST(PlannerSelectorTest, AllowsLegacyRtkInNavigationShell) {
  PlanningConfig config;
  config.mutable_navigation_planning_config()->add_planner_type(RTK);

  auto injector = std::make_shared<DependencyInjector>();
  std::unique_ptr<Planner> planner;
  const auto status =
      PlannerSelector::CreateNavigationPlanner(config, injector, &planner);

  ASSERT_TRUE(status.ok());
  ASSERT_NE(planner, nullptr);
  EXPECT_EQ(planner->Name(), "RTK");
}

}  // namespace planning
}  // namespace apollo
