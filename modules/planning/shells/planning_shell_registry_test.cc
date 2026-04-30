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

#include "modules/planning/shells/planning_shell_registry.h"

#include <memory>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace planning {
namespace {

class DummyPlanning final : public PlanningBase {
 public:
  DummyPlanning(const std::shared_ptr<DependencyInjector>& injector,
                PlanningMode mode, std::string name)
      : PlanningBase(injector), mode_(mode), name_(std::move(name)) {}

  std::string Name() const override { return name_; }

  PlanningMode Mode() const override { return mode_; }

  void RunOnce(const LocalView& local_view,
               ADCTrajectory* const adc_trajectory) override {
    (void)local_view;
    (void)adc_trajectory;
  }

  common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) override {
    (void)current_time_stamp;
    (void)stitching_trajectory;
    (void)trajectory;
    return common::Status::OK();
  }

 private:
  PlanningMode mode_ = MODE_UNKNOWN;
  std::string name_;
};

TEST(PlanningShellRegistryTest, RegistersShellsAndBuildsAvailability) {
  auto injector = std::make_shared<DependencyInjector>();
  PlanningShellRegistry registry;

  auto on_lane = std::make_unique<DummyPlanning>(injector, MODE_LANE_GRAPH,
                                                 "OnLane");
  auto* on_lane_ptr = on_lane.get();
  EXPECT_TRUE(registry.Register(MODE_LANE_GRAPH, PLANNING_SHELL_ON_LANE,
                                DOMAIN_HDMAP_ROUTED, "guided_route",
                                std::move(on_lane)));

  auto open_space = std::make_unique<DummyPlanning>(injector, MODE_OPEN_SPACE,
                                                    "OpenSpace");
  auto* open_space_ptr = open_space.get();
  EXPECT_TRUE(registry.Register(MODE_OPEN_SPACE, PLANNING_SHELL_OPEN_SPACE,
                                DOMAIN_OPEN_SPACE, "open_space",
                                std::move(open_space)));

  EXPECT_TRUE(registry.HasShell(MODE_LANE_GRAPH));
  EXPECT_TRUE(registry.HasShell(MODE_OPEN_SPACE));
  EXPECT_FALSE(registry.HasShell(MODE_CORRIDOR));

  EXPECT_EQ(registry.GetPlannerForMode(MODE_LANE_GRAPH), on_lane_ptr);
  EXPECT_EQ(registry.GetPlannerForMode(MODE_OPEN_SPACE), open_space_ptr);
  EXPECT_EQ(registry.GetPlannerForMode(MODE_FREE_SPACE), nullptr);

  const auto availability = registry.BuildAvailability();
  EXPECT_TRUE(availability.lane_graph_available);
  EXPECT_FALSE(availability.corridor_available);
  EXPECT_TRUE(availability.open_space_available);
  EXPECT_FALSE(availability.free_space_available);
  EXPECT_FALSE(availability.safety_hold_available);
}

TEST(PlanningShellRegistryTest, RejectsInvalidOrDuplicateRegistrations) {
  auto injector = std::make_shared<DependencyInjector>();
  PlanningShellRegistry registry;

  auto wrong_mode = std::make_unique<DummyPlanning>(injector, MODE_CORRIDOR,
                                                    "WrongMode");
  EXPECT_FALSE(registry.Register(MODE_LANE_GRAPH, PLANNING_SHELL_ON_LANE,
                                 DOMAIN_HDMAP_ROUTED, "invalid",
                                 std::move(wrong_mode)));
  EXPECT_FALSE(registry.HasShell(MODE_LANE_GRAPH));

  auto first = std::make_unique<DummyPlanning>(injector, MODE_FREE_SPACE,
                                               "StructuredMapless");
  EXPECT_TRUE(registry.Register(MODE_FREE_SPACE,
                                PLANNING_SHELL_STRUCTURED_MAPLESS,
                                DOMAIN_STRUCTURED_MAPLESS,
                                "structured_mapless", std::move(first)));

  auto duplicate = std::make_unique<DummyPlanning>(injector, MODE_FREE_SPACE,
                                                   "Duplicate");
  EXPECT_FALSE(registry.Register(MODE_FREE_SPACE,
                                 PLANNING_SHELL_STRUCTURED_MAPLESS,
                                 DOMAIN_STRUCTURED_MAPLESS, "duplicate",
                                 std::move(duplicate)));

  const auto availability = registry.BuildAvailability();
  EXPECT_TRUE(availability.free_space_available);

  registry.Clear();
  EXPECT_FALSE(registry.HasShell(MODE_FREE_SPACE));
}

}  // namespace
}  // namespace planning
}  // namespace apollo
