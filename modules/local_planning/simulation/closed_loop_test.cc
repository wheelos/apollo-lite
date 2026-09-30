// Copyright 2026 WheelOS All Rights Reserved.
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

#include "modules/local_planning/simulation/closed_loop.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "gtest/gtest.h"

namespace apollo {
namespace local_planning {
namespace {

TEST(LocalPlanningSimulationTest, KinematicScenarios) {
  for (const auto* scenario :
       {"straight", "curve", "obstacle", "moving_obstacle", "stale", "epoch",
        "odom_loss", "temporal_noise", "occlusion", "delayed", "odom_bias"}) {
    SCOPED_TRACE(scenario);
    SimulationOptions options;
    options.scenario = scenario;
    SimulationReport report;
    const auto status = RunClosedLoop(options, &report);
    ASSERT_TRUE(status.ok()) << status.ToString();
    EXPECT_TRUE(report.passed);
    EXPECT_EQ(1500U, report.samples.size());
    EXPECT_LE(report.max_lateral_error, 0.5);
    EXPECT_LE(report.final_speed, 0.1);
    EXPECT_GT(report.stitched_cycles, 20);
    EXPECT_GT(report.lane_updates, 250);
  }
}

TEST(LocalPlanningSimulationTest, MujocoScenarios) {
  for (const auto* scenario :
       {"straight", "curve", "obstacle", "moving_obstacle", "stale", "epoch",
        "odom_loss", "temporal_noise", "occlusion", "delayed", "odom_bias"}) {
    SCOPED_TRACE(scenario);
    SimulationOptions options;
    options.backend = "mujoco";
    options.scenario = scenario;
    SimulationReport report;
    const auto status = RunClosedLoop(options, &report);
    ASSERT_TRUE(status.ok()) << status.ToString();
    EXPECT_TRUE(report.passed);
    EXPECT_GT(report.progress, 2.0);
    EXPECT_LE(report.max_lateral_error, 0.5);
    EXPECT_GT(report.stitched_cycles, 20);
  }
}

TEST(LocalPlanningSimulationTest, RejectsInvalidOptionsAndWritesArtifacts) {
  SimulationOptions options;
  SimulationReport report;
  options.backend = "not-a-backend";
  EXPECT_FALSE(RunClosedLoop(options, &report).ok());
  options.backend = "kinematic";
  options.scenario = "not-a-scenario";
  EXPECT_FALSE(RunClosedLoop(options, &report).ok());
  EXPECT_FALSE(RunClosedLoop(options, nullptr).ok());
  EXPECT_FALSE(WriteSimulationArtifacts("", report).ok());
  report.samples.push_back({1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false});
  report.centerline = {{0.0, 0.0}, {1.0, 0.0}};
  const char* tmp = std::getenv("TEST_TMPDIR");
  ASSERT_NE(nullptr, tmp);
  const std::string prefix = std::string(tmp) + "/local_planning_artifacts";
  ASSERT_TRUE(WriteSimulationArtifacts(prefix, report).ok());
  std::ifstream csv(prefix + ".csv");
  std::string header;
  std::getline(csv, header);
  EXPECT_EQ(
      "time,truth_x,truth_y,odom_x,odom_y,speed,lateral_error,trajectory_valid",
      header);
  csv.close();
  std::ifstream svg(prefix + ".svg");
  std::getline(svg, header);
  EXPECT_EQ(0U, header.find("<svg"));
  svg.close();
  EXPECT_EQ(0, std::remove((prefix + ".csv").c_str()));
  EXPECT_EQ(0, std::remove((prefix + ".svg").c_str()));
}

}  // namespace
}  // namespace local_planning
}  // namespace apollo
