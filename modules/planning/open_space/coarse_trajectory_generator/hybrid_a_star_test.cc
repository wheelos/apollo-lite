/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/*
 * @file
 */

#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"

#include <cmath>

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;

namespace {

double LastMovingVelocity(const HybridAStartResult& result) {
  for (auto iter = result.v.rbegin(); iter != result.v.rend(); ++iter) {
    if (std::abs(*iter) > 1e-3) {
      return *iter;
    }
  }
  return 0.0;
}

double PathLength(const HybridAStartResult& result) {
  double length = 0.0;
  for (std::size_t index = 1; index < result.x.size(); ++index) {
    length += std::hypot(result.x[index] - result.x[index - 1],
                         result.y[index] - result.y[index - 1]);
  }
  return length;
}

int GearSwitchCount(const HybridAStartResult& result) {
  int switches = 0;
  int previous_sign = 0;
  for (const double v : result.v) {
    if (std::abs(v) <= 1e-3) {
      continue;
    }
    const int sign = v > 0.0 ? 1 : -1;
    if (previous_sign != 0 && previous_sign != sign) {
      ++switches;
    }
    previous_sign = sign;
  }
  return switches;
}

}  // namespace

class HybridATest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_planner_open_space_config_filename =
        "/apollo/modules/planning/testdata/conf/"
        "open_space_standard_parking_lot.pb.txt";

    ACHECK(apollo::cyber::common::GetProtoFromFile(
        FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
        << "Failed to load open space config file "
        << FLAGS_planner_open_space_config_filename;

    hybrid_test = std::unique_ptr<HybridAStar>(
        new HybridAStar(planner_open_space_config_));
  }

 protected:
  std::unique_ptr<HybridAStar> hybrid_test;
  PlannerOpenSpaceConfig planner_open_space_config_;
};

TEST_F(HybridATest, test1) {
  double sx = -15.0;
  double sy = 0.0;
  double sphi = 0.0;
  double ex = 15.0;
  double ey = 0.0;
  double ephi = 0.0;
  std::vector<std::vector<Vec2d>> obstacles_list;
  HybridAStartResult result;
  Vec2d obstacle_vertice_a(1.0, 0.0);
  Vec2d obstacle_vertice_b(-1.0, 0.0);
  std::vector<Vec2d> obstacle = {obstacle_vertice_a, obstacle_vertice_b};
  // load xy boundary into the Plan() from configuration(Independent from frame)
  std::vector<double> XYbounds_;
  XYbounds_.push_back(-50.0);
  XYbounds_.push_back(50.0);
  XYbounds_.push_back(-50.0);
  XYbounds_.push_back(50.0);

  obstacles_list.emplace_back(obstacle);
  ASSERT_TRUE(hybrid_test->Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_,
                                obstacles_list, &result));
}

TEST_F(HybridATest, SanMateo3111PoseIsKinematicallyReachableWithoutRoiWalls) {
  const double sx = -2.12098;
  const double sy = 2.51464;
  const double sphi = -0.037544;
  const double ex = -0.0346301;
  const double ey = 1.49188;
  const double ephi = 1.95494;
  const std::vector<double> xy_bounds = {-4.77976, 22.7353, -0.609046, 17.5657};
  const std::vector<std::vector<Vec2d>> obstacles_list;

  HybridAStartResult result;
  ASSERT_TRUE(hybrid_test->Plan(sx, sy, sphi, ex, ey, ephi, xy_bounds,
                                obstacles_list, &result));
}

TEST_F(HybridATest, SanMateo3111PoseIsReachableWithRectangularRoiBoundary) {
  const double sx = -2.12098;
  const double sy = 2.51464;
  const double sphi = -0.037544;
  const double ex = -0.0346301;
  const double ey = 1.49188;
  const double ephi = 1.95494;
  const std::vector<double> xy_bounds = {-4.77976, 22.7353, -0.609046, 17.5657};
  const std::vector<std::vector<Vec2d>> obstacles_list = {
      {Vec2d(-4.77976, -0.609046), Vec2d(22.7353, -0.609046),
       Vec2d(22.7353, 17.5657), Vec2d(-4.77976, 17.5657),
       Vec2d(-4.77976, -0.609046)}};

  HybridAStartResult result;
  ASSERT_TRUE(hybrid_test->Plan(sx, sy, sphi, ex, ey, ephi, xy_bounds,
                                obstacles_list, &result));
}

TEST_F(HybridATest, RequiredReverseFinalGearProducesTailInApproach) {
  const std::vector<double> xy_bounds = {-20.0, 20.0, -20.0, 20.0};
  const std::vector<std::vector<Vec2d>> obstacles_list;

  HybridAStartResult result;
  ASSERT_TRUE(hybrid_test->Plan(0.0, 8.0, -M_PI_2, 0.0, 0.0, M_PI_2,
                                xy_bounds, obstacles_list, &result, true,
                                false));
  EXPECT_LT(LastMovingVelocity(result), 0.0);
}

TEST_F(HybridATest, SunnyvaleTailInAvoidsExcessiveBackAndForth) {
  const std::vector<double> xy_bounds = {-22.7147, 6.35191, -14.237, 5.10005};
  const std::vector<std::vector<Vec2d>> obstacles_list = {
      {Vec2d(-22.7147, -13.6801), Vec2d(6.10791, -14.237),
       Vec2d(6.35191, -0.37518), Vec2d(6.10196, -0.370351),
       Vec2d(1.30025, -0.27757), Vec2d(1.34366, 2.18855),
       Vec2d(1.33458, 2.18872), Vec2d(1.38494, 5.04969),
       Vec2d(-1.22145, 5.10005), Vec2d(-1.27181, 2.23908),
       Vec2d(-1.28089, 2.23926), Vec2d(-1.3243, -0.226858),
       Vec2d(-22.2208, 0.17691), Vec2d(-22.4707, 0.18174),
       Vec2d(-22.7147, -13.6801)}};

  HybridAStartResult result;
  ASSERT_TRUE(hybrid_test->Plan(-17.4291, -6.46302, -0.0350393, 0.0690805,
                                3.92447, -1.5884, xy_bounds, obstacles_list,
                                &result));
  EXPECT_LE(GearSwitchCount(result), 2);
  EXPECT_LT(PathLength(result), 80.0);
}
}  // namespace planning
}  // namespace apollo
