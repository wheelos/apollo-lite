// Copyright 2025 WheelOS All Rights Reserved.
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

// Created Date: 2025-01-03
// Author: daohu527

#include "modules/planning/common/obstacle_decider.h"

#include "gtest/gtest.h"

#include "modules/map/hdmap/hdmap.h"

#include "modules/common_msgs/prediction_msgs/feature.pb.h"

#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

using apollo::prediction::ObstaclePriority;

namespace {

apollo::hdmap::HDMap* GetTestMap() {
  static apollo::hdmap::HDMap* test_map = [] {
    auto* map = new apollo::hdmap::HDMap();
    map->LoadMapFromFile("/apollo/modules/planning/testdata/garage_map/base_map.txt");
    return map;
  }();
  return test_map;
}

}  // namespace

class ObstacleDeciderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ego_width_ = 2.0;

    adc_sl_.set_start_s(10.0);
    adc_sl_.set_end_s(15.0);

    priority_ = ObstaclePriority::NORMAL;

    BuildReferenceLineFromMap();
  }

  void BuildReferenceLineFromMap() {
    auto lane_info_ptr = GetTestMap()->GetLaneById(apollo::hdmap::MakeMapId("1_-1"));
    ASSERT_NE(lane_info_ptr, nullptr);

    std::vector<ReferencePoint> ref_points;
    const auto& points = lane_info_ptr->points();
    const auto& headings = lane_info_ptr->headings();
    const auto& accumulate_s = lane_info_ptr->accumulate_s();
    for (size_t i = 0; i < points.size(); ++i) {
      std::vector<apollo::hdmap::LaneWaypoint> waypoint;
      waypoint.emplace_back(lane_info_ptr, accumulate_s[i]);
      apollo::hdmap::MapPathPoint map_path_point(points[i], headings[i],
                                                 waypoint);
      ref_points.emplace_back(map_path_point, 0.0, 0.0);
    }

    reference_line_ = std::make_unique<ReferenceLine>(ref_points);
  }

  perception::PerceptionObstacle CreateValidPerception(int id) {
    perception::PerceptionObstacle p;
    p.set_id(id);
    p.set_length(4.0);
    p.set_width(2.0);
    p.mutable_position()->set_x(0.0);
    p.mutable_position()->set_y(0.0);
    p.set_theta(0.0);
    p.set_type(perception::PerceptionObstacle::VEHICLE);
    return p;
  }

  void InjectSL(Obstacle* obs, double start_s, double end_s, double start_l,
                double end_l) {
    SLBoundary slb;
    slb.set_start_s(start_s);
    slb.set_end_s(end_s);
    slb.set_start_l(start_l);
    slb.set_end_l(end_l);
    obs->SetPerceptionSlBoundary(slb);
  }

 protected:
  double ego_width_ = 0.0;
  SLBoundary adc_sl_;
  std::unique_ptr<ReferenceLine> reference_line_;
  ObstaclePriority::Priority priority_;
};

TEST_F(ObstacleDeciderTest, StaticNudgeable) {
  auto p = CreateValidPerception(1);
  Obstacle obs("static_nudge", p, priority_, true);

  InjectSL(&obs, 50.0, 55.0, 0.8, 1.2);

  auto type = ObstacleDecider::ComputeInteractionType(
      obs.PerceptionSLBoundary(), obs, ego_width_, *reference_line_, adc_sl_,
      false);

  EXPECT_EQ(type, InteractionType::NUDGEABLE);
}

TEST_F(ObstacleDeciderTest, StaticBlocking) {
  auto p = CreateValidPerception(2);
  Obstacle obs("static_block", p, priority_, true);

  InjectSL(&obs, 50.0, 55.0, -0.8, 0.8);

  auto type = ObstacleDecider::ComputeInteractionType(
      obs.PerceptionSLBoundary(), obs, ego_width_, *reference_line_, adc_sl_,
      false);

  EXPECT_EQ(type, InteractionType::BLOCKING);
}

TEST_F(ObstacleDeciderTest, DynamicYielding) {
  auto p = CreateValidPerception(3);
  Obstacle obs("dynamic", p, priority_, false);

  InjectSL(&obs, 50.0, 55.0, -0.5, 0.5);

  auto type = ObstacleDecider::ComputeInteractionType(
      obs.PerceptionSLBoundary(), obs, ego_width_, *reference_line_, adc_sl_,
      false);

  EXPECT_EQ(type, InteractionType::YIELDING);
}

}  // namespace planning
}  // namespace apollo
