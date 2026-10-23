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

#include "modules/routing/topo_creator/graph_creator.h"

#include <cstdio>

#include "gtest/gtest.h"

#include "cyber/common/file.h"

using apollo::hdmap::Lane;
using apollo::routing::GraphCreator;

TEST(GraphCreatorTest, PreservesMapLaneDirection) {
  apollo::hdmap::Map map;
  auto* lane = map.add_lane();
  lane->mutable_id()->set_id("bidirectional");
  lane->set_type(Lane::CITY_DRIVING);
  lane->set_direction(Lane::BIDIRECTION);
  auto* curve = lane->mutable_central_curve()->add_segment();
  curve->set_length(10.0);
  curve->mutable_line_segment()->add_point()->set_x(0.0);
  curve->mutable_line_segment()->add_point()->set_x(10.0);
  for (auto& point : *curve->mutable_line_segment()->mutable_point()) {
    point.set_y(0.0);
  }
  const auto input = ::testing::TempDir() + "bidirectional_map.pb.txt";
  const auto output = ::testing::TempDir() + "bidirectional_topo.txt";
  ASSERT_TRUE(apollo::cyber::common::SetProtoToASCIIFile(map, input));
  apollo::routing::RoutingConfig config;
  config.set_base_speed(10.0);
  GraphCreator creator(input, output, config);
  ASSERT_TRUE(creator.Create());
  apollo::routing::Graph graph;
  ASSERT_TRUE(apollo::cyber::common::GetProtoFromFile(output, &graph));
  ASSERT_EQ(graph.node_size(), 1);
  EXPECT_EQ(graph.node(0).direction(), Lane::BIDIRECTION);
  ASSERT_TRUE(creator.Create());
  EXPECT_EQ(creator.graph_.node_size(), 1);
  EXPECT_EQ(std::remove(input.c_str()), 0);
  EXPECT_EQ(std::remove(output.c_str()), 0);
  const auto binary = ::testing::TempDir() + "bidirectional_topo.bin";
  EXPECT_EQ(std::remove(binary.c_str()), 0);
}

TEST(GraphCreatorTest, IsValidUTurn) {
  const double min_turn_radius = 6.0;
  Lane lane;
  lane.mutable_id()->set_id("lane1");
  lane.set_turn(Lane::LEFT_TURN);
  // left_turn is not a u-turn, return false
  EXPECT_FALSE(GraphCreator::IsValidUTurn(lane, min_turn_radius));

  lane.set_turn(Lane::U_TURN);

  auto* segment = lane.mutable_central_curve()->add_segment();
  auto* line_segment = segment->mutable_line_segment();
  {
    // a straight line case
    line_segment->clear_point();
    auto* p1 = line_segment->add_point();
    p1->set_x(-1.0);
    p1->set_y(0.0);
    auto* p2 = line_segment->add_point();
    p2->set_x(0.0);
    p2->set_y(0.0);
    auto* p3 = line_segment->add_point();
    p3->set_x(1.0);
    p3->set_y(0.0);
    EXPECT_TRUE(GraphCreator::IsValidUTurn(lane, min_turn_radius));
  }
  {
    // a narrow one
    line_segment->clear_point();
    auto* p1 = line_segment->add_point();
    p1->set_x(-5.0);
    p1->set_y(5.0);
    auto* p2 = line_segment->add_point();
    p2->set_x(0.0);
    p2->set_y(0.0);
    auto* p3 = line_segment->add_point();
    p3->set_x(5.0);
    p3->set_y(5.0);
    EXPECT_FALSE(GraphCreator::IsValidUTurn(lane, min_turn_radius));
  }
  {
    // a wide one
    line_segment->clear_point();
    auto* p1 = line_segment->add_point();
    p1->set_x(-10.0);
    p1->set_y(10.0);
    auto* p2 = line_segment->add_point();
    p2->set_x(0.0);
    p2->set_y(0.0);
    auto* p3 = line_segment->add_point();
    p3->set_x(10.0);
    p3->set_y(10.0);
    EXPECT_TRUE(GraphCreator::IsValidUTurn(lane, min_turn_radius));
  }
}
