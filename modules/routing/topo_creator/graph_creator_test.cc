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

#include <cstdlib>
#include <filesystem>
#include <string>
#include <unordered_set>

#include "cyber/common/file.h"
#include "gtest/gtest.h"

using apollo::hdmap::Lane;
using apollo::routing::GraphCreator;

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

TEST(GraphCreatorTest, CreatesDistinctDirectionalNodesWithoutUTurnEdges) {
  char temp_dir[] = "/tmp/apollo_bidirectional_topo_test_XXXXXX";
  ASSERT_NE(mkdtemp(temp_dir), nullptr);
  const std::filesystem::path directory(temp_dir);
  const auto map_path = directory / "base_map.bin";
  const auto topo_path = directory / "routing_map.bin";

  apollo::hdmap::Map map;
  auto* road = map.add_road();
  road->mutable_id()->set_id("road");
  auto* section = road->add_section();
  section->mutable_id()->set_id("section");

  const auto add_lane = [&](const std::string& id, double x,
                            apollo::hdmap::Lane::LaneDirection direction) {
    auto* lane = map.add_lane();
    lane->mutable_id()->set_id(id);
    lane->set_type(apollo::hdmap::Lane::CITY_DRIVING);
    lane->set_direction(direction);
    lane->set_length(1.0);
    auto* segment = lane->mutable_central_curve()->add_segment();
    segment->set_s(0.0);
    segment->set_length(1.0);
    segment->set_heading(0.0);
    segment->mutable_start_position()->set_x(x);
    auto* first = segment->mutable_line_segment()->add_point();
    first->set_x(x);
    auto* last = segment->mutable_line_segment()->add_point();
    last->set_x(x + 1.0);
    section->add_lane_id()->set_id(id);
  };

  add_lane("lane_a", 0.0, apollo::hdmap::Lane::FORWARD);
  add_lane("lane_b", 1.0, apollo::hdmap::Lane::FORWARD);
  add_lane("lane_a_reverse", 1.0, apollo::hdmap::Lane::BACKWARD);
  add_lane("lane_b_reverse", 0.0, apollo::hdmap::Lane::BACKWARD);
  map.mutable_lane(0)->add_successor_id()->set_id("lane_b");
  map.mutable_lane(1)->add_predecessor_id()->set_id("lane_a");
  map.mutable_lane(3)->add_successor_id()->set_id("lane_a_reverse");
  map.mutable_lane(2)->add_predecessor_id()->set_id("lane_b_reverse");

  ASSERT_TRUE(
      apollo::cyber::common::SetProtoToBinaryFile(map, map_path.string()));
  apollo::routing::RoutingConfig routing_config;
  routing_config.set_base_speed(10.0);
  routing_config.set_base_changing_length(10.0);
  routing_config.set_change_penalty(1.0);

  GraphCreator creator(map_path.string(), topo_path.string(), routing_config);
  ASSERT_TRUE(creator.Create());
  apollo::routing::Graph graph;
  ASSERT_TRUE(
      apollo::cyber::common::GetProtoFromFile(topo_path.string(), &graph));

  ASSERT_EQ(graph.node_size(), 4);
  std::unordered_set<std::string> lane_ids;
  for (const auto& node : graph.node()) {
    lane_ids.insert(node.lane_id());
  }
  EXPECT_EQ(lane_ids.size(), 4);
  EXPECT_TRUE(lane_ids.count("lane_a"));
  EXPECT_TRUE(lane_ids.count("lane_a_reverse"));

  bool has_forward_edge = false;
  bool has_reverse_edge = false;
  for (const auto& edge : graph.edge()) {
    if (edge.from_lane_id() == "lane_a" && edge.to_lane_id() == "lane_b") {
      has_forward_edge = true;
    }
    if (edge.from_lane_id() == "lane_b_reverse" &&
        edge.to_lane_id() == "lane_a_reverse") {
      has_reverse_edge = true;
    }
    EXPECT_FALSE(edge.from_lane_id() == "lane_a" &&
                 edge.to_lane_id() == "lane_a_reverse");
    EXPECT_FALSE(edge.from_lane_id() == "lane_a_reverse" &&
                 edge.to_lane_id() == "lane_a");
  }
  EXPECT_TRUE(has_forward_edge);
  EXPECT_TRUE(has_reverse_edge);

  std::filesystem::remove_all(directory);
}
