/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/map/tools/bidirectional_lane_converter.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace hdmap {
namespace tools {
namespace {

void AddPoint(LineSegment* line, double x, double y) {
  auto* point = line->add_point();
  point->set_x(x);
  point->set_y(y);
  point->set_z(0.0);
}

void AddCurveSegment(Curve* curve, double start_s, double length,
                     double x1, double y1, double x2, double y2) {
  auto* segment = curve->add_segment();
  segment->set_s(start_s);
  segment->set_length(length);
  segment->set_heading(y2 == y1 ? (x2 > x1 ? 0.0 : 3.141592653589793)
                                : (y2 > y1 ? 1.570796326794897
                                           : -1.570796326794897));
  auto* start = segment->mutable_start_position();
  start->set_x(x1);
  start->set_y(y1);
  AddPoint(segment->mutable_line_segment(), x1, y1);
  AddPoint(segment->mutable_line_segment(), x2, y2);
}

void AddLaneBoundary(LaneBoundary* boundary, double y_offset,
                     LaneBoundaryType::Type first_type,
                     LaneBoundaryType::Type second_type) {
  boundary->set_length(3.0);
  AddCurveSegment(boundary->mutable_curve(), 0.0, 1.0, 0.0, y_offset, 1.0,
                  y_offset);
  AddCurveSegment(boundary->mutable_curve(), 1.0, 2.0, 1.0, y_offset, 1.0,
                  y_offset + 2.0);
  auto* first = boundary->add_boundary_type();
  first->set_s(0.0);
  first->add_types(first_type);
  auto* second = boundary->add_boundary_type();
  second->set_s(2.0);
  second->add_types(second_type);
}

Lane* AddBidirectionalLane(Map* map, const std::string& lane_id,
                           double start_x) {
  auto* lane = map->add_lane();
  lane->mutable_id()->set_id(lane_id);
  lane->set_direction(Lane::BIDIRECTION);
  lane->set_type(Lane::CITY_DRIVING);
  lane->set_turn(Lane::LEFT_TURN);
  lane->set_length(3.0);
  AddCurveSegment(lane->mutable_central_curve(), 0.0, 1.0, start_x, 0.0,
                  start_x + 1.0, 0.0);
  AddCurveSegment(lane->mutable_central_curve(), 1.0, 2.0, start_x + 1.0, 0.0,
                  start_x + 1.0, 2.0);
  AddLaneBoundary(lane->mutable_left_boundary(), 1.0,
                  LaneBoundaryType::DOTTED_WHITE,
                  LaneBoundaryType::SOLID_WHITE);
  AddLaneBoundary(lane->mutable_right_boundary(), -1.0,
                  LaneBoundaryType::SOLID_YELLOW,
                  LaneBoundaryType::DOTTED_YELLOW);
  auto* left_sample = lane->add_left_sample();
  left_sample->set_s(0.5);
  left_sample->set_width(1.1);
  auto* right_sample = lane->add_right_sample();
  right_sample->set_s(2.5);
  right_sample->set_width(0.9);
  auto* left_road_sample = lane->add_left_road_sample();
  left_road_sample->set_s(1.0);
  left_road_sample->set_width(2.0);
  auto* right_road_sample = lane->add_right_road_sample();
  right_road_sample->set_s(2.0);
  right_road_sample->set_width(2.5);
  return lane;
}

void AddRoadSection(Map* map, const std::string& road_id,
                    const std::vector<std::string>& lane_ids) {
  auto* road = map->add_road();
  road->mutable_id()->set_id(road_id);
  auto* section = road->add_section();
  section->mutable_id()->set_id(road_id + "_section");
  for (const auto& lane_id : lane_ids) {
    section->add_lane_id()->set_id(lane_id);
  }
}

void AddLaneOverlap(Map* map, Lane* lane, double start_s, double end_s) {
  auto* overlap = map->add_overlap();
  overlap->mutable_id()->set_id("overlap_" + lane->id().id());
  lane->add_overlap_id()->set_id(overlap->id().id());
  auto* object = overlap->add_object();
  object->mutable_id()->set_id(lane->id().id());
  object->mutable_lane_overlap_info()->set_start_s(start_s);
  object->mutable_lane_overlap_info()->set_end_s(end_s);
}

TEST(BidirectionalLaneConverterTest, ReversesGeometryWidthsAndReferences) {
  Map map;
  auto* lane = AddBidirectionalLane(&map, "lane_a", 0.0);
  AddRoadSection(&map, "road_a", {"lane_a"});
  AddLaneOverlap(&map, lane, 0.5, 1.5);

  std::string error;
  ASSERT_TRUE(ConvertBidirectionalLanes(&map, &error)) << error;
  ASSERT_EQ(map.lane_size(), 2);

  const auto& forward = map.lane(0);
  const auto& reverse = map.lane(1);
  EXPECT_EQ(forward.id().id(), "lane_a");
  EXPECT_EQ(forward.direction(), Lane::FORWARD);
  ASSERT_EQ(forward.self_reverse_lane_id_size(), 1);
  EXPECT_EQ(forward.self_reverse_lane_id(0).id(), "lane_a_reverse");
  EXPECT_EQ(reverse.id().id(), "lane_a_reverse");
  EXPECT_EQ(reverse.direction(), Lane::BACKWARD);
  EXPECT_EQ(reverse.turn(), Lane::RIGHT_TURN);
  ASSERT_EQ(reverse.self_reverse_lane_id_size(), 1);
  EXPECT_EQ(reverse.self_reverse_lane_id(0).id(), "lane_a");

  ASSERT_EQ(reverse.central_curve().segment_size(), 2);
  const auto& first_segment = reverse.central_curve().segment(0);
  EXPECT_DOUBLE_EQ(first_segment.s(), 0.0);
  EXPECT_DOUBLE_EQ(first_segment.length(), 2.0);
  EXPECT_DOUBLE_EQ(first_segment.line_segment().point(0).x(), 1.0);
  EXPECT_DOUBLE_EQ(first_segment.line_segment().point(0).y(), 2.0);
  EXPECT_DOUBLE_EQ(first_segment.line_segment().point(1).x(), 1.0);
  EXPECT_DOUBLE_EQ(first_segment.line_segment().point(1).y(), 0.0);
  EXPECT_NEAR(first_segment.heading(), -1.570796326794897, 1e-12);
  EXPECT_DOUBLE_EQ(first_segment.start_position().x(), 1.0);
  EXPECT_DOUBLE_EQ(first_segment.start_position().y(), 2.0);
  const auto& second_segment = reverse.central_curve().segment(1);
  EXPECT_DOUBLE_EQ(second_segment.s(), 2.0);
  EXPECT_DOUBLE_EQ(second_segment.length(), 1.0);
  EXPECT_DOUBLE_EQ(second_segment.line_segment().point(0).x(), 1.0);
  EXPECT_DOUBLE_EQ(second_segment.line_segment().point(1).x(), 0.0);

  ASSERT_TRUE(reverse.has_left_boundary());
  ASSERT_EQ(reverse.left_boundary().curve().segment_size(), 2);
  EXPECT_DOUBLE_EQ(
      reverse.left_boundary().curve().segment(0).line_segment().point(0).y(),
      1.0);
  ASSERT_EQ(reverse.left_boundary().boundary_type_size(), 2);
  EXPECT_DOUBLE_EQ(reverse.left_boundary().boundary_type(0).s(), 0.0);
  EXPECT_EQ(reverse.left_boundary().boundary_type(0).types(0),
            LaneBoundaryType::DOTTED_YELLOW);
  EXPECT_DOUBLE_EQ(reverse.left_boundary().boundary_type(1).s(), 1.0);
  EXPECT_EQ(reverse.left_boundary().boundary_type(1).types(0),
            LaneBoundaryType::SOLID_YELLOW);

  ASSERT_EQ(reverse.left_sample_size(), 3);
  EXPECT_DOUBLE_EQ(reverse.left_sample(0).s(), 0.0);
  EXPECT_DOUBLE_EQ(reverse.left_sample(0).width(), 0.9);
  EXPECT_DOUBLE_EQ(reverse.left_sample(1).s(), 0.5);
  EXPECT_DOUBLE_EQ(reverse.left_sample(2).s(), 3.0);
  ASSERT_EQ(reverse.right_sample_size(), 3);
  EXPECT_DOUBLE_EQ(reverse.right_sample(0).s(), 0.0);
  EXPECT_DOUBLE_EQ(reverse.right_sample(1).s(), 2.5);
  EXPECT_DOUBLE_EQ(reverse.right_sample(2).s(), 3.0);
  EXPECT_DOUBLE_EQ(reverse.right_sample(0).width(), 1.1);
  ASSERT_EQ(reverse.left_road_sample_size(), 3);
  EXPECT_DOUBLE_EQ(reverse.left_road_sample(1).s(), 1.0);
  EXPECT_DOUBLE_EQ(reverse.left_road_sample(0).width(), 2.5);
  ASSERT_EQ(reverse.right_road_sample_size(), 3);
  EXPECT_DOUBLE_EQ(reverse.right_road_sample(1).s(), 2.0);
  EXPECT_DOUBLE_EQ(reverse.right_road_sample(0).width(), 2.0);

  ASSERT_EQ(map.road(0).section(0).lane_id_size(), 2);
  EXPECT_EQ(map.road(0).section(0).lane_id(1).id(), "lane_a_reverse");
  ASSERT_EQ(map.overlap(0).object_size(), 2);
  EXPECT_EQ(map.overlap(0).object(1).id().id(), "lane_a_reverse");
  EXPECT_DOUBLE_EQ(
      map.overlap(0).object(1).lane_overlap_info().start_s(), 1.5);
  EXPECT_DOUBLE_EQ(map.overlap(0).object(1).lane_overlap_info().end_s(), 2.5);
}

TEST(BidirectionalLaneConverterTest, ReversesConnectionsWithoutUTurnEdges) {
  Map map;
  AddBidirectionalLane(&map, "lane_a", 0.0);
  AddBidirectionalLane(&map, "lane_b", 2.0);
  map.mutable_lane(0)->add_successor_id()->set_id("lane_b");
  map.mutable_lane(0)->add_left_neighbor_forward_lane_id()->set_id("lane_b");
  map.mutable_lane(1)->add_predecessor_id()->set_id("lane_a");
  AddRoadSection(&map, "road_a", {"lane_a", "lane_b"});

  std::string error;
  ASSERT_TRUE(ConvertBidirectionalLanes(&map, &error)) << error;
  ASSERT_EQ(map.lane_size(), 4);
  const auto& reverse_first = map.lane(2);
  const auto& reverse_second = map.lane(3);
  EXPECT_EQ(reverse_first.id().id(), "lane_a_reverse");
  EXPECT_EQ(reverse_second.id().id(), "lane_b_reverse");
  ASSERT_EQ(reverse_first.right_neighbor_forward_lane_id_size(), 1);
  EXPECT_EQ(reverse_first.right_neighbor_forward_lane_id(0).id(),
            "lane_b_reverse");
  EXPECT_EQ(reverse_first.successor_id_size(), 0);
  ASSERT_EQ(reverse_second.successor_id_size(), 1);
  EXPECT_EQ(reverse_second.successor_id(0).id(), "lane_a_reverse");
  ASSERT_EQ(reverse_first.predecessor_id_size(), 1);
  EXPECT_EQ(reverse_first.predecessor_id(0).id(), "lane_b_reverse");
  EXPECT_EQ(map.lane(0).successor_id(0).id(), "lane_b");
}

TEST(BidirectionalLaneConverterTest, UnconvertedNeighborRemainsOpposite) {
  Map map;
  AddBidirectionalLane(&map, "lane_a", 0.0);
  auto* neighbor = map.add_lane();
  neighbor->mutable_id()->set_id("one_way");
  auto* source = map.mutable_lane(0);
  source->add_left_neighbor_forward_lane_id()->set_id("one_way");
  source->add_right_neighbor_reverse_lane_id()->set_id("one_way");
  AddRoadSection(&map, "road_a", {"lane_a", "one_way"});

  std::string error;
  ASSERT_TRUE(ConvertBidirectionalLanes(&map, &error)) << error;
  const auto& reverse = map.lane(2);
  ASSERT_EQ(reverse.right_neighbor_reverse_lane_id_size(), 1);
  EXPECT_EQ(reverse.right_neighbor_reverse_lane_id(0).id(), "one_way");
  ASSERT_EQ(reverse.left_neighbor_forward_lane_id_size(), 1);
  EXPECT_EQ(reverse.left_neighbor_forward_lane_id(0).id(), "one_way");
}

TEST(BidirectionalLaneConverterTest,
     OppositeNeighborBecomesSameDirectionWithoutChangingItsId) {
  Map map;
  AddBidirectionalLane(&map, "lane_a", 0.0);
  AddBidirectionalLane(&map, "lane_b", 2.0);
  map.mutable_lane(0)->add_right_neighbor_reverse_lane_id()->set_id("lane_b");
  AddRoadSection(&map, "road_a", {"lane_a", "lane_b"});

  std::string error;
  ASSERT_TRUE(ConvertBidirectionalLanes(&map, &error)) << error;
  const auto& reverse_lane = map.lane(2);
  ASSERT_EQ(reverse_lane.left_neighbor_forward_lane_id_size(), 1);
  EXPECT_EQ(reverse_lane.left_neighbor_forward_lane_id(0).id(), "lane_b");
  EXPECT_EQ(reverse_lane.left_neighbor_reverse_lane_id_size(), 0);
}

TEST(BidirectionalLaneConverterTest, RejectsCollisionAndUnsupportedGeometry) {
  Map collision_map;
  AddBidirectionalLane(&collision_map, "lane_a", 0.0);
  AddBidirectionalLane(&collision_map, "lane_a_reverse", 4.0);
  AddRoadSection(&collision_map, "road_a",
                 {"lane_a", "lane_a_reverse"});
  const std::string original_collision_map =
      collision_map.SerializeAsString();
  std::string error;
  EXPECT_FALSE(ConvertBidirectionalLanes(&collision_map, &error));
  EXPECT_NE(error.find("collides"), std::string::npos);
  EXPECT_EQ(collision_map.SerializeAsString(), original_collision_map);

  Map unsupported_map;
  auto* lane = AddBidirectionalLane(&unsupported_map, "lane_b", 0.0);
  lane->mutable_central_curve()->clear_segment();
  lane->mutable_central_curve()->add_segment();
  AddRoadSection(&unsupported_map, "road_b", {"lane_b"});
  const std::string original_unsupported_map =
      unsupported_map.SerializeAsString();
  EXPECT_FALSE(ConvertBidirectionalLanes(&unsupported_map, &error));
  EXPECT_NE(error.find("unsupported"), std::string::npos);
  EXPECT_EQ(unsupported_map.SerializeAsString(), original_unsupported_map);
}

}  // namespace
}  // namespace tools
}  // namespace hdmap
}  // namespace apollo
