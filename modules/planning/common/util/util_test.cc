/******************************************************************************
 * Copyright 2025 WheelOS All Rights Reserved.
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

#include "modules/planning/common/util/util.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {
namespace util {
namespace {

using apollo::routing::RoutingResponse;

RoutingResponse MakeRoutingResponse(int seq_num, const std::string& road_id,
                                    const std::string& parking_space_id) {
  RoutingResponse routing;
  routing.mutable_header()->set_sequence_num(seq_num);
  auto* request = routing.mutable_routing_request();
  request->mutable_header()->set_sequence_num(seq_num);
  auto* start = request->add_waypoint();
  start->set_id("start_lane");
  start->set_s(1.0);
  auto* end = request->add_waypoint();
  end->set_id("end_lane");
  end->set_s(2.0);
  request->mutable_parking_info()->set_parking_space_id(parking_space_id);
  routing.add_road()->set_id(road_id);
  return routing;
}

TEST(IsDifferentRoutingTest, IgnoresHeaderOnlyChanges) {
  const auto first = MakeRoutingResponse(1, "road_a", "slot_a");
  const auto second = MakeRoutingResponse(2, "road_a", "slot_a");

  EXPECT_FALSE(IsDifferentRouting(first, second));
}

TEST(IsDifferentRoutingTest, DetectsRouteContentChanges) {
  const auto first = MakeRoutingResponse(1, "road_a", "slot_a");
  const auto second = MakeRoutingResponse(2, "road_b", "slot_a");

  EXPECT_TRUE(IsDifferentRouting(first, second));
}

TEST(IsDifferentRoutingTest, DetectsParkingCommandChanges) {
  const auto first = MakeRoutingResponse(1, "road_a", "slot_a");
  const auto second = MakeRoutingResponse(2, "road_a", "slot_b");

  EXPECT_TRUE(IsDifferentRouting(first, second));
}

TEST(HasSameRoutingRequestTest, IgnoresResponseRouteDifferences) {
  const auto first = MakeRoutingResponse(1, "road_a", "slot_a");
  const auto second = MakeRoutingResponse(2, "road_b", "slot_a");

  EXPECT_TRUE(HasSameRoutingRequest(first, second));
}

TEST(HasSameRoutingRequestTest, DetectsWaypointChanges) {
  auto first = MakeRoutingResponse(1, "road_a", "slot_a");
  auto second = MakeRoutingResponse(2, "road_a", "slot_a");
  second.mutable_routing_request()->mutable_waypoint(0)->set_s(3.0);

  EXPECT_FALSE(HasSameRoutingRequest(first, second));
}

TEST(DirectValetParkingCommandTest, RequiresParkingSpaceIdForDirectMode) {
  ScenarioConfig config;
  config.add_stage_type(StageType::VALET_PARKING_PARKING);

  auto routing_with_id =
      std::make_shared<RoutingResponse>(MakeRoutingResponse(1, "road_a", "slot_a"));
  EXPECT_TRUE(HasParkingRoutingCommand(*routing_with_id));
  EXPECT_TRUE(HasParkingSpaceIdRoutingCommand(*routing_with_id));
  EXPECT_TRUE(SupportsDirectValetParkingEntry(config));
  EXPECT_TRUE(ShouldUseDirectValetParkingMode(true, routing_with_id));

  auto routing_with_corners = std::make_shared<RoutingResponse>();
  auto* corner_point = routing_with_corners->mutable_routing_request()
                           ->mutable_parking_info()
                           ->mutable_corner_point();
  corner_point->add_point()->set_x(0.0);
  corner_point->add_point()->set_x(1.0);
  EXPECT_TRUE(HasParkingRoutingCommand(*routing_with_corners));
  EXPECT_FALSE(HasParkingSpaceIdRoutingCommand(*routing_with_corners));
  EXPECT_FALSE(ShouldUseDirectValetParkingMode(true, routing_with_corners));
}

TEST(DirectValetParkingCommandTest, RequiresParkingOnlyValetScenario) {
  ScenarioConfig config;
  config.add_stage_type(StageType::PULL_OVER_APPROACH);
  config.add_stage_type(StageType::VALET_PARKING_PARKING);

  auto routing_with_id =
      std::make_shared<RoutingResponse>(MakeRoutingResponse(1, "road_a", "slot_a"));
  EXPECT_FALSE(SupportsDirectValetParkingEntry(config));
  EXPECT_FALSE(ShouldUseDirectValetParkingMode(false, routing_with_id));
}

}  // namespace
}  // namespace util
}  // namespace planning
}  // namespace apollo
