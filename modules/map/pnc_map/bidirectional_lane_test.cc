#include "modules/map/pnc_map/pnc_map.h"

#include <cmath>
#include <list>
#include <string>

#include "gtest/gtest.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace hdmap {
namespace {

constexpr double kLaneLength = 100.0;

void AddDirectedLane(const std::string &id, const std::string &reverse_id,
                     bool reverse, Map *map) {
  auto *lane = map->add_lane();
  lane->mutable_id()->set_id(id);
  lane->add_self_reverse_lane_id()->set_id(reverse_id);
  lane->set_type(Lane::CITY_DRIVING);
  auto *line =
      lane->mutable_central_curve()->add_segment()->mutable_line_segment();
  line->add_point()->set_x(reverse ? kLaneLength : 0.0);
  line->add_point()->set_x(reverse ? 0.0 : kLaneLength);
  for (auto *sample : {lane->add_left_sample(), lane->add_right_sample()}) {
    sample->set_s(0.0);
    sample->set_width(2.0);
  }
  auto *boundary = lane->mutable_left_boundary();
  auto *first_type = boundary->add_boundary_type();
  first_type->set_s(0.0);
  first_type->add_types(LaneBoundaryType::DOTTED_WHITE);
  auto *second_type = boundary->add_boundary_type();
  second_type->set_s(50.0);
  second_type->add_types(LaneBoundaryType::SOLID_WHITE);
}

routing::RoutingResponse MakeRoute(const std::string &id) {
  routing::RoutingResponse route;
  auto *segment = route.add_road()->add_passage()->add_segment();
  segment->set_id(id);
  segment->set_start_s(0.0);
  segment->set_end_s(kLaneLength);
  for (double s : {0.0, kLaneLength}) {
    auto *waypoint = route.mutable_routing_request()->add_waypoint();
    waypoint->set_id(id);
    waypoint->set_s(s);
  }
  return route;
}

TEST(BidirectionalLaneTest, UsesOnlyHeadingAlignedRoutedDirection) {
  Map map_proto;
  AddDirectedLane("forward", "reverse", false, &map_proto);
  AddDirectedLane("reverse", "forward", true, &map_proto);
  HDMap hdmap;
  ASSERT_EQ(0, hdmap.LoadMapFromProto(map_proto));

  for (const auto &id : {"forward", "reverse"}) {
    PncMap pnc_map(&hdmap);
    ASSERT_TRUE(pnc_map.UpdateRoutingResponse(MakeRoute(id)));
    common::VehicleState state;
    state.set_x(50.0);
    state.set_y(0.0);
    const auto lane = hdmap.GetLaneById(MakeMapId(id));
    ASSERT_TRUE(lane);
    EXPECT_EQ(LaneBoundaryType::DOTTED_WHITE,
              LeftBoundaryType(LaneWaypoint(lane, 25.0)));
    EXPECT_EQ(LaneBoundaryType::SOLID_WHITE,
              LeftBoundaryType(LaneWaypoint(lane, 75.0)));
    state.set_heading(lane->Heading(50.0));
    std::list<RouteSegments> segments;
    ASSERT_TRUE(pnc_map.GetRouteSegments(state, 10.0, 30.0, &segments));
    ASSERT_EQ(1U, segments.size());
    EXPECT_EQ(id, segments.front().front().lane->id().id());
    Path path(segments.front());
    EXPECT_NEAR(lane->Heading(50.0), path.GetSmoothPoint(10.0).heading(),
                1e-6);

    segments.clear();
    state.set_heading(state.heading() + M_PI);
    EXPECT_FALSE(pnc_map.GetRouteSegments(state, 10.0, 30.0, &segments));
  }
}

}  // namespace
}  // namespace hdmap
}  // namespace apollo
