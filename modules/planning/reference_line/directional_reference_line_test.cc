#include <cmath>
#include <list>
#include <vector>

#include "gtest/gtest.h"

#include "gflags/gflags.h"

#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/routing/core/directional_routing.h"

namespace apollo {
namespace planning {
namespace {

hdmap::Lane* AddLane(hdmap::Map* map, const std::string& id, double x) {
  auto* lane = map->add_lane();
  lane->mutable_id()->set_id(id);
  lane->set_type(hdmap::Lane::CITY_DRIVING);
  lane->set_direction(hdmap::Lane::BIDIRECTION);
  lane->set_length(10.0);
  auto* segment = lane->mutable_central_curve()->add_segment();
  segment->set_length(10.0);
  for (double offset : {0.0, 5.0, 10.0}) {
    auto* point = segment->mutable_line_segment()->add_point();
    point->set_x(x + offset);
    point->set_y(0.0);
  }
  for (double s : {0.0, 10.0}) {
    auto* left = lane->add_left_sample();
    left->set_s(s);
    left->set_width(2.0);
    auto* right = lane->add_right_sample();
    right->set_s(s);
    right->set_width(3.0);
    lane->add_left_road_sample()->CopyFrom(*left);
    lane->add_right_road_sample()->CopyFrom(*right);
  }
  lane->mutable_left_boundary()->add_boundary_type()->add_types(
      hdmap::LaneBoundaryType::SOLID_WHITE);
  lane->mutable_right_boundary()->add_boundary_type()->add_types(
      hdmap::LaneBoundaryType::DOTTED_WHITE);
  return lane;
}

routing::RoutingResponse ReverseRoute() {
  routing::RoutingResponse route;
  for (const auto& id : {"b", "a"}) {
    auto* road = route.add_road();
    road->set_id("road");
    auto* passage = road->add_passage();
    passage->set_can_exit(true);
    auto* segment = passage->add_segment();
    segment->set_id(id);
    segment->set_start_s(id == std::string("b") ? 0.0 : 2.0);
    segment->set_end_s(id == std::string("b") ? 8.0 : 10.0);
    segment->set_traversal_direction(routing::AGAINST_CENTERLINE);
  }
  auto* start = route.mutable_routing_request()->add_waypoint();
  start->set_id("b");
  start->set_s(8.0);
  auto* end = route.mutable_routing_request()->add_waypoint();
  end->set_id("a");
  end->set_s(2.0);
  return route;
}

TEST(DirectionalReferenceLineTest,
     ReverseRouteBuildsNativeCoordinateReference) {
  hdmap::Map map;
  auto* a = AddLane(&map, "a", 0.0);
  auto* b = AddLane(&map, "b", 10.0);
  a->add_successor_id()->set_id("b");
  b->add_predecessor_id()->set_id("a");
  b->add_overlap_id()->set_id("overlap");
  auto* speed_bump = map.add_speed_bump();
  speed_bump->mutable_id()->set_id("bump");
  auto* bump_line =
      speed_bump->add_position()->add_segment()->mutable_line_segment();
  for (double y : {-1.0, 1.0}) {
    auto* point = bump_line->add_point();
    point->set_x(13.0);
    point->set_y(y);
  }
  auto* overlap = map.add_overlap();
  overlap->mutable_id()->set_id("overlap");
  auto* lane_overlap = overlap->add_object();
  lane_overlap->mutable_id()->set_id("b");
  lane_overlap->mutable_lane_overlap_info()->set_start_s(2.0);
  lane_overlap->mutable_lane_overlap_info()->set_end_s(4.0);
  auto* bump = overlap->add_object();
  bump->mutable_id()->set_id("bump");
  bump->mutable_speed_bump_overlap_info();
  hdmap::HDMap hdmap;
  ASSERT_EQ(hdmap.LoadMapFromProto(map), 0);
  hdmap::PncMap pnc_map(&hdmap);
  ASSERT_TRUE(pnc_map.UpdateRoutingResponse(ReverseRoute()));
  common::VehicleState state;
  state.set_x(15.0);
  state.set_y(0.0);
  state.set_z(0.0);
  state.set_heading(M_PI);
  std::list<hdmap::RouteSegments> routes;
  ASSERT_TRUE(pnc_map.GetRouteSegments(state, 100.0, 100.0, &routes));
  ASSERT_EQ(routes.size(), 1U);
  const auto& route = routes.front();
  ASSERT_EQ(route.size(), 2U);
  EXPECT_TRUE(route[0].reverse);
  EXPECT_EQ(route[0].lane->id().id(), "b");
  EXPECT_EQ(route[1].lane->id().id(), "a");
  EXPECT_DOUBLE_EQ(route[0].end_s, 8.0);
  EXPECT_DOUBLE_EQ(route[1].start_s, 2.0);

  common::SLPoint sl;
  hdmap::LaneWaypoint waypoint;
  ASSERT_TRUE(
      route.GetProjection(common::math::Vec2d(15.0, 1.0), &sl, &waypoint));
  EXPECT_DOUBLE_EQ(sl.s(), 3.0);
  EXPECT_DOUBLE_EQ(sl.l(), -1.0);
  EXPECT_TRUE(waypoint.reverse);
  EXPECT_EQ(hdmap::LeftBoundaryType(waypoint),
            hdmap::LaneBoundaryType::DOTTED_WHITE);
  EXPECT_EQ(hdmap::RightBoundaryType(waypoint),
            hdmap::LaneBoundaryType::SOLID_WHITE);

  hdmap::Path path(std::vector<hdmap::LaneSegment>(route.begin(), route.end()));
  EXPECT_DOUBLE_EQ(path.length(), 16.0);
  EXPECT_DOUBLE_EQ(path.path_points().front().x(), 18.0);
  EXPECT_DOUBLE_EQ(path.path_points().back().x(), 2.0);
  const auto interpolated = path.GetSmoothPoint(1.5);
  EXPECT_DOUBLE_EQ(interpolated.x(), 16.5);
  ASSERT_FALSE(interpolated.lane_waypoints().empty());
  EXPECT_DOUBLE_EQ(interpolated.lane_waypoints()[0].s, 6.5);
  EXPECT_TRUE(interpolated.lane_waypoints()[0].reverse);
  double left = 0.0;
  double right = 0.0;
  ASSERT_TRUE(path.GetLaneWidth(1.5, &left, &right));
  EXPECT_DOUBLE_EQ(left, 3.0);
  EXPECT_DOUBLE_EQ(right, 2.0);
  ASSERT_EQ(path.speed_bump_overlaps().size(), 1U);
  EXPECT_DOUBLE_EQ(path.speed_bump_overlaps()[0].start_s, 4.0);
  EXPECT_DOUBLE_EQ(path.speed_bump_overlaps()[0].end_s, 6.0);

  ReferenceLine reference(path);
  const auto point = reference.GetReferencePoint(1.5);
  EXPECT_NEAR(point.x(), 16.5, 1e-8);
  EXPECT_NEAR(std::cos(point.heading()), -1.0, 1e-8);
  ASSERT_FALSE(point.lane_waypoints().empty());
  EXPECT_TRUE(point.lane_waypoints()[0].reverse);
  EXPECT_NEAR(point.lane_waypoints()[0].s, 6.5, 1e-8);
  ASSERT_TRUE(reference.GetLaneWidth(1.5, &left, &right));
  EXPECT_DOUBLE_EQ(left, 3.0);
  EXPECT_DOUBLE_EQ(right, 2.0);
  auto cropped = route;
  ASSERT_TRUE(cropped.Shrink(3.0, 1.0, 4.0));
  ASSERT_EQ(cropped.size(), 1U);
  EXPECT_DOUBLE_EQ(cropped[0].start_s, 1.0);
  EXPECT_DOUBLE_EQ(cropped[0].end_s, 6.0);
  EXPECT_TRUE(cropped[0].reverse);
  const auto subsegments = path.GetLaneSegments(2.0, 12.0);
  ASSERT_EQ(subsegments.size(), 2U);
  EXPECT_DOUBLE_EQ(subsegments[0].end_s, 6.0);
  EXPECT_DOUBLE_EQ(subsegments[1].start_s, 6.0);
}

TEST(DirectionalReferenceLineTest, ConsumesExpandedRoutingOutput) {
  hdmap::Map map;
  AddLane(&map, "a", 0.0);
  AddLane(&map, "b", 10.0);
  hdmap::HDMap hdmap;
  ASSERT_EQ(hdmap.LoadMapFromProto(map), 0);
  routing::Graph graph;
  for (const auto& lane : map.lane()) {
    auto* node = graph.add_node();
    node->set_lane_id(lane.id().id());
    node->set_road_id("road");
    node->set_length(lane.length());
    node->set_direction(lane.direction());
    node->mutable_central_curve()->CopyFrom(lane.central_curve());
  }
  auto* edge = graph.add_edge();
  edge->set_from_lane_id("a");
  edge->set_to_lane_id("b");
  edge->set_direction_type(routing::Edge::FORWARD);
  routing::DirectionalRouting router;
  ASSERT_TRUE(router.LoadGraph(graph));
  routing::RoutingResponse response;
  std::string error;
  ASSERT_EQ(router.Search(ReverseRoute().routing_request(), &response, &error),
            routing::DirectionalRouting::SearchResult::FOUND)
      << error;
  hdmap::PncMap pnc_map(&hdmap);
  ASSERT_TRUE(pnc_map.UpdateRoutingResponse(response));
  common::VehicleState state;
  state.set_x(15.0);
  state.set_y(0.0);
  state.set_z(0.0);
  state.set_heading(M_PI);
  std::list<hdmap::RouteSegments> routes;
  ASSERT_TRUE(pnc_map.GetRouteSegments(state, 100.0, 100.0, &routes));
  ASSERT_EQ(routes.size(), 1U);
  hdmap::Path path(std::vector<hdmap::LaneSegment>(routes.front().begin(),
                                                   routes.front().end()));
  ReferenceLine reference(path);
  EXPECT_DOUBLE_EQ(reference.Length(), response.measurement().distance());
  EXPECT_DOUBLE_EQ(reference.reference_points().front().x(), 18.0);
  EXPECT_DOUBLE_EQ(reference.reference_points().back().x(), 2.0);
  EXPECT_TRUE(reference.GetReferencePoint(1.5).lane_waypoints()[0].reverse);
}

TEST(DirectionalReferenceLineTest,
     JoinsReverseIntervalsWithoutLosingDirection) {
  hdmap::Map map;
  AddLane(&map, "a", 0.0);
  hdmap::HDMap hdmap;
  ASSERT_EQ(hdmap.LoadMapFromProto(map), 0);
  auto lane = hdmap.GetLaneById(hdmap::MakeMapId("a"));
  std::vector<hdmap::LaneSegment> segments{{lane, 5.0, 9.0, true},
                                           {lane, 1.0, 5.0, true}};
  hdmap::LaneSegment::Join(&segments);
  ASSERT_EQ(segments.size(), 1U);
  EXPECT_DOUBLE_EQ(segments[0].start_s, 1.0);
  EXPECT_DOUBLE_EQ(segments[0].end_s, 9.0);
  EXPECT_TRUE(segments[0].reverse);
  hdmap::Path path(segments);
  EXPECT_DOUBLE_EQ(path.path_points().front().x(), 9.0);
  EXPECT_DOUBLE_EQ(path.path_points().back().x(), 1.0);
}

TEST(DirectionalReferenceLineTest, RejectsReverseOnForwardOnlyMapLane) {
  hdmap::Map map;
  AddLane(&map, "a", 0.0)->set_direction(hdmap::Lane::FORWARD);
  AddLane(&map, "b", 10.0);
  hdmap::HDMap hdmap;
  ASSERT_EQ(hdmap.LoadMapFromProto(map), 0);
  hdmap::PncMap pnc_map(&hdmap);
  EXPECT_FALSE(pnc_map.UpdateRoutingResponse(ReverseRoute()));
}

TEST(DirectionalReferenceLineTest, RepeatedLaneExtensionUsesRouteOccurrence) {
  hdmap::Map map;
  AddLane(&map, "a", 0.0);
  AddLane(&map, "b", 10.0);
  hdmap::HDMap hdmap;
  ASSERT_EQ(hdmap.LoadMapFromProto(map), 0);
  auto response = ReverseRoute();
  const auto second_b = response.road(0);
  const auto second_a = response.road(1);
  response.add_road()->CopyFrom(second_b);
  response.add_road()->CopyFrom(second_a);
  hdmap::PncMap pnc_map(&hdmap);
  ASSERT_TRUE(pnc_map.UpdateRoutingResponse(response));
  hdmap::RouteSegments passage;
  passage.push_back(pnc_map.route_indices_[2].segment);
  EXPECT_EQ(passage.front().route_index, 2);
  hdmap::RouteSegments expanded;
  ASSERT_TRUE(pnc_map.ExtendSegments(passage, 2.0, 30.0, &expanded));
  ASSERT_EQ(expanded.size(), 2U);
  EXPECT_EQ(expanded.front().route_index, 2);
  EXPECT_EQ(expanded.back().route_index, 3);
  EXPECT_DOUBLE_EQ(expanded.front().end_s, 6.0);
  EXPECT_DOUBLE_EQ(expanded.back().start_s, 2.0);
  ReferenceLine reference{hdmap::Path(expanded)};
  EXPECT_DOUBLE_EQ(reference.Length(), 14.0);
  EXPECT_DOUBLE_EQ(reference.reference_points().front().x(), 16.0);
  EXPECT_DOUBLE_EQ(reference.reference_points().back().x(), 2.0);
  EXPECT_EQ(reference.GetReferencePoint(1.0).lane_waypoints()[0].route_index,
            2);
  ASSERT_TRUE(expanded.Shrink(3.0, 1.0, 2.0));
  EXPECT_EQ(expanded.front().route_index, 2);
  EXPECT_DOUBLE_EQ(expanded.front().end_s, 4.0);
  EXPECT_DOUBLE_EQ(expanded.front().start_s, 1.0);

  hdmap::RouteSegments first_visit;
  first_visit.push_back(pnc_map.route_indices_[0].segment);
  EXPECT_FALSE(passage.IsConnectedSegment(first_visit));
  EXPECT_FALSE(passage.Stitch(first_visit));
  EXPECT_EQ(passage.size(), 1U);

  hdmap::RouteSegments look_back;
  ASSERT_TRUE(pnc_map.ExtendSegments(passage, -3.0, 5.0, &look_back));
  ASSERT_EQ(look_back.size(), 2U);
  EXPECT_EQ(look_back.front().route_index, 1);
  EXPECT_DOUBLE_EQ(look_back.front().start_s, 2.0);
  EXPECT_DOUBLE_EQ(look_back.front().end_s, 5.0);
  EXPECT_EQ(look_back.back().route_index, 2);
  EXPECT_DOUBLE_EQ(look_back.back().start_s, 3.0);
  EXPECT_DOUBLE_EQ(look_back.back().end_s, 8.0);

  common::VehicleState state;
  state.set_x(15.0);
  state.set_y(0.0);
  state.set_z(0.0);
  state.set_heading(M_PI);
  pnc_map.adc_state_ = state;
  pnc_map.adc_route_index_ = 2;
  pnc_map.UpdateRoutingRange(2);
  std::list<hdmap::RouteSegments> windows;
  ASSERT_TRUE(pnc_map.GetRouteSegments(state, 1.0, 8.0, &windows));
  ASSERT_EQ(windows.size(), 1U);
  ASSERT_EQ(windows.front().size(), 2U);
  EXPECT_EQ(windows.front().front().route_index, 2);
  EXPECT_EQ(windows.front().back().route_index, 3);
}

TEST(DirectionalReferenceLineTest, ForwardPortionRetainsRouteOccurrence) {
  hdmap::Map map;
  AddLane(&map, "a", 0.0);
  hdmap::HDMap hdmap;
  ASSERT_EQ(hdmap.LoadMapFromProto(map), 0);
  hdmap::RouteSegments segments;
  segments.emplace_back(hdmap.GetLaneById(hdmap::MakeMapId("a")), 0.0, 10.0,
                        false, 4);
  hdmap::Path path(segments);
  const auto sliced = path.GetLaneSegments(3.0, 7.0);
  ASSERT_EQ(sliced.size(), 1U);
  EXPECT_EQ(sliced.front().route_index, 4);
  EXPECT_DOUBLE_EQ(sliced.front().start_s, 3.0);
  EXPECT_DOUBLE_EQ(sliced.front().end_s, 7.0);
  ASSERT_TRUE(segments.Shrink(5.0, 2.0, 2.0));
  ASSERT_EQ(segments.size(), 1U);
  EXPECT_EQ(segments.front().route_index, 4);
  EXPECT_DOUBLE_EQ(segments.front().start_s, 3.0);
  EXPECT_DOUBLE_EQ(segments.front().end_s, 7.0);
}

TEST(DirectionalReferenceLineTest, ProviderExtendsAndStitchesSecondVisit) {
  gflags::FlagSaver flags;
  FLAGS_enable_smooth_reference_line = false;
  hdmap::Map map;
  AddLane(&map, "a", 0.0);
  AddLane(&map, "b", 10.0);
  hdmap::HDMap hdmap;
  ASSERT_EQ(hdmap.LoadMapFromProto(map), 0);
  auto response = ReverseRoute();
  const auto second_b = response.road(0);
  const auto second_a = response.road(1);
  response.add_road()->CopyFrom(second_b);
  response.add_road()->CopyFrom(second_a);
  ReferenceLineProvider provider;
  provider.pnc_map_ = std::make_unique<hdmap::PncMap>(&hdmap);
  ASSERT_TRUE(provider.pnc_map_->UpdateRoutingResponse(response));
  hdmap::RouteSegments previous;
  previous.push_back(provider.pnc_map_->route_indices_[2].segment.Slice(0, 6));
  previous.SetIsOnSegment(true);
  provider.route_segments_.push_back(previous);
  provider.reference_lines_.emplace_back(hdmap::Path(previous));
  common::VehicleState state;
  state.set_x(15.0);
  state.set_y(0.0);
  state.set_z(0.0);
  state.set_heading(M_PI);
  ReferenceLine reference;
  auto current = previous;
  ASSERT_TRUE(provider.ExtendReferenceLine(state, &current, &reference));
  ASSERT_EQ(current.size(), 2U);
  EXPECT_EQ(current.front().route_index, 2);
  EXPECT_EQ(current.back().route_index, 3);
  EXPECT_DOUBLE_EQ(reference.Length(), 16.0);
  EXPECT_DOUBLE_EQ(reference.reference_points().front().x(), 18.0);
  EXPECT_DOUBLE_EQ(reference.reference_points().back().x(), 2.0);
  for (const auto& point : reference.reference_points()) {
    ASSERT_FALSE(point.lane_waypoints().empty());
    EXPECT_TRUE(point.lane_waypoints()[0].reverse);
    EXPECT_GE(point.lane_waypoints()[0].route_index, 2);
    EXPECT_NEAR(std::cos(point.heading()), -1.0, 1e-8);
  }

  FLAGS_enable_smooth_reference_line = true;
  provider.smoother_config_.set_max_constraint_interval(1.0);
  auto* fem = provider.smoother_config_.mutable_discrete_points()
                  ->mutable_fem_pos_deviation_smoothing();
  fem->set_weight_fem_pos_deviation(1e3);
  fem->set_time_limit(1.0);
  provider.smoother_ = std::make_unique<DiscretePointsReferenceLineSmoother>(
      provider.smoother_config_);
  ReferenceLine smoothed;
  ASSERT_TRUE(provider.SmoothRouteSegment(current, &smoothed));
  EXPECT_NEAR(smoothed.Length(), reference.Length(), 0.05);
  for (const auto& point : smoothed.reference_points()) {
    ASSERT_FALSE(point.lane_waypoints().empty());
    EXPECT_TRUE(point.lane_waypoints()[0].reverse);
    EXPECT_GE(point.lane_waypoints()[0].route_index, 2);
    EXPECT_NEAR(std::cos(point.heading()), -1.0, 0.01);
  }
  current = previous;
  ReferenceLine prefixed;
  ASSERT_TRUE(provider.ExtendReferenceLine(state, &current, &prefixed));
  ASSERT_EQ(current.size(), 2U);
  EXPECT_EQ(current.front().route_index, 2);
  EXPECT_EQ(current.back().route_index, 3);
  EXPECT_NEAR(prefixed.Length(), reference.Length(), 0.05);
  for (const auto& point : prefixed.reference_points()) {
    ASSERT_FALSE(point.lane_waypoints().empty());
    EXPECT_TRUE(point.lane_waypoints()[0].reverse);
    EXPECT_GE(point.lane_waypoints()[0].route_index, 2);
    EXPECT_NEAR(std::cos(point.heading()), -1.0, 0.01);
  }
}

}  // namespace
}  // namespace planning
}  // namespace apollo
