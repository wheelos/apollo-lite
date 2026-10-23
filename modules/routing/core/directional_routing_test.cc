#include "modules/routing/core/directional_routing.h"

#include <cstdio>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "modules/routing/core/navigator.h"

namespace apollo {
namespace routing {
namespace {

Node* AddLane(Graph* graph, const std::string& id,
              hdmap::Lane::LaneDirection direction = hdmap::Lane::BIDIRECTION,
              double length = 10.0) {
  auto* node = graph->add_node();
  node->set_lane_id(id);
  node->set_road_id("road_" + id);
  node->set_length(length);
  node->set_cost(length);
  node->set_direction(direction);
  auto* curve = node->mutable_central_curve()->add_segment();
  curve->set_length(length);
  curve->mutable_line_segment()->add_point()->set_x(0.0);
  curve->mutable_line_segment()->add_point()->set_x(length);
  for (auto& point : *curve->mutable_line_segment()->mutable_point()) {
    point.set_y(0.0);
  }
  return node;
}

void Connect(Graph* graph, const std::string& from, const std::string& to) {
  auto* edge = graph->add_edge();
  edge->set_from_lane_id(from);
  edge->set_to_lane_id(to);
  edge->set_direction_type(Edge::FORWARD);
}

RoutingRequest Request(const std::string& start, double start_s,
                       const std::string& end, double end_s) {
  RoutingRequest request;
  auto* source = request.add_waypoint();
  source->set_id(start);
  source->set_s(start_s);
  auto* target = request.add_waypoint();
  target->set_id(end);
  target->set_s(end_s);
  return request;
}

std::vector<std::string> LaneIds(const RoutingResponse& response) {
  std::vector<std::string> result;
  for (const auto& road : response.road()) {
    for (const auto& passage : road.passage()) {
      for (const auto& segment : passage.segment()) {
        result.push_back(segment.id());
      }
    }
  }
  return result;
}

TEST(DirectionalRoutingTest, CompressesAndExpandsBothDirections) {
  Graph graph;
  AddLane(&graph, "a");
  AddLane(&graph, "b");
  AddLane(&graph, "c");
  Connect(&graph, "a", "b");
  Connect(&graph, "b", "c");
  DirectionalRouting router;
  ASSERT_TRUE(router.LoadGraph(graph));
  EXPECT_EQ(router.chain_count(), 2U);
  RoutingResponse response;
  std::string error;
  ASSERT_EQ(router.Search(Request("a", 3.0, "c", 7.0), &response, &error),
            DirectionalRouting::SearchResult::FOUND)
      << error;
  EXPECT_EQ(LaneIds(response), (std::vector<std::string>{"a", "b", "c"}));
  EXPECT_DOUBLE_EQ(response.measurement().distance(), 24.0);
  EXPECT_DOUBLE_EQ(response.road(0).passage(0).segment(0).start_s(), 3.0);
  EXPECT_DOUBLE_EQ(response.road(2).passage(0).segment(0).end_s(), 7.0);
  ASSERT_EQ(router.Search(Request("c", 7.0, "a", 3.0), &response, &error),
            DirectionalRouting::SearchResult::FOUND)
      << error;
  EXPECT_EQ(LaneIds(response), (std::vector<std::string>{"c", "b", "a"}));
  EXPECT_DOUBLE_EQ(response.measurement().distance(), 24.0);
  for (const auto& road : response.road()) {
    const auto& segment = road.passage(0).segment(0);
    EXPECT_EQ(segment.traversal_direction(), AGAINST_CENTERLINE);
    EXPECT_LE(segment.start_s(), segment.end_s());
  }
  EXPECT_DOUBLE_EQ(response.road(0).passage(0).segment(0).end_s(), 7.0);
  EXPECT_DOUBLE_EQ(response.road(2).passage(0).segment(0).start_s(), 3.0);
}

TEST(DirectionalRoutingTest, DispersedLanesDoNotReverseOneWayConnections) {
  Graph graph;
  AddLane(&graph, "a");
  AddLane(&graph, "one_way", hdmap::Lane::FORWARD);
  AddLane(&graph, "b");
  Connect(&graph, "a", "one_way");
  Connect(&graph, "one_way", "b");
  DirectionalRouting router;
  ASSERT_TRUE(router.LoadGraph(graph));
  EXPECT_EQ(router.chain_count(), 5U);
  RoutingResponse response;
  std::string error;
  EXPECT_EQ(router.Search(Request("a", 5.0, "b", 5.0), &response, &error),
            DirectionalRouting::SearchResult::FOUND);
  EXPECT_EQ(router.Search(Request("b", 5.0, "a", 5.0), &response, &error),
            DirectionalRouting::SearchResult::UNREACHABLE);
  EXPECT_EQ(router.Search(Request("b", 8.0, "b", 2.0), &response, &error),
            DirectionalRouting::SearchResult::FOUND);
  EXPECT_DOUBLE_EQ(response.measurement().distance(), 6.0);
}

TEST(DirectionalRoutingTest, KeepsWaypointsAtLaneEndpoints) {
  Graph graph;
  AddLane(&graph, "a");
  AddLane(&graph, "b");
  Connect(&graph, "a", "b");
  DirectionalRouting router;
  ASSERT_TRUE(router.LoadGraph(graph));
  RoutingResponse response;
  std::string error;
  ASSERT_EQ(router.Search(Request("a", 10.0, "b", 5.0), &response, &error),
            DirectionalRouting::SearchResult::FOUND);
  EXPECT_EQ(LaneIds(response), (std::vector<std::string>{"a", "b"}));
  EXPECT_DOUBLE_EQ(response.road(0).passage(0).segment(0).start_s(), 10.0);
  EXPECT_DOUBLE_EQ(response.road(0).passage(0).segment(0).end_s(), 10.0);
  ASSERT_EQ(router.Search(Request("b", 5.0, "a", 10.0), &response, &error),
            DirectionalRouting::SearchResult::FOUND);
  EXPECT_EQ(LaneIds(response), (std::vector<std::string>{"b", "a"}));
  EXPECT_EQ(response.road(1).passage(0).segment(0).traversal_direction(),
            AGAINST_CENTERLINE);
  EXPECT_DOUBLE_EQ(response.measurement().distance(), 5.0);
}

TEST(DirectionalRoutingTest, BranchesAndDifferentLengthsChooseShortestPath) {
  Graph graph;
  for (const auto& id : {"a", "b", "c", "d"}) {
    AddLane(&graph, id);
  }
  AddLane(&graph, "long", hdmap::Lane::BIDIRECTION, 100.0);
  Connect(&graph, "a", "b");
  Connect(&graph, "b", "c");
  Connect(&graph, "a", "long");
  Connect(&graph, "long", "c");
  Connect(&graph, "c", "d");
  DirectionalRouting router;
  ASSERT_TRUE(router.LoadGraph(graph));
  RoutingResponse response;
  std::string error;
  ASSERT_EQ(router.Search(Request("d", 5.0, "a", 5.0), &response, &error),
            DirectionalRouting::SearchResult::FOUND);
  EXPECT_EQ(LaneIds(response), (std::vector<std::string>{"d", "c", "b", "a"}));
  EXPECT_DOUBLE_EQ(response.measurement().distance(), 30.0);
  auto request = Request("a", 5.0, "d", 5.0);
  request.add_blacklisted_road("road_b");
  ASSERT_EQ(router.Search(request, &response, &error),
            DirectionalRouting::SearchResult::FOUND);
  EXPECT_EQ(LaneIds(response),
            (std::vector<std::string>{"a", "long", "c", "d"}));
}

TEST(DirectionalRoutingTest, PartialAndPointBlocksSplitCompressedChains) {
  Graph graph;
  AddLane(&graph, "a");
  AddLane(&graph, "b");
  Connect(&graph, "a", "b");
  DirectionalRouting router;
  ASSERT_TRUE(router.LoadGraph(graph));
  std::string error;
  RoutingResponse response;
  auto request = Request("a", 8.0, "b", 5.0);
  auto* block = request.add_blacklisted_lane();
  block->set_id("a");
  block->set_start_s(3.0);
  block->set_end_s(6.0);
  EXPECT_EQ(router.Search(request, &response, &error),
            DirectionalRouting::SearchResult::FOUND);
  request.mutable_waypoint(0)->set_s(2.0);
  EXPECT_EQ(router.Search(request, &response, &error),
            DirectionalRouting::SearchResult::UNREACHABLE);
  request.mutable_waypoint(0)->set_id("b");
  request.mutable_waypoint(0)->set_s(5.0);
  request.mutable_waypoint(1)->set_id("a");
  request.mutable_waypoint(1)->set_s(8.0);
  EXPECT_EQ(router.Search(request, &response, &error),
            DirectionalRouting::SearchResult::FOUND);
  request.mutable_waypoint(1)->set_s(2.0);
  EXPECT_EQ(router.Search(request, &response, &error),
            DirectionalRouting::SearchResult::UNREACHABLE);
  block->set_end_s(3.0);
  EXPECT_EQ(router.Search(request, &response, &error),
            DirectionalRouting::SearchResult::UNREACHABLE);
}

TEST(DirectionalRoutingTest, CyclesAndInternalWaypointsPreserveDirection) {
  Graph graph;
  for (const auto& id : {"a", "b", "c"}) {
    AddLane(&graph, id);
  }
  Connect(&graph, "a", "b");
  Connect(&graph, "b", "c");
  Connect(&graph, "c", "a");
  DirectionalRouting router;
  ASSERT_TRUE(router.LoadGraph(graph));
  EXPECT_EQ(router.chain_count(), 2U);
  auto request = Request("b", 8.0, "b", 2.0);
  request.mutable_waypoint(0)->set_heading(0.0);
  request.mutable_waypoint(1)->set_heading(0.0);
  RoutingResponse response;
  std::string error;
  ASSERT_EQ(router.Search(request, &response, &error),
            DirectionalRouting::SearchResult::FOUND);
  EXPECT_EQ(LaneIds(response), (std::vector<std::string>{"b", "c", "a", "b"}));
  EXPECT_DOUBLE_EQ(response.measurement().distance(), 24.0);
  auto middle = request.waypoint(1);
  middle.set_id("c");
  middle.set_s(5.0);
  request.mutable_waypoint()->Add()->CopyFrom(request.waypoint(1));
  request.mutable_waypoint(1)->CopyFrom(middle);
  ASSERT_EQ(router.Search(request, &response, &error),
            DirectionalRouting::SearchResult::FOUND);
  EXPECT_DOUBLE_EQ(response.measurement().distance(), 24.0);
}

TEST(DirectionalRoutingTest, BackwardOnlyAndInvalidRequests) {
  Graph graph;
  AddLane(&graph, "a", hdmap::Lane::BACKWARD);
  DirectionalRouting router;
  ASSERT_TRUE(router.LoadGraph(graph));
  RoutingResponse response;
  std::string error;
  ASSERT_EQ(router.Search(Request("a", 8.0, "a", 2.0), &response, &error),
            DirectionalRouting::SearchResult::FOUND);
  EXPECT_EQ(response.road(0).passage(0).segment(0).traversal_direction(),
            AGAINST_CENTERLINE);
  EXPECT_EQ(router.Search(Request("a", 2.0, "a", 8.0), &response, &error),
            DirectionalRouting::SearchResult::UNREACHABLE);
  EXPECT_EQ(router.Search(Request("missing", 0.0, "a", 8.0), &response, &error),
            DirectionalRouting::SearchResult::INVALID_REQUEST);
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(router.Search(Request("a", -1.0, "a", 8.0), &response, &error),
            DirectionalRouting::SearchResult::INVALID_REQUEST);
}

TEST(DirectionalRoutingTest, NavigatorUsesDirectionalResult) {
  Graph graph;
  AddLane(&graph, "a", hdmap::Lane::BACKWARD);
  const std::string path =
      ::testing::TempDir() + "directional_routing_graph.pb.txt";
  ASSERT_TRUE(cyber::common::SetProtoToASCIIFile(graph, path));
  Navigator navigator(path);
  ASSERT_TRUE(navigator.IsReady());
  RoutingResponse response;
  ASSERT_TRUE(navigator.SearchRoute(Request("a", 9.0, "a", 1.0), &response));
  EXPECT_DOUBLE_EQ(response.measurement().distance(), 8.0);
  EXPECT_EQ(response.road(0).passage(0).segment(0).traversal_direction(),
            AGAINST_CENTERLINE);
  EXPECT_EQ(std::remove(path.c_str()), 0);
}

TEST(DirectionalRoutingTest, NavigatorRetainsForwardOnlyBehavior) {
  Graph graph;
  AddLane(&graph, "a", hdmap::Lane::FORWARD);
  AddLane(&graph, "b", hdmap::Lane::FORWARD);
  Connect(&graph, "a", "b");
  DirectionalRouting router;
  ASSERT_TRUE(router.LoadGraph(graph));
  EXPECT_FALSE(router.enabled());
  const std::string path =
      ::testing::TempDir() + "forward_only_routing_graph.pb.txt";
  ASSERT_TRUE(cyber::common::SetProtoToASCIIFile(graph, path));
  Navigator navigator(path);
  ASSERT_TRUE(navigator.IsReady());
  RoutingResponse response;
  ASSERT_TRUE(navigator.SearchRoute(Request("a", 5.0, "b", 5.0), &response));
  EXPECT_EQ(LaneIds(response), (std::vector<std::string>{"a", "b"}));
  EXPECT_DOUBLE_EQ(response.measurement().distance(), 10.0);
  EXPECT_EQ(response.road(0).passage(0).segment(0).traversal_direction(),
            ALONG_CENTERLINE);
  EXPECT_EQ(std::remove(path.c_str()), 0);
}

TEST(DirectionalRoutingTest, LargeChainRetainsOnlyTwoSearchChains) {
  Graph graph;
  constexpr int kLaneCount = 10000;
  for (int i = 0; i < kLaneCount; ++i) {
    AddLane(&graph, std::to_string(i));
    if (i > 0) {
      Connect(&graph, std::to_string(i - 1), std::to_string(i));
    }
  }
  DirectionalRouting router;
  ASSERT_TRUE(router.LoadGraph(graph));
  EXPECT_EQ(router.chain_count(), 2U);
  RoutingResponse response;
  std::string error;
  ASSERT_EQ(router.Search(Request("9999", 9.0, "0", 1.0), &response, &error),
            DirectionalRouting::SearchResult::FOUND);
  EXPECT_EQ(response.road_size(), kLaneCount);
  EXPECT_DOUBLE_EQ(response.measurement().distance(), 99998.0);
}

}  // namespace
}  // namespace routing
}  // namespace apollo
