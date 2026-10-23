#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/routing/proto/topo_graph.pb.h"
#include "wheelos_msgs/routing_msgs/routing.pb.h"

namespace apollo {
namespace routing {

// Longitudinal reachability is independent of lane-change passage generation.
class DirectionalRouting {
 public:
  enum class SearchResult { FOUND, UNREACHABLE, INVALID_REQUEST };

  bool LoadGraph(const Graph& graph);
  bool enabled() const { return enabled_; }
  std::size_t chain_count() const { return chains_.size(); }
  SearchResult Search(const RoutingRequest& request, RoutingResponse* response,
                      std::string* error) const;

 private:
  struct State {
    int lane = -1;
    bool reverse = false;
    std::vector<int> next;
    std::vector<int> previous;
    int chain = -1;
    double offset = 0.0;
    bool mergeable = false;
  };

  struct Chain {
    std::vector<int> states;
    std::vector<int> next;
    double length = 0.0;
  };

  struct LaneStates {
    int along = -1;
    int against = -1;
  };

  void AddConnection(int from, int to);
  void Compress();
  bool MatchesHeading(const LaneWaypoint& waypoint, int state) const;
  void AppendInterval(int chain, double start, double end,
                      RoutingResponse* response) const;

  bool enabled_ = false;
  std::string map_version_;
  std::vector<Node> lanes_;
  std::unordered_map<std::string, int> lane_index_;
  std::vector<LaneStates> lane_states_;
  std::vector<State> states_;
  std::vector<Chain> chains_;
};

}  // namespace routing
}  // namespace apollo
