#include "modules/routing/core/directional_routing.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_set>
#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace routing {
namespace {

constexpr double kEpsilon = 1e-8;

struct Interval {
  double start = 0.0;
  double end = 0.0;
};

struct QueryEdge {
  int to = -1;
  int chain = -1;
  double start = 0.0;
  double end = 0.0;
};

struct ChainQuery {
  std::vector<double> cuts;
  std::vector<Interval> blocked;
  std::vector<int> left;
  std::vector<int> right;
};

bool Contains(const Interval& interval, double position) {
  return interval.start <= position && position <= interval.end;
}

}  // namespace

void DirectionalRouting::AddConnection(int from, int to) {
  if (from < 0 || to < 0) {
    return;
  }
  auto& next = states_[from].next;
  if (std::find(next.begin(), next.end(), to) == next.end()) {
    next.push_back(to);
    states_[to].previous.push_back(from);
  }
}

bool DirectionalRouting::LoadGraph(const Graph& graph) {
  enabled_ = false;
  lanes_.clear();
  lane_index_.clear();
  lane_states_.clear();
  states_.clear();
  chains_.clear();
  map_version_ = graph.hdmap_version();
  for (const auto& node : graph.node()) {
    if (node.lane_id().empty() || !std::isfinite(node.length()) ||
        node.length() <= 0.0 ||
        !lane_index_.emplace(node.lane_id(), lanes_.size()).second) {
      AERROR << "Invalid or duplicate directional routing lane: "
             << node.lane_id();
      return false;
    }
    lanes_.push_back(node);
    lane_states_.emplace_back();
    const int lane = static_cast<int>(lanes_.size()) - 1;
    enabled_ = enabled_ || node.direction() != hdmap::Lane::FORWARD;
    for (bool reverse : {false, true}) {
      if ((!reverse && node.direction() == hdmap::Lane::BACKWARD) ||
          (reverse && node.direction() == hdmap::Lane::FORWARD)) {
        continue;
      }
      const int index = static_cast<int>(states_.size());
      (reverse ? lane_states_.back().against : lane_states_.back().along) =
          index;
      State state;
      state.lane = lane;
      state.reverse = reverse;
      state.mergeable = node.direction() == hdmap::Lane::BIDIRECTION;
      states_.push_back(std::move(state));
    }
  }
  for (const auto& edge : graph.edge()) {
    const auto from = lane_index_.find(edge.from_lane_id());
    const auto to = lane_index_.find(edge.to_lane_id());
    if (from == lane_index_.end() || to == lane_index_.end()) {
      AERROR << "Directional routing edge references an unknown lane";
      return false;
    }
    if (edge.direction_type() != Edge::FORWARD) {
      for (int lane : {from->second, to->second}) {
        for (int state :
             {lane_states_[lane].along, lane_states_[lane].against}) {
          if (state >= 0) {
            states_[state].mergeable = false;
          }
        }
      }
      continue;
    }
    // HDMap successor relations connect the native tail to the next head.
    AddConnection(lane_states_[from->second].along,
                  lane_states_[to->second].along);
    AddConnection(lane_states_[to->second].against,
                  lane_states_[from->second].against);
  }
  Compress();
  return !lanes_.empty();
}

void DirectionalRouting::Compress() {
  const auto can_join = [this](int from, int to) {
    return states_[from].mergeable && states_[to].mergeable &&
           states_[from].next.size() == 1 && states_[to].previous.size() == 1;
  };
  const auto add_chain = [this, &can_join](int first) {
    const int chain_index = static_cast<int>(chains_.size());
    chains_.emplace_back();
    auto& chain = chains_.back();
    int current = first;
    while (states_[current].chain < 0) {
      auto& state = states_[current];
      state.chain = chain_index;
      state.offset = chain.length;
      chain.states.push_back(current);
      chain.length += lanes_[state.lane].length();
      if (state.next.size() != 1 || !can_join(current, state.next.front())) {
        break;
      }
      current = state.next.front();
    }
  };
  for (int i = 0; i < static_cast<int>(states_.size()); ++i) {
    if (states_[i].chain < 0 && (states_[i].previous.size() != 1 ||
                                 !can_join(states_[i].previous.front(), i))) {
      add_chain(i);
    }
  }
  // Remaining components are cycles; keep their closing connection.
  for (int i = 0; i < static_cast<int>(states_.size()); ++i) {
    if (states_[i].chain < 0) {
      add_chain(i);
    }
  }
  for (auto& chain : chains_) {
    for (int next : states_[chain.states.back()].next) {
      chain.next.push_back(states_[next].chain);
    }
  }
}

bool DirectionalRouting::MatchesHeading(const LaneWaypoint& waypoint,
                                        int state_index) const {
  if (!waypoint.has_heading()) {
    return true;
  }
  const auto& state = states_[state_index];
  const auto& lane = lanes_[state.lane];
  double accumulated = 0.0;
  double heading = 0.0;
  bool found = false;
  for (const auto& segment : lane.central_curve().segment()) {
    const auto& points = segment.line_segment().point();
    for (int i = 1; i < points.size(); ++i) {
      const double dx = points[i].x() - points[i - 1].x();
      const double dy = points[i].y() - points[i - 1].y();
      const double length = std::hypot(dx, dy);
      if (length <= kEpsilon) {
        continue;
      }
      heading = std::atan2(dy, dx);
      found = true;
      accumulated += length;
      if (accumulated >= waypoint.s()) {
        break;
      }
    }
    if (found && accumulated >= waypoint.s()) {
      break;
    }
  }
  if (!found) {
    return false;
  }
  if (state.reverse) {
    heading += std::acos(-1.0);
  }
  return std::cos(heading - waypoint.heading()) > 0.0;
}

void DirectionalRouting::AppendInterval(int chain_index, double start,
                                        double end,
                                        RoutingResponse* response) const {
  const auto& members = chains_[chain_index].states;
  auto it = std::lower_bound(
      members.begin(), members.end(), start,
      [this](int index, double position) {
        const auto& state = states_[index];
        return state.offset + lanes_[state.lane].length() <= position;
      });
  for (; it != members.end(); ++it) {
    const auto& state = states_[*it];
    if (state.offset >= end) {
      break;
    }
    const auto& lane = lanes_[state.lane];
    double begin = std::max(start, state.offset) - state.offset;
    double finish = std::min(end, state.offset + lane.length()) - state.offset;
    if (finish <= begin) {
      continue;
    }
    if (state.reverse) {
      const double native_begin = lane.length() - finish;
      finish = lane.length() - begin;
      begin = native_begin;
    }
    const auto direction =
        state.reverse ? AGAINST_CENTERLINE : ALONG_CENTERLINE;
    if (response->road_size() > 0) {
      auto* previous = response->mutable_road()
                           ->rbegin()
                           ->mutable_passage(0)
                           ->mutable_segment(0);
      if (previous->id() == lane.lane_id() &&
          previous->traversal_direction() == direction &&
          ((!state.reverse && previous->end_s() == begin) ||
           (state.reverse && previous->start_s() == finish))) {
        previous->set_start_s(std::min(previous->start_s(), begin));
        previous->set_end_s(std::max(previous->end_s(), finish));
        continue;
      }
    }
    auto* road = response->add_road();
    road->set_id(lane.road_id());
    auto* passage = road->add_passage();
    passage->set_can_exit(true);
    passage->set_change_lane_type(FORWARD);
    auto* segment = passage->add_segment();
    segment->set_id(lane.lane_id());
    segment->set_start_s(begin);
    segment->set_end_s(finish);
    segment->set_traversal_direction(direction);
  }
}

DirectionalRouting::SearchResult DirectionalRouting::Search(
    const RoutingRequest& request, RoutingResponse* response,
    std::string* error) const {
  error->clear();
  const auto invalid = [error](const std::string& message) {
    *error = message;
    return SearchResult::INVALID_REQUEST;
  };
  if (request.waypoint_size() < 2) {
    return invalid("Routing requires at least two waypoints");
  }
  std::vector<ChainQuery> queries(chains_.size());
  for (std::size_t i = 0; i < chains_.size(); ++i) {
    queries[i].cuts = {0.0, chains_[i].length};
  }
  const auto add_block = [this, &queries](int lane_index, double start,
                                          double end) {
    const auto& lane = lanes_[lane_index];
    for (int index :
         {lane_states_[lane_index].along, lane_states_[lane_index].against}) {
      if (index < 0) {
        continue;
      }
      const auto& state = states_[index];
      const double begin =
          state.offset + (state.reverse ? lane.length() - end : start);
      const double finish =
          state.offset + (state.reverse ? lane.length() - start : end);
      auto& query = queries[state.chain];
      query.cuts.push_back(begin);
      query.cuts.push_back(finish);
      query.blocked.push_back({begin, finish});
    }
  };
  for (const auto& block : request.blacklisted_lane()) {
    const auto lane = lane_index_.find(block.id());
    if (lane == lane_index_.end() || !std::isfinite(block.start_s()) ||
        !std::isfinite(block.end_s()) || block.start_s() < 0.0 ||
        block.start_s() > block.end_s() ||
        block.end_s() > lanes_[lane->second].length()) {
      return invalid("Invalid blacklisted lane interval: " + block.id());
    }
    add_block(lane->second, block.start_s(), block.end_s());
  }
  for (const auto& road : request.blacklisted_road()) {
    bool found = false;
    for (int i = 0; i < static_cast<int>(lanes_.size()); ++i) {
      if (lanes_[i].road_id() == road) {
        found = true;
        add_block(i, 0.0, lanes_[i].length());
      }
    }
    if (!found) {
      return invalid("Unknown blacklisted road: " + road);
    }
  }
  std::vector<std::vector<std::pair<int, double>>> terminals(
      request.waypoint_size());
  for (int i = 0; i < request.waypoint_size(); ++i) {
    const auto& waypoint = request.waypoint(i);
    const auto lane = lane_index_.find(waypoint.id());
    if (lane == lane_index_.end() || !std::isfinite(waypoint.s()) ||
        waypoint.s() < 0.0 || waypoint.s() > lanes_[lane->second].length() ||
        (waypoint.has_heading() && !std::isfinite(waypoint.heading()))) {
      return invalid("Invalid routing waypoint: " + waypoint.id());
    }
    for (int index : {lane_states_[lane->second].along,
                      lane_states_[lane->second].against}) {
      if (index < 0 || !MatchesHeading(waypoint, index)) {
        continue;
      }
      const auto& state = states_[index];
      const double offset =
          state.offset + (state.reverse
                              ? lanes_[lane->second].length() - waypoint.s()
                              : waypoint.s());
      auto& query = queries[state.chain];
      bool blocked = false;
      for (const auto& interval : query.blocked) {
        blocked = blocked || Contains(interval, offset);
      }
      if (!blocked) {
        query.cuts.push_back(offset);
        terminals[i].emplace_back(state.chain, offset);
      }
    }
    if (terminals[i].empty()) {
      *error = "Waypoint is blocked or has no permitted traversal direction: " +
               waypoint.id();
      return SearchResult::UNREACHABLE;
    }
  }

  std::vector<std::vector<QueryEdge>> adjacency;
  const auto add_vertex = [&adjacency]() {
    const int vertex = static_cast<int>(adjacency.size());
    adjacency.emplace_back();
    return vertex;
  };
  for (int i = 0; i < static_cast<int>(queries.size()); ++i) {
    auto& query = queries[i];
    std::sort(query.cuts.begin(), query.cuts.end());
    query.cuts.erase(std::unique(query.cuts.begin(), query.cuts.end()),
                     query.cuts.end());
    std::sort(
        query.blocked.begin(), query.blocked.end(),
        [](const Interval& a, const Interval& b) { return a.start < b.start; });
    std::vector<Interval> merged;
    for (const auto& interval : query.blocked) {
      if (!merged.empty() && interval.start <= merged.back().end) {
        merged.back().end = std::max(merged.back().end, interval.end);
      } else {
        merged.push_back(interval);
      }
    }
    std::size_t blocked_index = 0;
    for (std::size_t j = 0; j < query.cuts.size(); ++j) {
      const double position = query.cuts[j];
      query.left.push_back(add_vertex());
      query.right.push_back(add_vertex());
      while (blocked_index < merged.size() &&
             merged[blocked_index].end < position) {
        ++blocked_index;
      }
      if (blocked_index == merged.size() ||
          !Contains(merged[blocked_index], position)) {
        adjacency[query.left.back()].push_back(
            {query.right.back(), -1, 0.0, 0.0});
      }
      if (j + 1 < query.cuts.size()) {
        const double middle = (position + query.cuts[j + 1]) / 2.0;
        if (blocked_index == merged.size() ||
            !Contains(merged[blocked_index], middle)) {
          // The next cut's left vertex is allocated in the next iteration.
          adjacency[query.right.back()].push_back(
              {static_cast<int>(adjacency.size()), i, position,
               query.cuts[j + 1]});
        }
      }
    }
  }
  for (std::size_t i = 0; i < chains_.size(); ++i) {
    for (int next : chains_[i].next) {
      adjacency[queries[i].right.back()].push_back(
          {queries[next].left.front(), -1, 0.0, 0.0});
    }
  }
  std::vector<std::unordered_set<int>> waypoint_vertices(terminals.size());
  for (std::size_t i = 0; i < terminals.size(); ++i) {
    for (const auto& terminal : terminals[i]) {
      const auto& query = queries[terminal.first];
      const auto position = std::lower_bound(query.cuts.begin(),
                                             query.cuts.end(), terminal.second);
      const auto index = position - query.cuts.begin();
      waypoint_vertices[i].insert(query.left[index]);
      waypoint_vertices[i].insert(query.right[index]);
    }
  }
  const std::size_t width = adjacency.size();
  const auto encode = [width](int vertex, std::size_t stage) {
    return stage * width + vertex;
  };
  const auto advance = [&waypoint_vertices](int vertex, std::size_t stage) {
    while (stage < waypoint_vertices.size() &&
           waypoint_vertices[stage].count(vertex) != 0) {
      ++stage;
    }
    return stage;
  };
  struct Label {
    double distance = std::numeric_limits<double>::infinity();
    std::size_t parent = std::numeric_limits<std::size_t>::max();
    QueryEdge edge;
  };
  std::unordered_map<std::size_t, Label> labels;
  using QueueEntry = std::pair<double, std::size_t>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                      std::greater<QueueEntry>>
      queue;
  for (int vertex : waypoint_vertices.front()) {
    const auto key = encode(vertex, advance(vertex, 1));
    labels[key].distance = 0.0;
    queue.emplace(0.0, key);
  }
  std::size_t goal = std::numeric_limits<std::size_t>::max();
  while (!queue.empty()) {
    const auto current = queue.top();
    queue.pop();
    if (current.first != labels.at(current.second).distance) {
      continue;
    }
    const auto stage = current.second / width;
    if (stage == waypoint_vertices.size()) {
      goal = current.second;
      break;
    }
    const int vertex = current.second % width;
    for (const auto& edge : adjacency[vertex]) {
      const auto key = encode(edge.to, advance(edge.to, stage));
      const double distance = current.first + edge.end - edge.start;
      auto& label = labels[key];
      if (distance < label.distance) {
        label.distance = distance;
        label.parent = current.second;
        label.edge = edge;
        queue.emplace(distance, key);
      }
    }
  }
  if (goal == std::numeric_limits<std::size_t>::max()) {
    *error = "No longitudinal path through the requested waypoints";
    return SearchResult::UNREACHABLE;
  }
  std::vector<QueryEdge> path;
  auto root = goal;
  while (labels.at(root).parent != std::numeric_limits<std::size_t>::max()) {
    if (labels.at(root).edge.chain >= 0) {
      path.push_back(labels.at(root).edge);
    }
    root = labels.at(root).parent;
  }
  RoutingResponse result;
  for (auto it = path.rbegin(); it != path.rend(); ++it) {
    AppendInterval(it->chain, it->start, it->end, &result);
  }
  const auto add_terminal = [this, &queries, &terminals, &request, width,
                             &result](int waypoint_index, std::size_t key) {
    const auto& waypoint = request.waypoint(waypoint_index);
    const auto& lane = lanes_[lane_index_.at(waypoint.id())];
    auto* road = result.add_road();
    road->set_id(lane.road_id());
    auto* passage = road->add_passage();
    passage->set_can_exit(true);
    auto* segment = passage->add_segment();
    segment->set_id(waypoint.id());
    segment->set_start_s(waypoint.s());
    segment->set_end_s(waypoint.s());
    const int vertex = key % width;
    for (const auto& terminal : terminals[waypoint_index]) {
      const auto& query = queries[terminal.first];
      const auto position = std::lower_bound(query.cuts.begin(),
                                             query.cuts.end(), terminal.second);
      const auto index = position - query.cuts.begin();
      if (query.left[index] == vertex || query.right[index] == vertex) {
        const auto& state = states_[chains_[terminal.first].states.front()];
        segment->set_traversal_direction(state.reverse ? AGAINST_CENTERLINE
                                                       : ALONG_CENTERLINE);
        break;
      }
    }
  };
  // Preserve native endpoint lane IDs even when their traversed length is zero.
  if (result.road().empty() ||
      result.road(0).passage(0).segment(0).id() != request.waypoint(0).id()) {
    add_terminal(0, root);
    for (int i = result.road_size() - 1; i > 0; --i) {
      result.mutable_road()->SwapElements(i, i - 1);
    }
  }
  if (result.road().rbegin()->passage(0).segment(0).id() !=
      request.waypoint().rbegin()->id()) {
    add_terminal(request.waypoint_size() - 1, goal);
  }
  result.set_map_version(map_version_);
  result.mutable_routing_request()->CopyFrom(request);
  result.mutable_measurement()->set_distance(labels.at(goal).distance);
  result.mutable_status()->set_error_code(common::OK);
  result.mutable_status()->set_msg("Success");
  response->Swap(&result);
  return SearchResult::FOUND;
}

}  // namespace routing
}  // namespace apollo
