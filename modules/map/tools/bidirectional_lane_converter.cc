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

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace apollo {
namespace hdmap {
namespace tools {
namespace {

constexpr double kSIndexTolerance = 1e-3;
constexpr double kGeometryJoinTolerance = 5e-2;
constexpr char kReverseSuffix[] = "_reverse";

using LaneIndex = std::unordered_map<std::string, int>;
using ReverseIdMap = std::unordered_map<std::string, std::string>;

bool Fail(std::string* error, const std::string& message) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

std::string ReverseId(const std::string& lane_id) {
  return lane_id + kReverseSuffix;
}

bool IsWithin(double value, double total) {
  return std::isfinite(value) && value >= -kSIndexTolerance &&
         value <= total + kSIndexTolerance;
}

double ReverseS(double value, double total) {
  const double reversed = total - value;
  if (std::fabs(reversed) <= kSIndexTolerance) {
    return 0.0;
  }
  if (std::fabs(reversed - total) <= kSIndexTolerance) {
    return total;
  }
  return reversed;
}

void AddMapId(const Id& id, std::unordered_set<std::string>* ids) {
  if (!id.has_id() || id.id().empty()) {
    return;
  }
  ids->insert(id.id());
}

void CollectMapIds(const Map& map, std::unordered_set<std::string>* ids) {
  const auto collect = [ids](const auto& elements) {
    for (const auto& element : elements) {
      if (element.has_id()) {
        AddMapId(element.id(), ids);
      }
    }
  };
  collect(map.crosswalk());
  collect(map.junction());
  collect(map.lane());
  collect(map.stop_sign());
  collect(map.signal());
  collect(map.yield());
  collect(map.overlap());
  collect(map.clear_area());
  collect(map.speed_bump());
  collect(map.road());
  collect(map.parking_space());
  collect(map.pnc_junction());
  collect(map.rsu());
  for (const auto& road : map.road()) {
    for (const auto& section : road.section()) {
      if (section.has_id()) {
        AddMapId(section.id(), ids);
      }
    }
  }
  for (const auto& overlap : map.overlap()) {
    for (const auto& region : overlap.region_overlap()) {
      if (region.has_id()) {
        AddMapId(region.id(), ids);
      }
    }
  }
  for (const auto& junction : map.pnc_junction()) {
    for (const auto& group : junction.passage_group()) {
      if (group.has_id()) {
        AddMapId(group.id(), ids);
      }
      for (const auto& passage : group.passage()) {
        if (passage.has_id()) {
          AddMapId(passage.id(), ids);
        }
      }
    }
  }
}

bool BuildLaneIndex(const Map& map, LaneIndex* lane_index,
                    std::string* error) {
  for (int i = 0; i < map.lane_size(); ++i) {
    const auto& lane = map.lane(i);
    if (!lane.has_id() || lane.id().id().empty()) {
      return Fail(error, "A lane has no ID.");
    }
    if (!lane_index->emplace(lane.id().id(), i).second) {
      return Fail(error, "Duplicate lane ID: " + lane.id().id());
    }
  }
  return true;
}

bool BuildOverlapIndex(const Map& map,
                       std::unordered_map<std::string, int>* overlap_index,
                       std::string* error) {
  for (int i = 0; i < map.overlap_size(); ++i) {
    const auto& overlap = map.overlap(i);
    if (!overlap.has_id() || overlap.id().id().empty()) {
      return Fail(error, "An overlap has no ID.");
    }
    if (!overlap_index->emplace(overlap.id().id(), i).second) {
      return Fail(error, "Duplicate overlap ID: " + overlap.id().id());
    }
  }
  return true;
}

bool PolylineLength(const LineSegment& line, double* length) {
  if (line.point_size() < 2) {
    return false;
  }
  for (const auto& point : line.point()) {
    if (!point.has_x() || !point.has_y()) {
      return false;
    }
  }
  *length = 0.0;
  for (int i = 1; i < line.point_size(); ++i) {
    const auto& previous = line.point(i - 1);
    const auto& current = line.point(i);
    *length += std::hypot(current.x() - previous.x(),
                          current.y() - previous.y());
  }
  return std::isfinite(*length) && *length > 0.0;
}

double GetCurveLength(const Curve& curve) {
  double length = 0.0;
  for (const auto& segment : curve.segment()) {
    length += segment.length();
  }
  return length;
}

bool ReverseCurve(const Curve& source, Curve* destination,
                  const std::string& lane_id, const std::string& curve_name,
                  bool allow_empty, std::string* error) {
  if (source.segment_size() == 0) {
    if (allow_empty) {
      destination->Clear();
      return true;
    }
    return Fail(error, "Lane " + lane_id + " has no " + curve_name + ".");
  }

  std::vector<double> lengths;
  lengths.reserve(source.segment_size());
  double total_length = 0.0;
  for (int i = 0; i < source.segment_size(); ++i) {
    const auto& segment = source.segment(i);
    if (!segment.has_line_segment() || !segment.has_s() ||
        !segment.has_length()) {
      return Fail(error, "Lane " + lane_id + " has an unsupported " +
                             curve_name + " segment.");
    }
    double geometric_length = 0.0;
    if (!PolylineLength(segment.line_segment(), &geometric_length) ||
        !std::isfinite(segment.s()) || !std::isfinite(segment.length()) ||
        segment.length() <= 0.0) {
      return Fail(error, "Lane " + lane_id + " has invalid " + curve_name +
                             " segment geometry.");
    }
    if (std::fabs(segment.s() - total_length) > kSIndexTolerance) {
      return Fail(error, "Lane " + lane_id + " has non-contiguous " +
                             curve_name + " s-indices.");
    }
    if (i > 0) {
      const auto& previous_segment = source.segment(i - 1).line_segment();
      const auto& previous = previous_segment.point(
          previous_segment.point_size() - 1);
      const auto& current = segment.line_segment().point(0);
      if (std::hypot(previous.x() - current.x(),
                     previous.y() - current.y()) > kGeometryJoinTolerance) {
        return Fail(error, "Lane " + lane_id + " has disconnected " +
                               curve_name + " segments.");
      }
    }
    lengths.push_back(segment.length());
    total_length += segment.length();
  }

  destination->Clear();
  double reverse_s = 0.0;
  for (int i = source.segment_size() - 1; i >= 0; --i) {
    const auto& source_segment = source.segment(i);
    auto* segment = destination->add_segment();
    segment->CopyFrom(source_segment);
    segment->set_s(reverse_s);
    reverse_s += lengths[i];

    auto* points = segment->mutable_line_segment()->mutable_point();
    std::reverse(points->begin(), points->end());
    const auto& first = points->Get(0);
    int second_index = 1;
    while (second_index < points->size() &&
           std::hypot(points->Get(second_index).x() - first.x(),
                      points->Get(second_index).y() - first.y()) <=
               kSIndexTolerance) {
      ++second_index;
    }
    if (second_index == points->size()) {
      return Fail(error, "Lane " + lane_id + " has a degenerate " +
                             curve_name + " segment.");
    }
    segment->mutable_start_position()->CopyFrom(first);
    segment->set_heading(
        std::atan2(points->Get(second_index).y() - first.y(),
                   points->Get(second_index).x() - first.x()));
  }
  if (std::fabs(reverse_s - total_length) > kSIndexTolerance) {
    return Fail(error, "Lane " + lane_id + " has invalid " + curve_name +
                           " length.");
  }
  return true;
}

bool ReverseBoundaryTypes(const LaneBoundary& source,
                          LaneBoundary* destination,
                          const std::string& lane_id,
                          const std::string& boundary_name,
                          std::string* error) {
  if (source.boundary_type_size() == 0) {
    return true;
  }
  if (!source.has_length() || !std::isfinite(source.length()) ||
      source.length() <= 0.0) {
    return Fail(error, "Lane " + lane_id + " has invalid " + boundary_name +
                           " length.");
  }
  double previous_s = -kSIndexTolerance;
  for (const auto& type : source.boundary_type()) {
    const double s = type.has_s() ? type.s() : 0.0;
    if (!IsWithin(s, source.length()) || s < previous_s ||
        type.types_size() == 0) {
      return Fail(error, "Lane " + lane_id + " has invalid " + boundary_name +
                             " type s-indices.");
    }
    previous_s = s;
  }
  destination->clear_boundary_type();
  for (int i = source.boundary_type_size() - 1; i >= 0; --i) {
    const auto& source_type = source.boundary_type(i);
    auto* type = destination->add_boundary_type();
    type->CopyFrom(source_type);
    // Boundary types are piecewise constant from each s to the next one.
    const double end_s =
        i + 1 < source.boundary_type_size()
            ? source.boundary_type(i + 1).s()
            : source.length();
    type->set_s(ReverseS(end_s, source.length()));
  }
  return true;
}

bool ReverseBoundary(const LaneBoundary& source, LaneBoundary* destination,
                     const std::string& lane_id,
                     const std::string& boundary_name,
                     std::string* error) {
  destination->CopyFrom(source);
  if (source.has_curve() &&
      !ReverseCurve(source.curve(), destination->mutable_curve(), lane_id,
                    boundary_name, true, error)) {
    return false;
  }
  if (source.has_curve() && source.curve().segment_size() > 0 &&
      source.has_length() &&
      (!std::isfinite(source.length()) ||
       std::fabs(GetCurveLength(source.curve()) - source.length()) >
           kSIndexTolerance)) {
    return Fail(error, "Lane " + lane_id + " has inconsistent " +
                           boundary_name + " length.");
  }
  return ReverseBoundaryTypes(source, destination, lane_id, boundary_name,
                              error);
}

template <typename AssociationList>
bool ReverseAssociations(const AssociationList& source,
                         AssociationList* destination, double lane_length,
                         const std::string& lane_id,
                         const std::string& association_name,
                         std::string* error) {
  destination->Clear();
  if (source.size() == 0) {
    return true;
  }
  double previous_s = lane_length + kSIndexTolerance;
  for (int i = source.size() - 1; i >= 0; --i) {
    const auto& association = source.Get(i);
    if (!association.has_s() || !IsWithin(association.s(), lane_length) ||
        association.s() > previous_s || !association.has_width() ||
        !std::isfinite(association.width()) || association.width() < 0.0) {
      return Fail(error, "Lane " + lane_id + " has invalid " +
                             association_name + " s-index or width.");
    }
    previous_s = association.s();
    if (i == source.size() - 1 && association.s() < lane_length) {
      auto* start = destination->Add();
      start->CopyFrom(association);
      start->set_s(0.0);
    }
    auto* reversed = destination->Add();
    reversed->CopyFrom(association);
    reversed->set_s(ReverseS(association.s(), lane_length));
  }
  if (source.Get(0).s() > 0.0) {
    auto* end = destination->Add();
    end->CopyFrom(source.Get(0));
    end->set_s(lane_length);
  }
  return true;
}

bool ReverseLaneLinks(const Lane& source, Lane* destination,
                      const LaneIndex& lane_index,
                      const ReverseIdMap& reverse_ids,
                      const std::string& lane_id, std::string* error) {
  destination->clear_predecessor_id();
  destination->clear_successor_id();
  for (const auto& successor : source.successor_id()) {
    const auto target = lane_index.find(successor.id());
    if (target == lane_index.end() ||
        reverse_ids.find(successor.id()) == reverse_ids.end()) {
      return Fail(error, "Bidirectional lane " + lane_id +
                             " has a successor without a reverse lane: " +
                             successor.id());
    }
    destination->add_predecessor_id()->set_id(reverse_ids.at(successor.id()));
  }
  for (const auto& predecessor : source.predecessor_id()) {
    const auto target = lane_index.find(predecessor.id());
    if (target == lane_index.end() ||
        reverse_ids.find(predecessor.id()) == reverse_ids.end()) {
      return Fail(error, "Bidirectional lane " + lane_id +
                             " has a predecessor without a reverse lane: " +
                             predecessor.id());
    }
    destination->add_successor_id()->set_id(reverse_ids.at(predecessor.id()));
  }
  return true;
}

bool HasLaneId(const google::protobuf::RepeatedPtrField<Id>& ids,
               const std::string& lane_id) {
  return std::any_of(ids.begin(), ids.end(),
                     [&lane_id](const Id& id) { return id.id() == lane_id; });
}

bool ValidateBidirectionalLinks(const Map& map, const LaneIndex& lane_index,
                                const ReverseIdMap& reverse_ids,
                                std::string* error) {
  for (const auto& [lane_id, reverse_id] : reverse_ids) {
    static_cast<void>(reverse_id);
    const auto& lane = map.lane(lane_index.at(lane_id));
    for (const auto& successor : lane.successor_id()) {
      const auto target = lane_index.find(successor.id());
      if (target == lane_index.end() ||
          reverse_ids.find(successor.id()) == reverse_ids.end()) {
        return Fail(error, "Bidirectional lane " + lane_id +
                               " has a successor without a reverse lane: " +
                               successor.id());
      }
      if (!HasLaneId(map.lane(target->second).predecessor_id(), lane_id)) {
        return Fail(error, "Lane successor/predecessor references are not "
                           "reciprocal for " +
                               lane_id + " and " + successor.id() + ".");
      }
    }
    for (const auto& predecessor : lane.predecessor_id()) {
      const auto target = lane_index.find(predecessor.id());
      if (target == lane_index.end() ||
          reverse_ids.find(predecessor.id()) == reverse_ids.end()) {
        return Fail(error, "Bidirectional lane " + lane_id +
                               " has a predecessor without a reverse lane: " +
                               predecessor.id());
      }
      if (!HasLaneId(map.lane(target->second).successor_id(), lane_id)) {
        return Fail(error, "Lane predecessor/successor references are not "
                           "reciprocal for " +
                               lane_id + " and " + predecessor.id() + ".");
      }
    }
  }
  return true;
}

bool ReverseNeighborLists(const Lane& source, Lane* destination,
                          const LaneIndex& lane_index,
                          const ReverseIdMap& reverse_ids,
                          const std::string& lane_id, std::string* error) {
  const auto reverse_neighbors =
      [&](const google::protobuf::RepeatedPtrField<Id>& source_ids,
          google::protobuf::RepeatedPtrField<Id>* same_direction_ids,
          google::protobuf::RepeatedPtrField<Id>* opposite_direction_ids) {
        for (const auto& neighbor : source_ids) {
          if (lane_index.find(neighbor.id()) == lane_index.end()) {
            return false;
          }
          const auto reverse = reverse_ids.find(neighbor.id());
          auto* reversed = (reverse == reverse_ids.end()
                                ? opposite_direction_ids
                                : same_direction_ids)->Add();
          reversed->set_id(reverse == reverse_ids.end() ? neighbor.id()
                                                         : reverse->second);
        }
        return true;
      };
  const auto copy_opposite_neighbors =
      [&](const google::protobuf::RepeatedPtrField<Id>& source_ids,
          google::protobuf::RepeatedPtrField<Id>* same_direction_ids) {
        for (const auto& neighbor : source_ids) {
          if (lane_index.find(neighbor.id()) == lane_index.end()) {
            return false;
          }
          same_direction_ids->Add()->CopyFrom(neighbor);
        }
        return true;
      };

  destination->clear_left_neighbor_forward_lane_id();
  destination->clear_right_neighbor_forward_lane_id();
  destination->clear_left_neighbor_reverse_lane_id();
  destination->clear_right_neighbor_reverse_lane_id();
  if (!reverse_neighbors(source.right_neighbor_forward_lane_id(),
                         destination->mutable_left_neighbor_forward_lane_id(),
                         destination->mutable_left_neighbor_reverse_lane_id()) ||
      !reverse_neighbors(source.left_neighbor_forward_lane_id(),
                         destination->mutable_right_neighbor_forward_lane_id(),
                         destination->mutable_right_neighbor_reverse_lane_id()) ||
      !copy_opposite_neighbors(
          source.right_neighbor_reverse_lane_id(),
          destination->mutable_left_neighbor_forward_lane_id()) ||
      !copy_opposite_neighbors(
          source.left_neighbor_reverse_lane_id(),
          destination->mutable_right_neighbor_forward_lane_id())) {
    return Fail(error, "Bidirectional lane " + lane_id +
                           " has a neighbor reference to a missing lane.");
  }
  return true;
}

bool FindRoadSection(const Map& map, const std::string& lane_id,
                     int* road_index, int* section_index,
                     std::string* error) {
  int matches = 0;
  for (int road = 0; road < map.road_size(); ++road) {
    for (int section = 0; section < map.road(road).section_size(); ++section) {
      for (const auto& section_lane : map.road(road).section(section).lane_id()) {
        if (section_lane.id() == lane_id) {
          ++matches;
          *road_index = road;
          *section_index = section;
        }
      }
    }
  }
  if (matches != 1) {
    return Fail(error, "Bidirectional lane " + lane_id +
                           " must belong to exactly one road section.");
  }
  return true;
}

bool ValidatePncPassages(const Map& map,
                         const std::unordered_set<std::string>& bidirectional,
                         std::string* error) {
  for (const auto& junction : map.pnc_junction()) {
    for (const auto& group : junction.passage_group()) {
      for (const auto& passage : group.passage()) {
        for (const auto& lane_id : passage.lane_id()) {
          if (bidirectional.find(lane_id.id()) != bidirectional.end()) {
            return Fail(error, "Bidirectional lane " + lane_id.id() +
                                   " is referenced by a PNC junction passage.");
          }
        }
      }
    }
  }
  return true;
}

bool ValidateOverlapReferences(
    const Map& map, const Lane& lane,
    const std::unordered_map<std::string, int>& overlap_index,
    std::string* error) {
  std::unordered_set<std::string> referenced;
  for (const auto& overlap_id : lane.overlap_id()) {
    if (!referenced.insert(overlap_id.id()).second) {
      return Fail(error, "Lane " + lane.id().id() +
                             " has a duplicate overlap reference.");
    }
    const auto overlap = overlap_index.find(overlap_id.id());
    if (overlap == overlap_index.end()) {
      return Fail(error, "Lane " + lane.id().id() +
                             " references a missing overlap: " +
                             overlap_id.id());
    }
    int lane_matches = 0;
    for (const auto& object : map.overlap(overlap->second).object()) {
      if (object.has_id() && object.id().id() == lane.id().id()) {
        if (!object.has_lane_overlap_info()) {
          return Fail(error, "Lane " + lane.id().id() +
                                 " has an unsupported overlap object.");
        }
        ++lane_matches;
      }
    }
    if (lane_matches != 1) {
      return Fail(error, "Lane " + lane.id().id() +
                             " has a missing or duplicate overlap object.");
    }
  }
  for (const auto& overlap : map.overlap()) {
    for (const auto& object : overlap.object()) {
      if (object.has_id() && object.id().id() == lane.id().id() &&
          referenced.find(overlap.id().id()) == referenced.end()) {
        return Fail(error, "Lane " + lane.id().id() +
                               " is referenced by an unlisted overlap.");
      }
    }
  }
  return true;
}

bool DuplicateLaneOverlaps(Map* map, const Lane& source,
                           const Lane& reverse_lane,
                           const std::unordered_map<std::string, int>&
                               overlap_index,
                           std::string* error) {
  for (const auto& overlap_id : source.overlap_id()) {
    auto* overlap = map->mutable_overlap(overlap_index.at(overlap_id.id()));
    int lane_object_index = -1;
    for (int i = 0; i < overlap->object_size(); ++i) {
      if (overlap->object(i).has_id() &&
          overlap->object(i).id().id() == source.id().id()) {
        lane_object_index = i;
        break;
      }
    }
    if (lane_object_index < 0) {
      return Fail(error, "Lane " + source.id().id() +
                             " lost its overlap object during conversion.");
    }
    for (const auto& object : overlap->object()) {
      if (object.has_id() && object.id().id() == reverse_lane.id().id()) {
        return Fail(error, "Generated reverse lane ID already occurs in an "
                           "overlap: " +
                               reverse_lane.id().id());
      }
    }
    const ObjectOverlapInfo source_object =
        overlap->object(lane_object_index);
    const auto& source_info = source_object.lane_overlap_info();
    if (!source_info.has_start_s() || !source_info.has_end_s() ||
        !IsWithin(source_info.start_s(), source.length()) ||
        !IsWithin(source_info.end_s(), source.length()) ||
        source_info.start_s() > source_info.end_s()) {
      return Fail(error, "Lane " + source.id().id() +
                             " has invalid lane overlap s-indices.");
    }
    const double reverse_start_s =
        ReverseS(source_info.end_s(), source.length());
    const double reverse_end_s =
        ReverseS(source_info.start_s(), source.length());
    auto* reverse_object = overlap->add_object();
    reverse_object->CopyFrom(source_object);
    reverse_object->mutable_id()->set_id(reverse_lane.id().id());
    auto* reverse_info = reverse_object->mutable_lane_overlap_info();
    reverse_info->set_start_s(reverse_start_s);
    reverse_info->set_end_s(reverse_end_s);
  }
  return true;
}

bool BuildReverseLane(const Lane& source, const std::string& reverse_id,
                      const LaneIndex& lane_index,
                      const ReverseIdMap& reverse_ids, Lane* reverse_lane,
                      std::string* error) {
  const std::string& lane_id = source.id().id();
  if (!source.has_length() || !std::isfinite(source.length()) ||
      source.length() <= 0.0) {
    return Fail(error, "Bidirectional lane " + lane_id +
                           " has invalid lane length.");
  }
  if (source.self_reverse_lane_id_size() != 0) {
    return Fail(error, "Bidirectional lane " + lane_id +
                           " already has a self-reverse reference.");
  }
  reverse_lane->CopyFrom(source);
  reverse_lane->mutable_id()->set_id(reverse_id);
  reverse_lane->set_direction(Lane::BACKWARD);
  if (source.has_turn()) {
    if (source.turn() == Lane::LEFT_TURN) {
      reverse_lane->set_turn(Lane::RIGHT_TURN);
    } else if (source.turn() == Lane::RIGHT_TURN) {
      reverse_lane->set_turn(Lane::LEFT_TURN);
    }
  }

  if (!source.has_central_curve()) {
    return Fail(error, "Bidirectional lane " + lane_id +
                           " has no central curve.");
  }
  if (!ReverseCurve(source.central_curve(),
                    reverse_lane->mutable_central_curve(), lane_id,
                    "central curve", false, error)) {
    return false;
  }
  if (std::fabs(GetCurveLength(source.central_curve()) - source.length()) >
      kSIndexTolerance) {
    return Fail(error, "Bidirectional lane " + lane_id +
                           " has inconsistent central curve length.");
  }
  const auto& last_segment = source.central_curve().segment(
      source.central_curve().segment_size() - 1);
  if (std::fabs(last_segment.s() + last_segment.length() - source.length()) >
      kSIndexTolerance) {
    return Fail(error, "Bidirectional lane " + lane_id +
                           " has inconsistent centerline and lane lengths.");
  }
  if (source.has_right_boundary() &&
      !ReverseBoundary(source.right_boundary(),
                       reverse_lane->mutable_left_boundary(), lane_id,
                       "right boundary", error)) {
    return false;
  } else if (!source.has_right_boundary()) {
    reverse_lane->clear_left_boundary();
  }
  if (source.has_left_boundary() &&
      !ReverseBoundary(source.left_boundary(),
                       reverse_lane->mutable_right_boundary(), lane_id,
                       "left boundary", error)) {
    return false;
  } else if (!source.has_left_boundary()) {
    reverse_lane->clear_right_boundary();
  }
  if (!ReverseAssociations(source.right_sample(),
                           reverse_lane->mutable_left_sample(),
                           source.length(), lane_id, "right sample", error) ||
      !ReverseAssociations(source.left_sample(),
                           reverse_lane->mutable_right_sample(),
                           source.length(), lane_id, "left sample", error) ||
      !ReverseAssociations(source.right_road_sample(),
                           reverse_lane->mutable_left_road_sample(),
                           source.length(), lane_id, "right road sample",
                           error) ||
      !ReverseAssociations(source.left_road_sample(),
                           reverse_lane->mutable_right_road_sample(),
                           source.length(), lane_id, "left road sample",
                           error)) {
    return false;
  }
  if (!ReverseLaneLinks(source, reverse_lane, lane_index, reverse_ids, lane_id,
                        error) ||
      !ReverseNeighborLists(source, reverse_lane, lane_index, reverse_ids,
                            lane_id, error)) {
    return false;
  }

  reverse_lane->clear_self_reverse_lane_id();
  reverse_lane->add_self_reverse_lane_id()->set_id(lane_id);
  return true;
}

bool Convert(Map* map, std::string* error) {
  LaneIndex lane_index;
  if (!BuildLaneIndex(*map, &lane_index, error)) {
    return false;
  }

  std::unordered_set<std::string> all_ids;
  CollectMapIds(*map, &all_ids);
  ReverseIdMap reverse_ids;
  std::unordered_set<std::string> bidirectional;
  std::vector<std::string> bidirectional_lane_ids;
  for (const auto& lane : map->lane()) {
    if (lane.direction() != Lane::BIDIRECTION) {
      continue;
    }
    const auto& lane_id = lane.id().id();
    const std::string reverse_id = ReverseId(lane_id);
    if (!all_ids.insert(reverse_id).second) {
      return Fail(error, "Generated reverse lane ID collides with an existing "
                         "map ID: " +
                             reverse_id);
    }
    reverse_ids.emplace(lane_id, reverse_id);
    bidirectional.insert(lane_id);
    bidirectional_lane_ids.push_back(lane_id);
  }
  if (bidirectional.empty()) {
    return true;
  }
  if (!ValidateBidirectionalLinks(*map, lane_index, reverse_ids, error)) {
    return false;
  }
  if (!ValidatePncPassages(*map, bidirectional, error)) {
    return false;
  }

  std::unordered_map<std::string, int> overlap_index;
  if (!BuildOverlapIndex(*map, &overlap_index, error)) {
    return false;
  }

  std::unordered_map<std::string, std::pair<int, int>> road_sections;
  for (const auto& lane_id : bidirectional_lane_ids) {
    int road_index = -1;
    int section_index = -1;
    if (!FindRoadSection(*map, lane_id, &road_index, &section_index, error) ||
        !ValidateOverlapReferences(*map, map->lane(lane_index.at(lane_id)),
                                   overlap_index, error)) {
      return false;
    }
    road_sections.emplace(lane_id,
                          std::make_pair(road_index, section_index));
  }

  std::vector<Lane> reverse_lanes;
  reverse_lanes.reserve(bidirectional_lane_ids.size());
  for (const auto& lane_id : bidirectional_lane_ids) {
    const auto& source = map->lane(lane_index.at(lane_id));
    Lane reverse_lane;
    if (!BuildReverseLane(source, reverse_ids.at(lane_id), lane_index,
                          reverse_ids, &reverse_lane, error)) {
      return false;
    }
    reverse_lanes.push_back(std::move(reverse_lane));
  }

  for (const auto& lane_id : bidirectional) {
    auto* source = map->mutable_lane(lane_index.at(lane_id));
    source->set_direction(Lane::FORWARD);
    source->add_self_reverse_lane_id()->set_id(reverse_ids.at(lane_id));
  }

  for (const auto& reverse_lane : reverse_lanes) {
    const auto& source = map->lane(lane_index.at(
        reverse_lane.self_reverse_lane_id(0).id()));
    if (!DuplicateLaneOverlaps(map, source, reverse_lane, overlap_index, error)) {
      return false;
    }
    const auto& road_section = road_sections.at(source.id().id());
    map->mutable_road(road_section.first)
        ->mutable_section(road_section.second)
        ->add_lane_id()
        ->set_id(reverse_lane.id().id());
    map->add_lane()->CopyFrom(reverse_lane);
  }
  return true;
}

}  // namespace

bool ConvertBidirectionalLanes(Map* map, std::string* error) {
  if (map == nullptr) {
    return Fail(error, "Map is null.");
  }
  if (error != nullptr) {
    error->clear();
  }
  Map converted;
  converted.CopyFrom(*map);
  if (!Convert(&converted, error)) {
    return false;
  }
  map->Swap(&converted);
  return true;
}

}  // namespace tools
}  // namespace hdmap
}  // namespace apollo
