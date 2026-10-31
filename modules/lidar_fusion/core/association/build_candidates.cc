// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/association/build_candidates.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <vector>

#include "Eigen/Core"

#include "modules/lidar_fusion/core/evidence/point_geometry.h"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {

ObjectCandidate* AddRecallCandidate(
    const FusionConfig& c, const std::vector<int>& ids,
    const Eigen::Vector3d& low, const Eigen::Vector3d& high, UnifiedClass cls,
    google::protobuf::RepeatedPtrField<PointDecision>* decisions,
    CandidateResult* candidates) {
  auto* object = candidates->objects.Add();
  object->set_id(candidates->objects.size());
  object->set_source(SEG_RECALL);
  object->set_classification_use(CLASSIFICATION_CANDIDATE);
  object->add_reason(SEMANTIC_RECALL);
  object->set_consistency(SUPPORT);
  object->set_unified_class(cls);
  object->set_group(THING);
  SetBox({(low + high) * 0.5,
          (high - low).cwiseMax(Eigen::Vector3d::Constant(c.resolution())),
          Eigen::Matrix3d::Identity()},
         object->mutable_box());
  for (int id : ids) {
    object->add_point_id(id);
    auto* decision = decisions->Mutable(id);
    decision->set_object_id(object->id());
    decision->set_explanation(EXPL_OBJECT);
    decision->set_source(SEG_RECALL);
    decision->add_reason(SEMANTIC_RECALL);
  }
  return object;
}

void RecallForeground(
    const FusionConfig& c, const std::vector<int>& ids,
    const std::vector<Eigen::Vector3d>& positions,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    google::protobuf::RepeatedPtrField<PointDecision>* decisions,
    CandidateResult* candidates) {
  std::map<UnifiedClass, std::vector<int>> foreground;
  for (int id : ids) {
    const auto& p = evidence.Get(id);
    if (p.group() == THING &&
        p.semantic_probability() >= c.semantic_min_confidence()) {
      foreground[p.unified_class()].push_back(id);
    }
  }
  const auto dominant = std::max_element(
      foreground.begin(), foreground.end(), [](const auto& a, const auto& b) {
        return a.second.size() < b.second.size();
      });
  if (dominant == foreground.end() ||
      dominant->second.size() < c.recall_min_points() ||
      dominant->second.size() < c.support_ratio() * ids.size()) {
    return;
  }
  std::vector<Eigen::Vector3d> points;
  points.reserve(dominant->second.size());
  for (int id : dominant->second) {
    points.push_back(positions[id]);
  }
  const std::vector<bool> eligible(points.size(), true);
  const ConnectedVoxels voxels(points, eligible, c.connectivity_size());
  const auto components = voxels.Components(eligible);
  std::map<int, std::vector<int>> clusters;
  for (std::size_t i = 0; i < components.size(); ++i) {
    clusters[components[i]].push_back(dominant->second[i]);
  }
  for (const auto& cluster : clusters) {
    if (cluster.second.size() < c.recall_min_points()) {
      continue;
    }
    Eigen::Vector3d low = positions[cluster.second.front()], high = low;
    for (int id : cluster.second) {
      low = low.cwiseMin(positions[id]);
      high = high.cwiseMax(positions[id]);
    }
    const Eigen::Vector3d size = high - low;
    if (size.head<2>().maxCoeff() < c.recall_min_size() ||
        size.maxCoeff() > c.recall_max_size()) {
      continue;
    }
    auto* object = AddRecallCandidate(c, cluster.second, low, high,
                                      dominant->first, decisions, candidates);
    object->add_reason(FOREGROUND_SUBSET_RECALL);
    for (int id : cluster.second) {
      decisions->Mutable(id)->add_reason(FOREGROUND_SUBSET_RECALL);
    }
  }
}

void BuildCandidates(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<int>& scene_components, const ConnectedVoxels& voxels,
    const FusionConfig& c,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    google::protobuf::RepeatedPtrField<PointDecision>* decisions,
    CandidateResult* candidates) {
  std::map<int, std::set<uint32_t>> scene_objects;
  for (int i = 0; i < decisions->size(); ++i) {
    if (scene_components[i] >= 0 && decisions->Get(i).has_object_id()) {
      scene_objects[scene_components[i]].insert(decisions->Get(i).object_id());
    }
  }
  std::vector<bool> eligible(positions.size(), false);
  for (int i = 0; i < evidence.size(); ++i) {
    const auto& p = evidence.Get(i);
    const auto explanation = decisions->Get(i).explanation();
    eligible[i] =
        p.finite() && InRoi(positions[i], c) && ObstacleHeight(p, c) &&
        (explanation == UNEXPLAINED || explanation == EXPL_OBJECT_WEAK ||
         (p.in_corridor() &&
          (explanation == EXPL_STATIC || explanation == EXPL_VEG_HIGH)));
  }
  const auto components = c.candidates().separate_semantic_instances() ||
                                  c.candidates().has_far_connectivity_size()
                              ? CandidateComponents(c, evidence, *decisions)
                              : voxels.Components(eligible);
  std::map<int, std::vector<int>> clusters;
  for (std::size_t i = 0; i < components.size(); ++i) {
    if (components[i] >= 0) {
      clusters[components[i]].push_back(i);
    }
  }
  for (const auto& cluster : clusters) {
    Eigen::Vector3d low = positions[cluster.second.front()];
    Eigen::Vector3d high = low;
    bool corridor = false;
    bool all_thing = true;
    std::map<UnifiedClass, int> votes;
    std::set<uint32_t> related;
    bool semantic_conflict = false;
    for (int id : cluster.second) {
      low = low.cwiseMin(positions[id]);
      high = high.cwiseMax(positions[id]);
      const auto& p = evidence.Get(id);
      const auto& d = decisions->Get(id);
      corridor |= p.in_corridor();
      all_thing &= p.group() == THING &&
                   p.semantic_probability() >= c.semantic_min_confidence();
      ++votes[p.unified_class()];
      if (d.has_object_id()) {
        related.insert(d.object_id());
      }
      const auto nearby = scene_objects.find(scene_components[id]);
      if (!c.candidates().separate_semantic_instances() &&
          !c.candidates().local_object_relations() &&
          nearby != scene_objects.end()) {
        related.insert(nearby->second.begin(), nearby->second.end());
      }
      semantic_conflict |=
          (p.group() == SURFACE || p.group() == SURFACE_SOFT) &&
          p.ground_valid() &&
          p.height_above_ground() >= c.obstacle_min_height();
    }
    const Eigen::Vector3d size = high - low;
    const bool recall = !c.quality().semantic_identity_unverified() &&
                        all_thing && related.empty() &&
                        cluster.second.size() >= c.recall_min_points() &&
                        size.head<2>().maxCoeff() >= c.recall_min_size() &&
                        size.maxCoeff() <= c.recall_max_size();
    if (recall) {
      const auto vote = std::max_element(
          votes.begin(), votes.end(),
          [](const auto& a, const auto& b) { return a.second < b.second; });
      AddRecallCandidate(c, cluster.second, low, high, vote->first, decisions,
                         candidates);
      continue;
    }
    const bool retain_unknown =
        corridor || cluster.second.size() >= c.unknown_min_points();
    if (c.candidates().recall_confident_foreground() && retain_unknown &&
        !all_thing && related.empty() &&
        !c.quality().semantic_identity_unverified()) {
      RecallForeground(c, cluster.second, positions, evidence, decisions,
                       candidates);
      // The mixed component remains an independent unknown risk.
    }
    if (!retain_unknown) {
      continue;
    }
    auto* unknown = candidates->unknowns.Add();
    unknown->set_point_supported_footprint(
        c.quality().point_supported_unknown());
    unknown->set_id(candidates->unknowns.size());
    unknown->set_source(GEOMETRY);
    unknown->set_in_corridor(corridor);
    unknown->set_z_min(low.z());
    unknown->set_z_max(high.z());
    // Pad the footprint by half a cell so single returns have nonzero area.
    const double pad = c.resolution() * 0.5;
    for (const auto& xy :
         std::array<CellKey, 4>{{{0, 0}, {1, 0}, {1, 1}, {0, 1}}}) {
      auto* p = unknown->add_polygon();
      p->set_x(xy.first ? high.x() + pad : low.x() - pad);
      p->set_y(xy.second ? high.y() + pad : low.y() - pad);
      p->set_z(low.z());
    }
    for (int id : cluster.second) {
      unknown->add_point_id(id);
    }
    for (uint32_t id : related) {
      unknown->add_related_object_id(id);
    }
    if (!related.empty()) {
      unknown->add_reason(AMBIGUOUS_OBJECT_RELATION);
    }
    if (semantic_conflict) {
      unknown->add_reason(SEM_GEO_CONFLICT);
    }
    if (corridor && cluster.second.size() < c.unknown_min_points()) {
      unknown->add_reason(CORRIDOR_SMALL_CLUSTER);
    }
    if (unknown->reason().empty()) {
      unknown->add_reason(UNEXPLAINED_GEOMETRY);
    }
  }
}

// Conservatively rasterize the projected box AABB, including tilted boxes.
// Inferred occupancy is separate; no inferred cell is labelled observed/free.

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
