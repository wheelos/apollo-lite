// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/association/detection_association.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "Eigen/Geometry"

#include "modules/lidar_fusion/core/alignment/input_validation.h"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {
using alignment_internal::Mappings;
namespace {
double Cross(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return a.x() * b.y() - a.y() * b.x();
}

std::vector<Eigen::Vector2d> Footprint(const Detection& box) {
  std::vector<Eigen::Vector2d> polygon;
  const Eigen::Rotation2Dd r(box.yaw());
  for (const auto& corner :
       std::array<CellKey, 4>{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}}) {
    polygon.push_back(Eigen::Vector2d(box.center_x(), box.center_y()) +
                      r * Eigen::Vector2d(corner.first * box.size_x() * 0.5,
                                          corner.second * box.size_y() * 0.5));
  }
  return polygon;
}

double BevIou(const Detection& a, const Detection& b) {
  auto polygon = Footprint(a);
  const auto clip = Footprint(b);
  for (std::size_t edge = 0; edge < clip.size() && !polygon.empty(); ++edge) {
    const auto& start = clip[edge];
    const auto direction = clip[(edge + 1) % clip.size()] - start;
    std::vector<Eigen::Vector2d> next;
    auto previous = polygon.back();
    double previous_side = Cross(direction, previous - start);
    for (const auto& point : polygon) {
      const double side = Cross(direction, point - start);
      if ((side >= 0) != (previous_side >= 0)) {
        const double fraction = previous_side / (previous_side - side);
        next.push_back(previous + fraction * (point - previous));
      }
      if (side >= 0) {
        next.push_back(point);
      }
      previous = point;
      previous_side = side;
    }
    polygon = std::move(next);
  }
  double area = 0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    area += Cross(polygon[i], polygon[(i + 1) % polygon.size()]);
  }
  area = std::abs(area) * 0.5;
  const double denominator =
      a.size_x() * a.size_y() + b.size_x() * b.size_y() - area;
  return denominator > 0 ? area / denominator : 0;
}

bool SemanticCompatible(const PointEvidence& point, UnifiedClass cls,
                        const FusionConfig& c) {
  return point.group() == THING && point.unified_class() == cls &&
         point.semantic_probability() >= c.semantic_min_confidence();
}
}  // namespace

std::vector<OrientedBox> Associate(
    const DetectionFrame* detection, const FusionConfig& c,
    const std::vector<Eigen::Vector3d>& positions, const SpatialIndex& index,
    const std::vector<int>& components, FusionOutput* out) {
  std::vector<OrientedBox> boxes;
  if (detection == nullptr) {
    return boxes;
  }
  const auto& t = c.sensor_to_base();
  const auto rotation = Rotation(t.roll(), t.pitch(), t.yaw());
  const Eigen::Vector3d translation(t.x(), t.y(), t.z());
  const auto mappings = Mappings(c.detection_mapping());
  std::vector<double> best(positions.size(), -1);
  std::vector<int> owner(positions.size(), -1);
  std::vector<std::set<int>> instance_support;
  std::vector<Detection> instance_boxes;
  for (int det_index = 0; det_index < detection->detection_size();
       ++det_index) {
    const auto& det = detection->detection(det_index);
    if (det.score() < c.evidence_score_min()) {
      continue;
    }
    OrientedBox box{rotation * Eigen::Vector3d(det.center_x(), det.center_y(),
                                               det.center_z()) +
                        translation,
                    Eigen::Vector3d(det.size_x(), det.size_y(), det.size_z()),
                    rotation * Rotation(0, 0, det.yaw())};
    boxes.push_back(box);
    auto* hypothesis = out->add_hypothesis();
    hypothesis->set_detection_index(det_index);
    *hypothesis->mutable_raw_detection() = det;
    SetBox(box, hypothesis->mutable_box());
    hypothesis->set_unified_class(mappings.at(det.class_id()).unified_class());
    hypothesis->set_group(mappings.at(det.class_id()).group());
    std::vector<int> core;
    std::vector<int> margin;
    std::set<int> core_components;
    const auto range = BoxRange(box, c, c.association_margin());
    for (int x = range.first.first; x <= range.second.first; ++x) {
      for (int y = range.first.second; y <= range.second.second; ++y) {
        auto found = index.find({x, y});
        if (found == index.end()) {
          continue;
        }
        for (int id : found->second) {
          const auto& p = out->point_evidence(id);
          if (components[id] < 0 || !ObstacleHeight(p, c)) {
            continue;
          }
          if (InBox(positions[id], box, 0)) {
            core.push_back(id);
            core_components.insert(components[id]);
          } else if (InBox(positions[id], box, c.association_margin())) {
            margin.push_back(id);
          }
        }
      }
    }
    auto* stats = hypothesis->mutable_association();
    stats->set_n_core(core.size());
    auto* semantic = stats->mutable_semantic_evidence();
    semantic->set_source_available(out->health().seg_valid());
    semantic->set_identity_qualified(
        out->health().seg_valid() &&
        !c.quality().semantic_identity_unverified());
    std::map<UnifiedClass, std::vector<uint32_t>> class_support;
    std::map<Group, uint32_t> group_support;
    uint32_t unavailable = 0, low_confidence = 0;
    int same = 0;
    int conflict = 0;
    for (int id : core) {
      const auto& p = out->point_evidence(id);
      same += SemanticCompatible(p, hypothesis->unified_class(), c);
      conflict += (p.group() == STATIC_STRUCT || p.group() == VEGETATION) &&
                  p.semantic_probability() >= c.semantic_min_confidence();
      if (!semantic->source_available()) {
        ++unavailable;
      } else if (p.semantic_probability() < c.semantic_min_confidence()) {
        ++low_confidence;
      } else {
        ++group_support[p.group()];
        if (p.group() == THING && p.unified_class() != CLASS_UNKNOWN) {
          class_support[p.unified_class()].push_back(p.point_id());
        }
      }
    }
    semantic->set_n_unavailable(unavailable);
    semantic->set_n_low_confidence(low_confidence);
    for (auto& entry : class_support) {
      auto* support = semantic->add_thing_support();
      support->set_unified_class(entry.first);
      std::sort(entry.second.begin(), entry.second.end());
      for (uint32_t id : entry.second) {
        support->add_point_id(id);
      }
    }
    for (const auto& entry : group_support) {
      auto* support = semantic->add_group_support();
      support->set_group(entry.first);
      support->set_n_points(entry.second);
    }
    if (!core.empty()) {
      stats->set_ratio_same_class(static_cast<double>(same) / core.size());
      stats->set_ratio_static_vegetation(static_cast<double>(conflict) /
                                         core.size());
    }
    for (int id : margin) {
      if (core_components.count(components[id]) != 0) {
        core.push_back(id);
        stats->set_n_margin(stats->n_margin() + 1);
      } else {
        stats->set_n_unconnected_margin(stats->n_unconnected_margin() + 1);
      }
    }
    const bool enough = stats->n_core() >= c.core_min_points();
    hypothesis->set_existence_supported(enough);
    const bool identity_available = semantic->identity_qualified();
    bool disagreement = false;
    if (identity_available && enough && hypothesis->group() == THING &&
        hypothesis->unified_class() != CLASS_UNKNOWN) {
      for (const auto& support : semantic->thing_support()) {
        disagreement |=
            support.unified_class() != hypothesis->unified_class() &&
            static_cast<double>(support.point_id_size()) / stats->n_core() >=
                c.conflict_ratio();
      }
    }
    semantic->set_cross_thing_disagreement(disagreement);
    if (c.quality().semantic_identity_unverified()) {
      hypothesis->add_reason(SEMANTIC_IDENTITY_UNVERIFIED);
    }
    if (identity_available && enough &&
        stats->ratio_static_vegetation() >= c.conflict_ratio()) {
      hypothesis->set_consistency(CONFLICT);
    } else if (identity_available && enough &&
               stats->ratio_same_class() >= c.support_ratio()) {
      hypothesis->set_consistency(SUPPORT);
      hypothesis->set_identity_supported(true);
    } else {
      hypothesis->set_consistency(INSUFFICIENT);
    }
    if (hypothesis->consistency() == INSUFFICIENT &&
        det.score() < c.unsupported_score_min()) {
      hypothesis->set_decision(BELOW_INSTANCE_THRESHOLD);
      continue;
    }
    if (c.quality().separate_weak_detections() &&
        hypothesis->consistency() == CONFLICT &&
        det.score() < c.unsupported_score_min()) {
      hypothesis->set_decision(RETAINED_WEAK);
      hypothesis->add_reason(WEAK_CLASS_HYPOTHESIS);
      out->add_risk_event(WEAK_CLASS_HYPOTHESIS);
      continue;
    }
    hypothesis->set_decision(INSTANCE_ACCEPTED);
    const std::set<int> support(core.begin(), core.end());
    int object_index = -1;
    for (int i = 0; i < out->object_size() && !support.empty(); ++i) {
      if (out->object(i).unified_class() != hypothesis->unified_class() ||
          out->object(i).group() != hypothesis->group() ||
          BevIou(det, instance_boxes[i]) < c.dedup_bev_iou()) {
        continue;
      }
      std::size_t shared = 0;
      for (int id : support) {
        shared += instance_support[i].count(id);
      }
      if (static_cast<double>(shared) /
              std::max(support.size(), instance_support[i].size()) >=
          c.dedup_point_overlap()) {
        object_index = i;
        break;
      }
    }
    const bool new_instance = object_index < 0;
    if (new_instance) {
      object_index = out->object_size();
      out->add_object()->set_id(object_index + 1);
      instance_support.push_back(support);
      instance_boxes.push_back(det);
    } else {
      instance_support[object_index].insert(support.begin(), support.end());
    }
    auto* object = out->mutable_object(object_index);
    hypothesis->set_object_id(object->id());
    if (new_instance || det.score() > object->raw_score()) {
      *object->mutable_box() = hypothesis->box();
      object->set_source(DET);
      object->set_raw_score(det.score());
      object->set_unified_class(hypothesis->unified_class());
      object->set_group(hypothesis->group());
      object->set_consistency(hypothesis->consistency());
      object->set_orientation_reliable(true);
      object->clear_velocity();
      if (det.has_velocity_x() && det.has_velocity_y()) {
        SetVector(
            rotation * Eigen::Vector3d(det.velocity_x(), det.velocity_y(), 0),
            object->mutable_velocity());
      }
      instance_boxes[object_index] = det;
    }
    if (new_instance) {
      object->add_reason(MODEL_HYPOTHESIS);
    }
    if (object->class_conflict() || hypothesis->consistency() == CONFLICT) {
      object->set_class_conflict(true);
      object->set_consistency(CONFLICT);
      object->add_reason(DET_CLASS_CONFLICT);
      out->add_risk_event(DET_CLASS_CONFLICT);
    } else if (hypothesis->consistency() == INSUFFICIENT) {
      object->add_reason(SPARSE_DETECTION);
    }
    object->set_classification_use(object->class_conflict()
                                       ? IDENTITY_UNCERTAIN
                                       : CLASSIFICATION_CANDIDATE);
    for (int id : core) {
      if (c.quality().compatible_margin() && identity_available &&
          !InBox(positions[id], box, 0) &&
          !SemanticCompatible(out->point_evidence(id),
                              hypothesis->unified_class(), c)) {
        continue;
      }
      auto* decision = out->mutable_point_decision(id);
      if (owner[id] >= 0) {
        decision->set_competition(true);
        if (decision->reason().empty() ||
            decision->reason(decision->reason_size() - 1) !=
                ASSOCIATION_COMPETITION) {
          decision->add_reason(ASSOCIATION_COMPETITION);
        }
      }
      const double score = (InBox(positions[id], box, 0) ? 4.0 : 0.0) +
                           (SemanticCompatible(out->point_evidence(id),
                                               hypothesis->unified_class(), c)
                                ? 2.0
                                : 0.0) +
                           det.score() -
                           0.01 * (positions[id] - box.center).norm();
      if (score > best[id]) {
        best[id] = score;
        owner[id] = object_index;
      }
    }
  }
  for (std::size_t id = 0; id < owner.size(); ++id) {
    if (owner[id] < 0) {
      continue;
    }
    auto* object = out->mutable_object(owner[id]);
    object->add_point_id(id);
    auto* decision = out->mutable_point_decision(id);
    decision->set_object_id(object->id());
    decision->set_source(DET);
    decision->add_reason(MODEL_HYPOTHESIS);
    decision->set_explanation(object->class_conflict() ? EXPL_OBJECT_WEAK
                                                       : EXPL_OBJECT);
  }
  return boxes;
}

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
