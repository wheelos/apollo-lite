// Copyright 2026 WheelOS. All Rights Reserved.

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

#include "Eigen/StdVector"

#include "modules/lidar_fusion/core/association/candidate_geometry.h"
#include "modules/lidar_fusion/core/association/candidates.h"
#include "modules/lidar_fusion/core/association/relation_support.h"

namespace apollo {
namespace lidar_fusion {
namespace candidate_detail {

PointBounds ActualBounds(
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    const google::protobuf::RepeatedField<uint32_t>& ids) {
  PointBounds bounds;
  for (uint32_t id : ids) {
    const auto& p = evidence.Get(id).position();
    const Eigen::Vector3d point(p.x(), p.y(), p.z());
    if (!point.allFinite()) {
      return PointBounds{};
    }
    bounds.low = bounds.low.cwiseMin(point);
    bounds.high = bounds.high.cwiseMax(point);
    bounds.valid = true;
  }
  return bounds;
}

std::vector<PointBlock> PointBlocks(
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    const UnknownCandidate& unknown) {
  std::vector<PointBlock> blocks;
  for (int begin = 0; begin < unknown.point_id_size(); begin += 128) {
    PointBlock block;
    block.begin = begin;
    block.end = std::min(begin + 128, unknown.point_id_size());
    bool finite = true;
    for (int i = block.begin; i < block.end; ++i) {
      const auto& p = evidence.Get(unknown.point_id(i)).position();
      const Eigen::Vector3d point(p.x(), p.y(), p.z());
      finite &= point.allFinite();
      block.bounds.low = block.bounds.low.cwiseMin(point);
      block.bounds.high = block.bounds.high.cwiseMax(point);
    }
    block.bounds.valid = finite;
    blocks.push_back(block);
  }
  return blocks;
}

ObjectSupport::ObjectSupport(const ObjectCandidate& object)
    : rotation(Eigen::Quaterniond(object.box().qw(), object.box().qx(),
                                  object.box().qy(), object.box().qz())
                   .conjugate()),
      ids(object.point_id().begin(), object.point_id().end()) {
  const auto& box = object.box();
  const Eigen::Vector3d center(box.center().x(), box.center().y(),
                               box.center().z());
  const Eigen::Vector3d half(box.size().x() * 0.5, box.size().y() * 0.5,
                             box.size().z() * 0.5);
  const double epsilon = std::numeric_limits<double>::epsilon();
  if (!center.allFinite() || !half.allFinite() || (half.array() < 0).any() ||
      !std::isfinite(rotation.squaredNorm()) ||
      std::abs(rotation.squaredNorm() - 1) > 32 * epsilon) {
    return;
  }
  const Eigen::Vector3d extent =
      rotation.conjugate().toRotationMatrix().cwiseAbs() * half;
  bounds.low = center - extent;
  bounds.high = center + extent;
  bounds.valid = bounds.low.allFinite() && bounds.high.allFinite();
  scale = center.cwiseAbs().maxCoeff() + extent.sum() + half.sum();
}

bool ObjectSupport::Separated(const PointBounds& points,
                              double adjacent) const {
  if (!points.valid || !bounds.valid || !std::isfinite(adjacent) ||
      adjacent < 0) {
    return false;
  }
  const double magnitude =
      1 + scale + adjacent +
      points.low.cwiseAbs().cwiseMax(points.high.cwiseAbs()).maxCoeff();
  if (!std::isfinite(magnitude) ||
      magnitude > std::sqrt(std::numeric_limits<double>::max()) / 32) {
    return false;
  }
  // Cover quaternion norm error and all bound/transform rounding. Uncertain
  // or extreme inputs retain the original exact relation calculation.
  const double guard =
      4096 * std::numeric_limits<double>::epsilon() * magnitude;
  const Eigen::Vector3d gap = (bounds.low - points.high)
                                  .cwiseMax(points.low - bounds.high)
                                  .cwiseMax(0.);
  return gap.norm() > adjacent + guard;
}

}  // namespace candidate_detail

using candidate_detail::ActualBounds;
using candidate_detail::ObjectSupport;
using candidate_detail::PointBlock;
using candidate_detail::PointBlocks;
using candidate_detail::PointBounds;

bool ValidateCandidates(const FusionConfig& c) {
  const auto& o = c.candidates();
  return std::isfinite(o.adjacent_distance()) && o.adjacent_distance() >= 0 &&
         std::isfinite(o.far_distance()) && o.far_distance() > 0 &&
         (!o.recall_confident_foreground() ||
          (!o.separate_semantic_instances() &&
           !o.has_far_connectivity_size())) &&
         (!o.has_far_connectivity_size() ||
          (std::isfinite(o.far_connectivity_size()) &&
           o.far_connectivity_size() >= c.connectivity_size() &&
           o.far_connectivity_size() <= c.connectivity_size() * 4));
}

void RefineCandidates(
    const FusionConfig& c,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    CandidateResult* candidates, CandidateWork* work, const PointIndex* index) {
  if (work != nullptr) {
    *work = CandidateWork{};
  }
  for (auto& object : candidates->objects) {
    candidate_detail::RefineObjectGeometry(c, evidence, &object, index);
  }
  if (candidates->unknowns.empty()) {
    return;
  }
  std::vector<ObjectSupport, Eigen::aligned_allocator<ObjectSupport>> support;
  support.reserve(candidates->objects.size());
  for (const auto& object : candidates->objects) {
    support.emplace_back(object);
  }
  for (auto& unknown : candidates->unknowns) {
    candidate_detail::RefineUnknownGeometry(c, evidence, &unknown);
    unknown.clear_relation();
    std::set<uint32_t> references(unknown.related_object_id().begin(),
                                  unknown.related_object_id().end());
    const PointBounds points = ActualBounds(evidence, unknown.point_id());
    std::vector<PointBlock> blocks;
    for (int i = 0; i < candidates->objects.size(); ++i) {
      const auto& object = candidates->objects.Get(i);
      const auto& cached = support[i];
      if (references.count(object.id()) == 0 &&
          cached.Separated(points, c.candidates().adjacent_distance())) {
        continue;
      }
      const auto& b = object.box();
      if (work != nullptr) {
        ++work->pairs;
      }
      const bool referenced = references.count(object.id()) != 0;
      const bool duplicate =
          !object.class_conflict() && !unknown.point_id().empty() &&
          std::all_of(unknown.point_id().begin(), unknown.point_id().end(),
                      [&](uint32_t id) { return cached.ids.count(id) != 0; });
      bool inside = false;
      double distance = std::numeric_limits<double>::infinity();
      if (!(referenced && (object.class_conflict() || duplicate))) {
        if (blocks.empty()) {
          blocks = PointBlocks(evidence, unknown);
        }
        bool decided = false;
        for (const auto& block : blocks) {
          // A reference already establishes a relation; only inside support
          // can change its kind. Otherwise retain all possible near support.
          if (cached.Separated(
                  block.bounds,
                  referenced ? 0 : c.candidates().adjacent_distance())) {
            continue;
          }
          for (int j = block.begin; j < block.end; ++j) {
            const uint32_t id = unknown.point_id(j);
            if (work != nullptr) {
              ++work->point_transforms;
            }
            const auto& p = evidence.Get(id).position();
            const Eigen::Vector3d local =
                cached.rotation * Eigen::Vector3d(p.x() - b.center().x(),
                                                  p.y() - b.center().y(),
                                                  p.z() - b.center().z());
            const double dx =
                std::max(0., std::abs(local.x()) - b.size().x() * 0.5);
            const double dy =
                std::max(0., std::abs(local.y()) - b.size().y() * 0.5);
            const double dz =
                std::max(0., std::abs(local.z()) - b.size().z() * 0.5);
            distance =
                std::min(distance, std::sqrt(dx * dx + dy * dy + dz * dz));
            inside |= dx == 0 && dy == 0 && dz == 0;
            decided =
                inside || ((object.class_conflict() || duplicate) &&
                           distance <= c.candidates().adjacent_distance());
            if (decided) {
              break;
            }
          }
          if (decided) {
            break;
          }
        }
      }
      if (distance > c.candidates().adjacent_distance() && !referenced) {
        continue;
      }
      auto* relation = unknown.add_relation();
      relation->set_object_id(object.id());
      references.insert(object.id());
      if (object.class_conflict()) {
        relation->set_kind(RELATION_AMBIGUOUS);
      } else if (duplicate) {
        relation->set_kind(RELATION_DUPLICATE);
      } else if (inside) {
        relation->set_kind(RELATION_SUPPLEMENT);
      } else {
        relation->set_kind(RELATION_ADJACENT);
      }
    }
    unknown.clear_related_object_id();
    for (uint32_t id : references) {
      unknown.add_related_object_id(id);
    }
  }
}

}  // namespace lidar_fusion
}  // namespace apollo
