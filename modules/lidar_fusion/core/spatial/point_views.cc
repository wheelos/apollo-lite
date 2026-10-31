// Copyright 2026 WheelOS. All Rights Reserved.

#include <map>
#include <set>
#include <vector>

#include "modules/lidar_fusion/core/spatial/spatial_stages.h"

namespace apollo {
namespace lidar_fusion {
namespace spatial_internal {

void BuildPointViews(const FusionConfig& config, FusionOutput* out) {
  out->clear_point_view();
  for (int kind = GROUND_POINTS; kind <= CORRIDOR_RISK_POINTS; ++kind) {
    out->add_point_view()->set_kind(static_cast<ViewKind>(kind));
  }
  if (config.quality().trusted_point_view()) {
    out->add_point_view()->set_kind(TRUSTED_OBJECT_POINTS);
  }
  std::map<uint32_t, const ObjectCandidate*> objects;
  for (const auto& object : out->object()) {
    objects[object.id()] = &object;
  }
  std::vector<bool> unknown(out->point_evidence_size(), false);
  std::set<uint32_t> sparse_unknown;
  for (const auto& cluster : out->unknown()) {
    for (uint32_t id : cluster.point_id()) {
      if (id < unknown.size()) {
        unknown[id] = true;
      } else {
        sparse_unknown.insert(id);
      }
    }
  }
  for (int i = 0; i < out->point_evidence_size(); ++i) {
    const auto& p = out->point_evidence(i);
    if (!p.finite()) {
      continue;
    }
    const auto& d = out->point_decision(i);
    if (p.is_ground_geo()) {
      out->mutable_point_view(GROUND_POINTS)->add_point_id(p.point_id());
    } else if (d.explanation() != EXPL_NOISE) {
      out->mutable_point_view(NON_GROUND_POINTS)->add_point_id(p.point_id());
    }
    if (d.has_object_id()) {
      out->mutable_point_view(OBJECT_POINTS)->add_point_id(p.point_id());
      const auto object = objects.find(d.object_id());
      if (config.quality().trusted_point_view() &&
          !config.quality().semantic_identity_unverified() &&
          out->health().seg_valid() && object != objects.end() &&
          !object->second->class_conflict() && p.group() == THING &&
          p.unified_class() == object->second->unified_class() &&
          p.semantic_probability() >= config.semantic_min_confidence()) {
        out->mutable_point_view(TRUSTED_OBJECT_POINTS)
            ->add_point_id(p.point_id());
      }
    }
    if (p.group() == STATIC_STRUCT || p.group() == VEGETATION) {
      out->mutable_point_view(STRUCTURE_POINTS)->add_point_id(p.point_id());
    }
    if (p.point_id() < unknown.size()
            ? unknown[p.point_id()]
            : sparse_unknown.count(p.point_id()) != 0) {
      out->mutable_point_view(UNKNOWN_POINTS)->add_point_id(p.point_id());
    }
    if (p.in_corridor() && !p.is_ground_geo() &&
        d.explanation() != EXPL_NOISE &&
        (!p.ground_valid() ||
         p.height_above_ground() <= config.clearance_height())) {
      out->mutable_point_view(CORRIDOR_RISK_POINTS)->add_point_id(p.point_id());
    }
  }
}

}  // namespace spatial_internal
}  // namespace lidar_fusion
}  // namespace apollo
