// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/association/prior_association.h"

#include <algorithm>
#include <map>
#include <vector>

#include "Eigen/Geometry"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {

void AssociatePrior(const ObjectPrior* prior, const FusionConfig& c,
                    FusionOutput* out) {
  if (prior == nullptr) {
    return;
  }
  for (const auto& prediction : *prior) {
    const auto& b = prediction.box();
    const Eigen::Quaterniond rotation(b.qw(), b.qx(), b.qy(), b.qz());
    std::vector<int> points;
    std::map<uint32_t, uint32_t> observed_support;
    for (int i = 0; i < out->point_evidence_size(); ++i) {
      const auto& p = out->point_evidence(i);
      const auto& d = out->point_decision(i);
      if (!p.finite() || !ObstacleHeight(p, c) ||
          d.explanation() == EXPL_GROUND || d.explanation() == EXPL_NOISE ||
          !InRoi(Vector(p.position()), c)) {
        continue;
      }
      if (!c.quality().semantic_identity_unverified() &&
          p.semantic_probability() >= c.semantic_min_confidence() &&
          (p.group() == STATIC_STRUCT || p.group() == VEGETATION ||
           p.group() == SURFACE || p.group() == SURFACE_SOFT)) {
        continue;
      }
      const Eigen::Vector3d local =
          rotation.conjugate() * (Vector(p.position()) - Vector(b.center()));
      if ((local.cwiseAbs().array() <= (Vector(b.size()) * 0.5).array())
              .all()) {
        if (d.has_object_id()) {
          ++observed_support[d.object_id()];
        } else {
          points.push_back(i);
        }
      }
    }
    // A current candidate already supporting this prediction takes precedence.
    // Recovering only its unowned outer returns would create a duplicate track.
    if (std::any_of(observed_support.begin(), observed_support.end(),
                    [&](const auto& entry) {
                      return entry.second >= c.core_min_points();
                    })) {
      continue;
    }
    if (points.size() < c.core_min_points()) {
      continue;
    }
    auto* object = out->add_object();
    *object = prediction;
    object->set_id(out->object_size());
    object->set_source(TRACK_PRIOR);
    object->clear_point_id();
    object->clear_raw_score();
    object->set_consistency(INSUFFICIENT);
    object->add_reason(TRACK_RECOVERY);
    if (object->class_conflict()) {
      out->add_risk_event(DET_CLASS_CONFLICT);
    }
    for (int id : points) {
      object->add_point_id(id);
      auto* d = out->mutable_point_decision(id);
      d->set_object_id(object->id());
      d->set_source(TRACK_PRIOR);
      d->set_explanation(object->class_conflict() ||
                                 object->classification_use() ==
                                     IDENTITY_UNCERTAIN
                             ? EXPL_OBJECT_WEAK
                             : EXPL_OBJECT);
      d->add_reason(TRACK_RECOVERY);
    }
  }
}

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
