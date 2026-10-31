// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/evidence/build_evidence.h"

#include <cmath>
#include <map>
#include <vector>

#include "modules/lidar_fusion/core/alignment/input_validation.h"
#include "modules/lidar_fusion/core/evidence/point_geometry.h"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {
using alignment_internal::Mappings;

void BuildEvidence(const drivers::PointCloud& cloud,
                   const SemanticFrame* segmentation, const FusionConfig& c,
                   std::vector<Eigen::Vector3d>* positions, SpatialIndex* index,
                   FusionOutput* out, const GroundPrior* prior) {
  const auto& t = c.sensor_to_base();
  const auto rotation = Rotation(t.roll(), t.pitch(), t.yaw());
  const Eigen::Vector3d translation(t.x(), t.y(), t.z());
  const auto mappings = Mappings(c.semantic_mapping());
  positions->resize(cloud.point_size());
  for (int i = 0; i < cloud.point_size(); ++i) {
    const auto& raw = cloud.point(i);
    auto* p = out->add_point_evidence();
    p->set_point_id(i);
    const Eigen::Vector3d value(raw.x(), raw.y(), raw.z());
    // Bound conversion to integer spatial indices, including finite outliers.
    p->set_finite(value.allFinite() && value.cwiseAbs().maxCoeff() < 10000);
    auto* decision = out->add_point_decision();
    decision->set_point_id(i);
    decision->set_source(GEOMETRY);
    if (!p->finite()) {
      decision->set_explanation(INVALID_POINT);
      decision->add_reason(INVALID_GEOMETRY);
      continue;
    }
    (*positions)[i] = rotation * value + translation;
    SetVector((*positions)[i], p->mutable_position());
    p->set_in_corridor(InCorridor((*positions)[i], c));
    if (InRoi((*positions)[i], c)) {
      (*index)[Key((*positions)[i].x(), (*positions)[i].y(), c.resolution())]
          .push_back(i);
    }
  }
  if (segmentation != nullptr) {
    for (const auto& label : segmentation->point_label()) {
      auto* point = out->mutable_point_evidence(label.point_index());
      const auto& mapping = mappings.at(label.semantic_label());
      point->set_raw_semantic_label(label.semantic_label());
      point->set_semantic_probability(label.confidence());
      point->set_group(mapping.group());
      point->set_unified_class(mapping.unified_class());
    }
  }
  SpatialIndex seed_tiles;
  SpatialIndex semantic_seeds;
  std::map<CellKey, double> prior_ground;
  if (prior != nullptr) {
    for (const auto& p : *prior) {
      prior_ground[Key(p.x(), p.y(), c.ground_tile_size())] = p.z();
    }
  }
  for (int i = 0; i < cloud.point_size(); ++i) {
    const auto& p = out->point_evidence(i);
    if (!p.finite()) {
      continue;
    }
    const auto key =
        Key((*positions)[i].x(), (*positions)[i].y(), c.ground_tile_size());
    const auto previous = prior_ground.find(key);
    if (previous != prior_ground.end()) {
      out->mutable_point_evidence(i)->set_ground_prior_z(previous->second);
    }
    if (p.finite() && InRoi((*positions)[i], c) &&
        std::abs((*positions)[i].z() - c.expected_ground_z()) <=
            c.ground_seed_band()) {
      seed_tiles[Key((*positions)[i].x(), (*positions)[i].y(),
                     c.ground_tile_size())]
          .push_back(i);
      if ((p.group() == SURFACE || p.group() == SURFACE_SOFT) &&
          p.semantic_probability() >= c.semantic_min_confidence()) {
        semantic_seeds[Key((*positions)[i].x(), (*positions)[i].y(),
                           c.ground_tile_size())]
            .push_back(i);
      }
    }
  }
  auto planes = FitGround(*positions, seed_tiles, c);
  const auto semantic_planes = FitGround(*positions, semantic_seeds, c);
  for (const auto& entry : semantic_planes) {
    // Semantics refine a geometrically accepted surface, never create support
    // on a raised semantic island which failed the independent geometry fit.
    auto geometric = planes.find(entry.first);
    if (geometric == planes.end()) {
      continue;
    }
    bool compatible = true;
    if (c.quality().robust_ground()) {
      for (int id : seed_tiles.at(entry.first)) {
        const auto& p = (*positions)[id];
        const Eigen::Vector3d row(p.x() - geometric->second.origin_x,
                                  p.y() - geometric->second.origin_y, 1);
        if (std::abs(row.dot(entry.second.coefficients -
                             geometric->second.coefficients)) >
            c.ground_max_residual()) {
          compatible = false;
          break;
        }
      }
    }
    if (compatible) {
      geometric->second = entry.second;
    }
  }
  for (int i = 0; i < cloud.point_size(); ++i) {
    auto* p = out->mutable_point_evidence(i);
    auto* d = out->mutable_point_decision(i);
    if (!p->finite()) {
      continue;
    }
    const auto& position = (*positions)[i];
    auto plane =
        planes.find(Key(position.x(), position.y(), c.ground_tile_size()));
    if (plane != planes.end()) {
      const auto& g = plane->second;
      const double ground = g.coefficients.dot(Eigen::Vector3d(
          position.x() - g.origin_x, position.y() - g.origin_y, 1));
      const double height = position.z() - ground;
      p->set_ground_valid(true);
      p->set_z_ground(ground);
      p->set_height_above_ground(height);
      p->set_is_ground_geo(std::abs(height) <= c.ground_tolerance());
      p->set_ground_slope(g.coefficients.head<2>().norm());
      p->set_ground_residual(g.residual);
      p->set_ground_seed_count(g.count);
      if (p->has_ground_prior_z()) {
        p->set_ground_prior_consistent(std::abs(p->ground_prior_z() - ground) <=
                                       c.ground_tolerance());
      }
    } else {
      d->add_reason(GROUND_UNSUPPORTED);
    }
    if (p->is_ground_geo()) {
      d->set_explanation(EXPL_GROUND);
      d->add_reason(GEO_GROUND);
    } else if (p->group() == NOISE &&
               p->semantic_probability() >=
                   (p->in_corridor() ? c.noise_corridor_confidence()
                                     : c.noise_outside_confidence())) {
      d->set_explanation(EXPL_NOISE);
      d->add_reason(CONFIDENT_NOISE);
    } else if (p->group() == STATIC_STRUCT && !p->in_corridor() &&
               p->semantic_probability() >= c.semantic_min_confidence()) {
      d->set_explanation(EXPL_STATIC);
      d->add_reason(SEMANTIC_STRUCTURE);
    } else if (p->group() == VEGETATION && p->ground_valid() &&
               p->height_above_ground() > c.clearance_height()) {
      d->set_explanation(EXPL_VEG_HIGH);
      d->add_reason(OUTSIDE_CLEARANCE);
    } else if ((p->group() == SURFACE || p->group() == SURFACE_SOFT) &&
               p->ground_valid() &&
               p->height_above_ground() >= c.obstacle_min_height()) {
      d->add_reason(SEM_GEO_CONFLICT);
    }
    if (d->reason().empty()) {
      d->add_reason(UNEXPLAINED_GEOMETRY);
    }
  }
}

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
