// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/lidar_fusion/core/spatial/spatial.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Geometry"

#include "modules/lidar_fusion/core/spatial/grid_queries.h"
#include "modules/lidar_fusion/core/spatial/spatial_stages.h"

namespace apollo {
namespace lidar_fusion {
namespace {

using Key = SpaceKey;
using Cells = SpaceCells;
using spatial_internal::Free;
using spatial_internal::Index;
using spatial_internal::Inside;
using spatial_internal::Occupied;

template <typename Visitor>
void UnknownCells(const UnknownCandidate& unknown, const FusionOutput& frame,
                  Visitor visitor) {
  const auto& map = frame.space_map();
  if (unknown.point_supported_footprint()) {
    std::set<Key> keys;
    for (uint32_t id : unknown.point_id()) {
      if (id >= static_cast<uint32_t>(frame.point_evidence_size())) {
        continue;
      }
      const auto& p = frame.point_evidence(id);
      if (!p.finite()) {
        continue;
      }
      const Key key = Index(map, p.position().x(), p.position().y());
      if (Inside(map, key)) {
        keys.insert(key);
      }
    }
    for (const auto& key : keys) {
      visitor(key);
    }
    return;
  }
  if (unknown.polygon().empty()) {
    return;
  }
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = min_x;
  double max_x = -min_x;
  double max_y = -min_x;
  for (const auto& p : unknown.polygon()) {
    if (!std::isfinite(p.x()) || !std::isfinite(p.y())) {
      return;
    }
    min_x = std::min(min_x, p.x());
    min_y = std::min(min_y, p.y());
    max_x = std::max(max_x, p.x());
    max_y = std::max(max_y, p.y());
  }
  const Key low = Index(map, min_x, min_y);
  const Key high = Index(map, max_x, max_y);
  for (int x = std::max(0, low.first);
       x <= std::min(static_cast<int>(map.width()) - 1, high.first); ++x) {
    for (int y = std::max(0, low.second);
         y <= std::min(static_cast<int>(map.height()) - 1, high.second); ++y) {
      visitor(Key{x, y});
    }
  }
}

}  // namespace

bool ValidateSpatial(const FusionConfig& config, std::string* error) {
  const auto& o = config.spatial();
  const bool valid_ray = std::isfinite(o.band_min_z()) &&
                         std::isfinite(o.band_max_z()) &&
                         o.band_max_z() > o.band_min_z() &&
                         o.height_bands() > 0 && o.height_bands() <= 16;
  bool vehicle = o.has_vehicle_length() && o.has_vehicle_width() &&
                 o.has_vehicle_height() && o.has_safety_margin() &&
                 o.has_max_slope() && o.has_max_step() &&
                 o.has_max_roughness() && o.has_ground_clearance();
  for (double v : {o.vehicle_length(), o.vehicle_width(), o.vehicle_height(),
                   o.safety_margin(), o.max_slope(), o.max_step(),
                   o.max_roughness(), o.ground_clearance()}) {
    vehicle &= std::isfinite(v) && v >= 0;
  }
  vehicle &= o.vehicle_length() > 0 && o.vehicle_width() > 0 &&
             o.vehicle_height() > 0 && o.vehicle_length() <= 20 &&
             o.vehicle_width() <= 10 &&
             o.vehicle_height() <= config.clearance_height() &&
             o.ground_clearance() > 0 &&
             config.obstacle_min_height() <= o.ground_clearance() &&
             o.safety_margin() <= 5 &&
             o.band_min_z() <= config.obstacle_min_height() &&
             o.band_max_z() >= o.vehicle_height();
  const double radius =
      std::hypot(o.vehicle_length(), o.vehicle_width()) * 0.5 +
      o.safety_margin();
  vehicle &= radius / config.resolution() <= 50;
  if (!valid_ray || o.has_seed_x() != o.has_seed_y() ||
      (o.has_seed_x() &&
       (!std::isfinite(o.seed_x()) || !std::isfinite(o.seed_y()))) ||
      (o.traversability() &&
       (!vehicle || !o.trace_rays() || !o.verified_single_origin()))) {
    if (error != nullptr) {
      *error =
          "invalid spatial options or missing verified visibility/vehicle "
          "contract";
    }
    return false;
  }
  return true;
}

void EnrichSpatial(const FusionConfig& config, FusionOutput* out) {
  SpaceCells cells(out->space_map());
  EnrichSpatial(config, out, &cells);
  cells.Publish(out->mutable_space_map());
}

void EnrichSpatial(const FusionConfig& config, FusionOutput* out,
                   SpaceCells* workspace) {
  if (config.spatial().point_views()) {
    spatial_internal::BuildPointViews(config, out);
  }
  auto* map = out->mutable_space_map();
  auto& cells = *workspace;
  for (auto& unknown : *out->mutable_unknown()) {
    std::set<uint32_t> related(unknown.related_object_id().begin(),
                               unknown.related_object_id().end());
    UnknownCells(unknown, *out, [&](const Key& key) {
      auto& cell = cells[key];
      cell.set_occupancy_unknown(true);
      cell.set_traversability(OCCUPIED);
      related.insert(cell.object_ref().begin(), cell.object_ref().end());
    });
    unknown.clear_related_object_id();
    for (uint32_t id : related) {
      unknown.add_related_object_id(id);
    }
  }
  if (config.spatial().trace_rays()) {
    for (const auto& p : out->point_evidence()) {
      if (p.finite()) {
        spatial_internal::TraceRay(config, p, *map, &cells);
      }
    }
  }
  if (config.spatial().traversability()) {
    spatial_internal::ClassifyVisibleTraversability(config, *map, &cells);
  }
}

void CheckOutput(const FusionConfig& config, FusionOutput* out) {
  SpaceCells cells(out->space_map());
  CheckOutput(config, out, &cells);
  cells.Publish(out->mutable_space_map());
}

void CheckOutput(const FusionConfig& config, FusionOutput* out,
                 SpaceCells* workspace) {
  auto* map = out->mutable_space_map();
  auto& cells = *workspace;
  std::set<uint32_t> objects;
  uint32_t repairs = 0;
  for (auto& object : *out->mutable_object()) {
    objects.insert(object.id());
    if (object.consistency() == CONFLICT && !object.class_conflict()) {
      object.set_class_conflict(true);
      object.add_reason(OUTPUT_REPAIRED);
      ++repairs;
    }
  }
  for (auto& entry : cells) {
    auto& c = entry.second;
    std::set<uint32_t> refs;
    for (uint32_t ref : c.object_ref()) {
      if (objects.count(ref) != 0) {
        refs.insert(ref);
      } else {
        c.add_reason(OUTPUT_REPAIRED);
        ++repairs;
      }
    }
    c.clear_object_ref();
    for (uint32_t ref : refs) {
      c.add_object_ref(ref);
    }
    if (!refs.empty() && !c.occupancy_obs() && !c.occupancy_det()) {
      c.set_occupancy_det(true);
      c.add_reason(OUTPUT_REPAIRED);
      ++repairs;
    }
  }
  auto occupied = [&](double x, double y, uint32_t id) {
    const Key key = Index(*map, x, y);
    if (!Inside(*map, key)) {
      return;
    }
    auto& c = cells[key];
    if (id > 0 ? !Occupied(c) : !c.occupancy_unknown()) {
      if (id > 0) {
        c.set_occupancy_det(true);
      } else {
        c.set_occupancy_unknown(true);
      }
      c.add_reason(OUTPUT_REPAIRED);
      ++repairs;
    }
    if (id > 0 && std::find(c.object_ref().begin(), c.object_ref().end(), id) ==
                      c.object_ref().end()) {
      c.add_object_ref(id);
      c.add_reason(OUTPUT_REPAIRED);
      ++repairs;
    }
  };
  for (const auto& object : out->object()) {
    const auto& b = object.box();
    Eigen::Quaterniond q(b.qw(), b.qx(), b.qy(), b.qz());
    if (!q.coeffs().allFinite() || q.norm() < 1e-6) {
      continue;
    }
    const Eigen::Vector3d ext =
        q.normalized().toRotationMatrix().cwiseAbs() *
        Eigen::Vector3d(b.size().x(), b.size().y(), b.size().z()) * 0.5;
    const Key low =
        Index(*map, b.center().x() - ext.x(), b.center().y() - ext.y());
    const Key high =
        Index(*map, b.center().x() + ext.x(), b.center().y() + ext.y());
    for (int x = std::max(0, low.first);
         x <= std::min(static_cast<int>(map->width()) - 1, high.first); ++x) {
      for (int y = std::max(0, low.second);
           y <= std::min(static_cast<int>(map->height()) - 1, high.second);
           ++y) {
        occupied(map->origin_x() + (x + 0.5) * map->resolution(),
                 map->origin_y() + (y + 0.5) * map->resolution(), object.id());
      }
    }
  }
  for (auto& unknown : *out->mutable_unknown()) {
    std::set<uint32_t> related;
    std::map<uint32_t, RelationKind> relations;
    for (const auto& relation : unknown.relation()) {
      if (objects.count(relation.object_id()) != 0 &&
          relations.emplace(relation.object_id(), relation.kind()).second) {
        related.insert(relation.object_id());
      } else {
        unknown.add_reason(OUTPUT_REPAIRED);
        ++repairs;
      }
    }
    for (uint32_t ref : unknown.related_object_id()) {
      if (objects.count(ref) != 0) {
        related.insert(ref);
      } else {
        unknown.add_reason(OUTPUT_REPAIRED);
        ++repairs;
      }
    }
    unknown.clear_related_object_id();
    for (uint32_t ref : related) {
      unknown.add_related_object_id(ref);
    }
    for (uint32_t id : unknown.point_id()) {
      if (id < static_cast<uint32_t>(out->point_evidence_size())) {
        const auto& p = out->point_evidence(id);
        if (p.finite()) {
          occupied(p.position().x(), p.position().y(), 0);
        }
      }
    }
    UnknownCells(unknown, *out, [&](const Key& key) {
      occupied(map->origin_x() + (key.first + 0.5) * map->resolution(),
               map->origin_y() + (key.second + 0.5) * map->resolution(), 0);
      for (uint32_t ref : cells[key].object_ref()) {
        if (related.insert(ref).second) {
          unknown.add_reason(OUTPUT_REPAIRED);
          ++repairs;
        }
      }
    });
    unknown.clear_related_object_id();
    unknown.clear_relation();
    for (uint32_t ref : related) {
      unknown.add_related_object_id(ref);
      auto* relation = unknown.add_relation();
      relation->set_object_id(ref);
      auto existing = relations.find(ref);
      relation->set_kind(existing == relations.end() ? RELATION_AMBIGUOUS
                                                     : existing->second);
    }
  }
  for (auto& entry : cells) {
    auto& c = entry.second;
    if (Occupied(c)) {
      if (c.traversability() != OCCUPIED) {
        c.set_traversability(OCCUPIED);
        c.set_physically_traversable(false);
        c.set_reachable(false);
        c.add_reason(OUTPUT_REPAIRED);
        ++repairs;
      }
    } else if (Free(c) &&
               (!config.spatial().traversability() ||
                !c.physically_traversable() || !c.ground_supported() ||
                !config.spatial().verified_single_origin() ||
                c.visibility_mask() !=
                    ((1U << config.spatial().height_bands()) - 1) ||
                c.visibility_approximate())) {
      c.set_traversability(UNKNOWN);
      c.set_reachable(false);
      c.add_reason(OUTPUT_REPAIRED);
      ++repairs;
    }
  }
  out->mutable_health()->set_consistency_repairs(
      out->health().consistency_repairs() + repairs);
  if (repairs > 0) {
    out->add_risk_event(OUTPUT_REPAIRED);
    // A repaired obstruction can invalidate both inflated clearance and the
    // connectivity path. Fail closed rather than retain stale FREE/reachable.
    for (auto& entry : cells) {
      auto& cell = entry.second;
      cell.set_reachable(false);
      if (Free(cell)) {
        cell.set_traversability(UNKNOWN);
        cell.set_physically_traversable(false);
        cell.add_reason(OUTPUT_REPAIRED);
      }
    }
  }
}

}  // namespace lidar_fusion
}  // namespace apollo
