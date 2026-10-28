// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_fusion/core/spatial.h"

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

namespace apollo {
namespace lidar_fusion {
namespace {

using Key = SpaceKey;
using Cells = SpaceCells;

Key Index(const SpaceMap& map, double x, double y) {
  return {
      static_cast<int>(std::floor((x - map.origin_x()) / map.resolution())),
      static_cast<int>(std::floor((y - map.origin_y()) / map.resolution()))};
}

bool Inside(const SpaceMap& map, const Key& key) {
  return key.first >= 0 && key.second >= 0 &&
         key.first < static_cast<int>(map.width()) &&
         key.second < static_cast<int>(map.height());
}

bool Free(const SpaceCell& c) {
  return c.traversability() == FREE || c.traversability() == FREE_SOFT;
}

bool Occupied(const SpaceCell& c) {
  return c.occupancy_obs() || c.occupancy_det() || c.occupancy_unknown();
}

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

void Views(const FusionConfig& config, FusionOutput* out) {
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
  std::set<uint32_t> unknown;
  for (const auto& cluster : out->unknown()) {
    unknown.insert(cluster.point_id().begin(), cluster.point_id().end());
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
    if (unknown.count(p.point_id()) != 0) {
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

// Grid DDA visits actual crossed cells, not a sampled line that can skip
// diagonal occluders. Height evidence remains separate from ground support.
void Ray(const FusionConfig& config, const PointEvidence& p,
         const SpaceMap& map, Cells* cells) {
  const auto& o = config.spatial();
  const double ox = config.sensor_to_base().x();
  const double oy = config.sensor_to_base().y();
  const double oz = config.sensor_to_base().z();
  if (!Inside(map, Index(map, ox, oy))) {
    return;
  }
  const double dx = p.position().x() - ox;
  const double dy = p.position().y() - oy;
  const double dz = p.position().z() - oz;
  Key key = Index(map, ox, oy);
  const Key end = Index(map, p.position().x(), p.position().y());
  const int sx = dx > 0 ? 1 : -1;
  const int sy = dy > 0 ? 1 : -1;
  const double infinity = std::numeric_limits<double>::infinity();
  const double tx = dx != 0 ? map.resolution() / std::abs(dx) : infinity;
  const double ty = dy != 0 ? map.resolution() / std::abs(dy) : infinity;
  double nx =
      dx != 0
          ? (map.origin_x() + (key.first + (sx > 0)) * map.resolution() - ox) /
                dx
          : infinity;
  double ny =
      dy != 0
          ? (map.origin_y() + (key.second + (sy > 0)) * map.resolution() - oy) /
                dy
          : infinity;
  double previous = 0;
  for (uint32_t count = 0; count <= map.width() + map.height(); ++count) {
    if (!Inside(map, key) || key == end) {
      break;
    }
    const double next = std::min({nx, ny, 1.0});
    auto& cell = (*cells)[key];
    const double za = oz + dz * previous - cell.ground_z();
    const double zb = oz + dz * next - cell.ground_z();
    // A band is observed only when this segment stays in it. Never claim an
    // entire height column clear because a single ray passed somewhere above.
    const double width = (o.band_max_z() - o.band_min_z()) / o.height_bands();
    const int a = static_cast<int>(std::floor((za - o.band_min_z()) / width));
    const int b = static_cast<int>(std::floor((zb - o.band_min_z()) / width));
    if (a == b && a >= 0 && a < static_cast<int>(o.height_bands()) &&
        next > previous) {
      cell.set_visibility_mask(cell.visibility_mask() | (1U << a));
      cell.set_visibility_approximate(!o.verified_single_origin());
    }
    // Any independently observed/inferred obstruction blocks this conservative
    // 2D reference ray, even when the beam might pass above the obstruction.
    if (Occupied(cell)) {
      break;
    }
    if (next >= 1) {
      break;
    }
    if (nx == next) {
      key.first += sx;
      nx += tx;
    }
    if (ny == next) {
      key.second += sy;
      ny += ty;
    }
    previous = next;
  }
}

void Traversable(const FusionConfig& config, const SpaceMap& map,
                 Cells* cells) {
  const auto& o = config.spatial();
  const uint32_t required = (1U << o.height_bands()) - 1;
  const double radius =
      std::hypot(o.vehicle_width(), o.vehicle_length()) * 0.5 +
      o.safety_margin() + map.resolution() * std::sqrt(0.5);
  const int offset = static_cast<int>(std::ceil(radius / map.resolution()));
  std::set<Key> local_free;
  for (const auto& entry : *cells) {
    const auto& c = entry.second;
    if (!Occupied(c) && c.ground_supported() &&
        c.ground_slope() <= o.max_slope() &&
        c.ground_residual() <= o.max_roughness() &&
        (c.visibility_mask() & required) == required &&
        !c.visibility_approximate()) {
      local_free.insert(entry.first);
    }
  }
  for (auto& entry : *cells) {
    auto& c = entry.second;
    c.set_reachable(false);
    if (Occupied(c)) {
      c.set_traversability(OCCUPIED);
      c.set_physically_traversable(false);
      continue;
    }
    bool supported = local_free.count(entry.first) != 0;
    for (int dx = -offset; dx <= offset && supported; ++dx) {
      for (int dy = -offset; dy <= offset && supported; ++dy) {
        if (std::hypot(dx, dy) * map.resolution() > radius) {
          continue;
        }
        const Key key{entry.first.first + dx, entry.first.second + dy};
        const auto neighbor = cells->find(key);
        supported = local_free.count(key) != 0 && neighbor != cells->end() &&
                    std::abs(neighbor->second.ground_z() - c.ground_z()) <=
                        o.max_step();
      }
    }
    c.set_physically_traversable(supported);
    if (!supported) {
      c.set_traversability(UNKNOWN);
      c.add_reason((c.visibility_mask() & required) != required
                       ? MISSING_VISIBILITY
                       : VEHICLE_CONSTRAINT);
    } else if (c.semantic() == SURFACE) {
      c.set_traversability(FREE);
    } else if (c.semantic() == SURFACE_SOFT) {
      c.set_traversability(FREE_SOFT);
    } else if (c.semantic() == BOUNDARY) {
      c.set_traversability(BOUNDARY_CELL);
    } else {
      c.set_traversability(UNKNOWN);
    }
  }
  if (!o.has_seed_x() || !o.has_seed_y()) {
    return;
  }
  std::queue<Key> queue;
  queue.push(Index(map, o.seed_x(), o.seed_y()));
  while (!queue.empty()) {
    const Key key = queue.front();
    queue.pop();
    auto found = cells->find(key);
    if (found == cells->end() || !Free(found->second) ||
        found->second.reachable()) {
      continue;
    }
    found->second.set_reachable(true);
    for (const Key& d : {Key{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
      const Key next{key.first + d.first, key.second + d.second};
      const auto neighbor = cells->find(next);
      if (neighbor != cells->end() &&
          std::abs(neighbor->second.ground_z() - found->second.ground_z()) <=
              o.max_step()) {
        queue.push(next);
      }
    }
  }
}

}  // namespace

SpaceCells::SpaceCells(const SpaceMap& map) {
  const uint64_t count = uint64_t{map.width()} * map.height();
  if (count > 0 && count <= 1000000) {
    width_ = map.width();
    height_ = map.height();
    slots_.assign(count, -1);
  }
  cells_.reserve(map.cell_size());
  for (const auto& cell : map.cell()) {
    (*this)[{cell.x_index(), cell.y_index()}] = cell;
  }
}

int* SpaceCells::Slot(const SpaceKey& key) {
  if (key.first >= 0 && key.first < width_ && key.second >= 0 &&
      key.second < height_) {
    return &slots_[key.first * height_ + key.second];
  }
  const auto found = overflow_.find(key);
  return found == overflow_.end() ? nullptr : &found->second;
}

SpaceCell& SpaceCells::operator[](const SpaceKey& key) {
  int* slot = Slot(key);
  if (slot != nullptr && *slot >= 0) {
    return cells_[*slot].second;
  }
  const int index = cells_.size();
  if (slot != nullptr) {
    *slot = index;
  } else {
    overflow_.emplace(key, index);
  }
  cells_.emplace_back(key, SpaceCell{});
  return cells_.back().second;
}

SpaceCells::Storage::iterator SpaceCells::find(const SpaceKey& key) {
  const int* slot = Slot(key);
  return slot == nullptr || *slot < 0 ? end() : begin() + *slot;
}

void SpaceCells::Publish(SpaceMap* map) {
  std::vector<std::size_t> order(cells_.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return cells_[a].first < cells_[b].first;
  });
  map->clear_cell();
  for (std::size_t id : order) {
    auto& entry = cells_[id];
    entry.second.set_x_index(entry.first.first);
    entry.second.set_y_index(entry.first.second);
    *map->add_cell() = std::move(entry.second);
  }
}

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
    Views(config, out);
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
        Ray(config, p, *map, &cells);
      }
    }
  }
  if (config.spatial().traversability()) {
    Traversable(config, *map, &cells);
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
