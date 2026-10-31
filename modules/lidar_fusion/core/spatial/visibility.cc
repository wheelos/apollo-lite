// Copyright 2026 WheelOS. All Rights Reserved.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <set>

#include "modules/lidar_fusion/core/spatial/grid_queries.h"
#include "modules/lidar_fusion/core/spatial/spatial_stages.h"

namespace apollo {
namespace lidar_fusion {
namespace spatial_internal {

// Grid DDA visits actual crossed cells, not a sampled line that can skip
// diagonal occluders. Height evidence remains separate from ground support.
void TraceRay(const FusionConfig& config, const PointEvidence& p,
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

void ClassifyVisibleTraversability(const FusionConfig& config,
                                   const SpaceMap& map, Cells* cells) {
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

}  // namespace spatial_internal
}  // namespace lidar_fusion
}  // namespace apollo
