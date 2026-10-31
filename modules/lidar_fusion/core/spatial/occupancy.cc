// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/spatial/occupancy.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <vector>

#include "Eigen/Geometry"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {
namespace {
void Rasterize(const OrientedBox& box, const FusionConfig& c,
               uint32_t object_id, SpaceCells* cells) {
  const auto range = BoxRange(box, c, 0);
  const int width =
      static_cast<int>(std::ceil((c.max_x() - c.min_x()) / c.resolution()));
  const int height =
      static_cast<int>(std::ceil((c.max_y() - c.min_y()) / c.resolution()));
  for (int x = range.first.first; x <= range.second.first; ++x) {
    for (int y = range.first.second; y <= range.second.second; ++y) {
      const double px = (x + 0.5) * c.resolution();
      const double py = (y + 0.5) * c.resolution();
      const int ix =
          static_cast<int>(std::floor((px - c.min_x()) / c.resolution()));
      const int iy =
          static_cast<int>(std::floor((py - c.min_y()) / c.resolution()));
      if (ix < 0 || iy < 0 || ix >= width || iy >= height) {
        continue;
      }
      auto& cell = (*cells)[{ix, iy}];
      cell.set_occupancy_det(true);
      if (object_id > 0) {
        cell.add_object_ref(object_id);
      }
    }
  }
}
}  // namespace

SpaceCells BuildSpace(const FusionConfig& c, const FusionOutput& frame,
                      const std::vector<OrientedBox>& detection_boxes,
                      SpaceMap* map) {
  map->set_resolution(c.resolution());
  map->set_origin_x(c.min_x());
  map->set_origin_y(c.min_y());
  map->set_width(std::ceil((c.max_x() - c.min_x()) / c.resolution()));
  map->set_height(std::ceil((c.max_y() - c.min_y()) / c.resolution()));
  SpaceCells cells(*map);
  std::vector<std::array<int, 8>> semantics(
      static_cast<std::size_t>(map->width()) * map->height());
  for (int i = 0; i < frame.point_evidence_size(); ++i) {
    const auto& p = frame.point_evidence(i);
    if (!p.finite() || !InRoi(Vector(p.position()), c)) {
      continue;
    }
    const CellKey key = Key(p.position().x() - c.min_x(),
                            p.position().y() - c.min_y(), c.resolution());
    auto& cell = cells[key];
    cell.set_endpoint_observed(true);
    ++semantics[key.first * map->height() + key.second][p.group()];
    if (p.is_ground_geo()) {
      cell.set_ground_supported(true);
      cell.set_ground_z(p.z_ground());
      cell.set_ground_slope(p.ground_slope());
      cell.set_ground_residual(p.ground_residual());
    }
    if (frame.point_decision(i).explanation() != EXPL_NOISE &&
        !p.is_ground_geo() && ObstacleHeight(p, c)) {
      cell.set_occupancy_obs(true);
    }
    if (frame.point_decision(i).has_object_id()) {
      cell.add_object_ref(frame.point_decision(i).object_id());
    }
  }
  for (std::size_t i = 0; i < detection_boxes.size(); ++i) {
    // Even rejected low-score instances retain their independent inference.
    Rasterize(detection_boxes[i], c, frame.hypothesis(i).object_id(), &cells);
  }
  for (const auto& object : frame.object()) {
    if (object.source() == SEG_RECALL || object.source() == TRACK_PRIOR ||
        (object.source() == DET &&
         std::find(object.reason().begin(), object.reason().end(),
                   ASSOCIATED_SUPPORT_EXTENT) != object.reason().end())) {
      Rasterize({Vector(object.box().center()), Vector(object.box().size()),
                 Eigen::Quaterniond(object.box().qw(), object.box().qx(),
                                    object.box().qy(), object.box().qz())
                     .toRotationMatrix()},
                c, object.id(), &cells);
    }
  }
  for (auto& entry : cells) {
    auto& cell = entry.second;
    cell.set_x_index(entry.first.first);
    cell.set_y_index(entry.first.second);
    cell.set_traversability(
        cell.occupancy_obs() || cell.occupancy_det() ? OCCUPIED : UNKNOWN);
    const auto& votes =
        semantics[entry.first.first * map->height() + entry.first.second];
    const auto dominant = std::max_element(votes.begin(), votes.end());
    if (*dominant > 0) {
      cell.set_semantic(static_cast<Group>(dominant - votes.begin()));
    }
    std::set<uint32_t> refs(cell.object_ref().begin(), cell.object_ref().end());
    cell.clear_object_ref();
    for (uint32_t ref : refs) {
      cell.add_object_ref(ref);
    }
  }
  return cells;
}

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
