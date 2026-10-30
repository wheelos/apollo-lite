// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_fusion/core/fusion.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "Eigen/QR"
#include "google/protobuf/arena.h"

#include "modules/lidar_fusion/core/candidates.h"
#include "modules/lidar_fusion/core/spatial.h"

namespace apollo {
namespace lidar_fusion {
namespace {

using CellKey = std::pair<int, int>;
using VoxelKey = std::tuple<int, int, int>;

using SpatialIndex = PointIndex;

bool Fail(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

bool Probability(double value) {
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

Eigen::Matrix3d Rotation(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

void SetVector(const Eigen::Vector3d& value, Vec3* output) {
  output->set_x(value.x());
  output->set_y(value.y());
  output->set_z(value.z());
}

Eigen::Vector3d Vector(const Vec3& value) {
  return {value.x(), value.y(), value.z()};
}

struct OrientedBox {
  Eigen::Vector3d center;
  Eigen::Vector3d size;
  Eigen::Matrix3d rotation;
};

using Detection = lidar_object_detection::LidarObjectDetection;

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

void SetBox(const OrientedBox& box, Box* output) {
  SetVector(box.center, output->mutable_center());
  SetVector(box.size, output->mutable_size());
  const Eigen::Quaterniond q(box.rotation);
  output->set_qx(q.x());
  output->set_qy(q.y());
  output->set_qz(q.z());
  output->set_qw(q.w());
}

bool InBox(const Eigen::Vector3d& point, const OrientedBox& box,
           double margin) {
  const Eigen::Vector3d local = box.rotation.transpose() * (point - box.center);
  return (local.array().abs() <= box.size.array() * 0.5 + margin).all();
}

CellKey Key(double x, double y, double size) {
  return {static_cast<int>(std::floor(x / size)),
          static_cast<int>(std::floor(y / size))};
}

bool InRoi(const Eigen::Vector3d& p, const FusionConfig& c) {
  return p.x() >= c.min_x() && p.x() < c.max_x() && p.y() >= c.min_y() &&
         p.y() < c.max_y();
}

bool InCorridor(const Eigen::Vector3d& p, const FusionConfig& c) {
  return p.x() >= c.corridor_min_x() && p.x() <= c.corridor_max_x() &&
         std::abs(p.y()) <= c.corridor_half_width();
}

std::map<uint32_t, ClassMapping> Mappings(
    const google::protobuf::RepeatedPtrField<ClassMapping>& entries) {
  std::map<uint32_t, ClassMapping> result;
  for (const auto& entry : entries) {
    result.emplace(entry.raw_id(), entry);
  }
  return result;
}

bool ValidMappings(
    const google::protobuf::RepeatedPtrField<ClassMapping>& entries,
    bool detection) {
  std::set<uint32_t> seen;
  for (const auto& entry : entries) {
    if (!entry.has_raw_id() || !entry.has_group() ||
        !seen.insert(entry.raw_id()).second ||
        (detection && entry.group() != THING && entry.group() != GENERIC_OBS) ||
        (entry.group() == THING && entry.unified_class() == CLASS_UNKNOWN)) {
      return false;
    }
  }
  return !entries.empty();
}

Reason ValidateDetection(const drivers::PointCloud& cloud,
                         const DetectionFrame* input, const FusionConfig& c) {
  if (input == nullptr) {
    return SOURCE_MISSING;
  }
  if (!input->has_header() || !SameSweep(cloud, input->header()) ||
      input->sensor_frame_id() != c.sensor_frame()) {
    return SOURCE_MISALIGNED;
  }
  if (input->backend() != c.detection_backend() ||
      input->model_revision().empty() ||
      input->detection_size() > static_cast<int>(c.max_detections())) {
    return SOURCE_INVALID;
  }
  const auto mappings = Mappings(c.detection_mapping());
  for (const auto& det : input->detection()) {
    const std::array<double, 9> values = {
        det.center_x(), det.center_y(),   det.center_z(),
        det.size_x(),   det.size_y(),     det.size_z(),
        det.yaw(),      det.velocity_x(), det.velocity_y()};
    if (!std::all_of(values.begin(), values.end(),
                     [](double v) { return std::isfinite(v); }) ||
        !det.has_center_x() || !det.has_center_y() || !det.has_center_z() ||
        !det.has_yaw() || !det.has_score() || !det.has_class_id() ||
        det.size_x() <= 0 || det.size_y() <= 0 || det.size_z() <= 0 ||
        det.size_x() > 100 || det.size_y() > 100 || det.size_z() > 100 ||
        std::abs(det.center_x()) > 10000 || std::abs(det.center_y()) > 10000 ||
        std::abs(det.center_z()) > 10000 || !Probability(det.score()) ||
        det.class_id() < 0 || mappings.count(det.class_id()) == 0 ||
        det.has_velocity_x() != det.has_velocity_y()) {
      return SOURCE_INVALID;
    }
  }
  return NO_REASON;
}

Reason ValidateSemantic(const drivers::PointCloud& cloud,
                        const SemanticFrame* input, const FusionConfig& c) {
  if (input == nullptr) {
    return SOURCE_MISSING;
  }
  if (!input->has_header() || !SameSweep(cloud, input->header()) ||
      input->source_frame_id() != c.sensor_frame() ||
      input->source_topic() != c.pointcloud_channel()) {
    return SOURCE_MISALIGNED;
  }
  if (input->point_count() != static_cast<uint32_t>(cloud.point_size()) ||
      input->point_label_size() != cloud.point_size()) {
    return SOURCE_INVALID;
  }
  const auto mappings = Mappings(c.semantic_mapping());
  std::vector<bool> seen(cloud.point_size(), false);
  for (const auto& label : input->point_label()) {
    if (!label.has_point_index() || !label.has_semantic_label() ||
        !label.has_confidence() || label.point_index() >= seen.size() ||
        seen[label.point_index()] || !Probability(label.confidence()) ||
        mappings.count(label.semantic_label()) == 0) {
      return SOURCE_INVALID;
    }
    seen[label.point_index()] = true;
  }
  return NO_REASON;
}

struct GroundPlane {
  Eigen::Vector3d coefficients;
  double origin_x = 0;
  double origin_y = 0;
  double residual = 0;
  uint32_t count = 0;
};

using GroundPlanes = std::unordered_map<CellKey, GroundPlane, CellHash>;

bool SolveGround(const Eigen::MatrixXd& design, const Eigen::VectorXd& heights,
                 const std::vector<int>& rows, const FusionConfig& config,
                 GroundPlane* plane) {
  if (rows.size() < config.ground_min_points()) {
    return false;
  }
  Eigen::MatrixXd subset(rows.size(), 3);
  Eigen::VectorXd z(rows.size());
  for (std::size_t i = 0; i < rows.size(); ++i) {
    subset.row(i) = design.row(rows[i]);
    z[i] = heights[rows[i]];
  }
  const auto qr = subset.colPivHouseholderQr();
  if (qr.rank() != 3) {
    return false;
  }
  plane->coefficients = qr.solve(z);
  plane->residual = (subset * plane->coefficients - z).cwiseAbs().maxCoeff();
  plane->count = rows.size();
  return plane->coefficients.allFinite() &&
         plane->coefficients.head<2>().norm() <= config.ground_max_slope() &&
         plane->residual <= config.ground_max_residual();
}

bool RobustGround(const Eigen::MatrixXd& design, const Eigen::VectorXd& heights,
                  const FusionConfig& config, GroundPlane* plane) {
  std::vector<double> sorted(heights.data(), heights.data() + heights.size());
  auto median = sorted.begin() + sorted.size() / 2;
  std::nth_element(sorted.begin(), median, sorted.end());
  const std::array<Eigen::Vector3d, 2> starts{plane->coefficients,
                                              Eigen::Vector3d(0, 0, *median)};
  GroundPlane best = *plane;
  best.count = 0;
  for (const auto& start : starts) {
    GroundPlane candidate = *plane;
    candidate.coefficients = start;
    for (int iteration = 0; iteration < 3; ++iteration) {
      std::vector<int> rows;
      const Eigen::VectorXd residual =
          (design * candidate.coefficients - heights).cwiseAbs();
      for (int i = 0; i < residual.size(); ++i) {
        if (residual[i] <= config.ground_max_residual()) {
          rows.push_back(i);
        }
      }
      if (rows.size() < config.quality().ground_inlier_ratio() *
                            static_cast<double>(heights.size()) ||
          !SolveGround(design, heights, rows, config, &candidate)) {
        break;
      }
      if (candidate.count > best.count ||
          (candidate.count == best.count &&
           candidate.residual < best.residual)) {
        best = candidate;
      }
    }
  }
  if (best.count == 0) {
    return false;
  }
  *plane = best;
  return true;
}

GroundPlanes FitGround(const std::vector<Eigen::Vector3d>& positions,
                       const SpatialIndex& seed_tiles,
                       const FusionConfig& config) {
  GroundPlanes planes;
  planes.reserve(seed_tiles.size());
  for (const auto& tile : seed_tiles) {
    if (tile.second.size() < config.ground_min_points()) {
      continue;
    }
    GroundPlane plane;
    plane.origin_x = (tile.first.first + 0.5) * config.ground_tile_size();
    plane.origin_y = (tile.first.second + 0.5) * config.ground_tile_size();
    Eigen::MatrixXd design(tile.second.size(), 3);
    Eigen::VectorXd heights(tile.second.size());
    for (std::size_t row = 0; row < tile.second.size(); ++row) {
      const auto& p = positions[tile.second[row]];
      design.row(row) << p.x() - plane.origin_x, p.y() - plane.origin_y, 1;
      heights[row] = p.z();
    }
    const auto qr = design.colPivHouseholderQr();
    if (qr.rank() != 3) {
      continue;
    }
    plane.coefficients = qr.solve(heights);
    plane.residual =
        (design * plane.coefficients - heights).cwiseAbs().maxCoeff();
    plane.count = tile.second.size();
    const bool accepted =
        plane.coefficients.allFinite() &&
        plane.coefficients.head<2>().norm() <= config.ground_max_slope() &&
        plane.residual <= config.ground_max_residual();
    if (!accepted) {
      if (!config.quality().robust_ground() ||
          !RobustGround(design, heights, config, &plane)) {
        continue;
      }
    }
    planes.emplace(tile.first, plane);
  }
  return planes;
}

// Connected voxels, not all point pairs: dense returns do not create an O(N^2)
// neighborhood search. Ground/noise points cannot bridge obstacle components.
class ConnectedVoxels {
 public:
  ConnectedVoxels(const std::vector<Eigen::Vector3d>& positions,
                  const std::vector<bool>& eligible, double size)
      : count_(positions.size()) {
    std::vector<std::pair<VoxelKey, int>> sorted;
    sorted.reserve(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) {
      if (eligible[i]) {
        const auto& p = positions[i];
        sorted.push_back({{static_cast<int>(std::floor(p.x() / size)),
                           static_cast<int>(std::floor(p.y() / size)),
                           static_cast<int>(std::floor(p.z() / size))},
                          i});
      }
    }
    std::sort(sorted.begin(), sorted.end());
    for (const auto& entry : sorted) {
      if (buckets_.empty() || buckets_.back().key != entry.first) {
        buckets_.push_back({entry.first, ids_.size(), ids_.size(), 0, 0});
      }
      ids_.push_back(entry.second);
      buckets_.back().end = ids_.size();
    }
    std::unordered_map<VoxelKey, int, Hash> lookup;
    lookup.reserve(buckets_.size());
    for (std::size_t i = 0; i < buckets_.size(); ++i) {
      lookup.emplace(buckets_[i].key, i);
    }
    std::vector<std::pair<int, int>> edges;
    edges.reserve(buckets_.size());
    for (std::size_t i = 0; i < buckets_.size(); ++i) {
      const auto [x, y, z] = buckets_[i].key;
      for (int dx = 0; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && (dy < 0 || (dy == 0 && dz <= 0))) {
              continue;
            }
            const auto found = lookup.find({x + dx, y + dy, z + dz});
            if (found != lookup.end()) {
              edges.emplace_back(i, found->second);
              ++buckets_[i].neighbor_end;
              ++buckets_[found->second].neighbor_end;
            }
          }
        }
      }
    }
    std::size_t neighbor_count = 0;
    for (auto& bucket : buckets_) {
      bucket.neighbor_begin = neighbor_count;
      neighbor_count += bucket.neighbor_end;
      bucket.neighbor_end = bucket.neighbor_begin;
    }
    neighbors_.resize(neighbor_count);
    for (const auto& edge : edges) {
      neighbors_[buckets_[edge.first].neighbor_end++] = edge.second;
      neighbors_[buckets_[edge.second].neighbor_end++] = edge.first;
    }
    // Query each undirected edge once, retaining the original neighbor order.
    for (auto& bucket : buckets_) {
      std::sort(neighbors_.begin() + bucket.neighbor_begin,
                neighbors_.begin() + bucket.neighbor_end);
    }
  }

  std::vector<int> Components(const std::vector<bool>& eligible) const {
    std::vector<int> labels(buckets_.size(), -1), result(count_, -1);
    for (std::size_t i = 0; i < buckets_.size(); ++i) {
      const auto& bucket = buckets_[i];
      for (std::size_t j = bucket.begin; j < bucket.end; ++j) {
        if (eligible[ids_[j]]) {
          labels[i] = -2;
          break;
        }
      }
    }
    int component = 0;
    std::vector<int> pending;
    pending.reserve(buckets_.size());
    for (std::size_t i = 0; i < buckets_.size(); ++i) {
      if (labels[i] != -2) {
        continue;
      }
      labels[i] = component;
      pending.clear();
      pending.push_back(i);
      for (std::size_t j = 0; j < pending.size(); ++j) {
        const auto& bucket = buckets_[pending[j]];
        for (std::size_t k = bucket.neighbor_begin; k < bucket.neighbor_end;
             ++k) {
          const int next = neighbors_[k];
          if (labels[next] == -2) {
            labels[next] = component;
            pending.push_back(next);
          }
        }
      }
      ++component;
    }
    for (std::size_t i = 0; i < buckets_.size(); ++i) {
      const auto& bucket = buckets_[i];
      for (std::size_t j = bucket.begin; j < bucket.end; ++j) {
        if (eligible[ids_[j]]) {
          result[ids_[j]] = labels[i];
        }
      }
    }
    return result;
  }

  void Count(FusionWork* work) const {
    if (work != nullptr) {
      work->voxels = buckets_.size();
      work->neighbor_edges += neighbors_.size();
    }
  }

 private:
  struct Hash {
    std::size_t operator()(const VoxelKey& key) const {
      std::size_t result = 0;
      for (int value : {std::get<0>(key), std::get<1>(key), std::get<2>(key)}) {
        result ^= std::hash<int>{}(value) + 0x9e3779b9 + (result << 6) +
                  (result >> 2);
      }
      return result;
    }
  };
  struct Bucket {
    VoxelKey key;
    std::size_t begin;
    std::size_t end;
    std::size_t neighbor_begin;
    std::size_t neighbor_end;
  };
  std::size_t count_;
  std::vector<int> ids_;
  std::vector<Bucket> buckets_;
  std::vector<int> neighbors_;
};

std::pair<CellKey, CellKey> BoxRange(const OrientedBox& box,
                                     const FusionConfig& c, double margin) {
  const Eigen::Vector3d ext =
      box.rotation.cwiseAbs() *
      (box.size * 0.5 + Eigen::Vector3d::Constant(margin));
  const CellKey low =
      Key(std::max(c.min_x(), box.center.x() - ext.x()),
          std::max(c.min_y(), box.center.y() - ext.y()), c.resolution());
  const CellKey high =
      Key(std::min(c.max_x(), box.center.x() + ext.x()),
          std::min(c.max_y(), box.center.y() + ext.y()), c.resolution());
  return {low, high};
}

bool SemanticCompatible(const PointEvidence& point, UnifiedClass cls,
                        const FusionConfig& c) {
  return point.group() == THING && point.unified_class() == cls &&
         point.semantic_probability() >= c.semantic_min_confidence();
}

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

bool ObstacleHeight(const PointEvidence& p, const FusionConfig& c);

void AssociatePrior(const ObjectPrior* prior, const FusionConfig& c,
                    FusionOutput* out) {
  if (prior == nullptr) {
    return;
  }
  for (const auto& prediction : *prior) {
    const auto& b = prediction.box();
    const Eigen::Quaterniond rotation(b.qw(), b.qx(), b.qy(), b.qz());
    std::vector<int> points;
    for (int i = 0; i < out->point_evidence_size(); ++i) {
      const auto& p = out->point_evidence(i);
      const auto& d = out->point_decision(i);
      if (!p.finite() || d.has_object_id() || !ObstacleHeight(p, c) ||
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
        points.push_back(i);
      }
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

bool ObstacleHeight(const PointEvidence& p, const FusionConfig& c) {
  return !p.ground_valid() ||
         (p.height_above_ground() >= c.obstacle_min_height() &&
          p.height_above_ground() <= c.clearance_height());
}

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

}  // namespace

bool SameSweep(const drivers::PointCloud& cloud, const common::Header& header) {
  if (!cloud.has_header()) {
    return false;
  }
  const auto& source = cloud.header();
  if (source.has_sequence_num() && header.has_sequence_num() &&
      source.sequence_num() != header.sequence_num()) {
    return false;
  }
  if (source.lidar_timestamp() != 0 && header.lidar_timestamp() != 0) {
    return source.lidar_timestamp() == header.lidar_timestamp();
  }
  // PointPillars uses measurement_time as timestamp_sec; the other producers
  // copy the source header. Neither case permits a nearest-neighbor sweep join.
  return source.has_sequence_num() && header.has_sequence_num() &&
         header.has_timestamp_sec() && std::isfinite(header.timestamp_sec()) &&
         ((source.has_timestamp_sec() &&
           std::abs(source.timestamp_sec() - header.timestamp_sec()) <= 1e-6) ||
          (cloud.has_measurement_time() &&
           std::abs(cloud.measurement_time() - header.timestamp_sec()) <=
               1e-6));
}

bool SingleFrameFusion::Init(const FusionConfig& c, std::string* error) {
  initialized_ = false;
  const auto& t = c.sensor_to_base();
  const std::array<double, 38> finite_values = {
      t.x(),
      t.y(),
      t.z(),
      t.roll(),
      t.pitch(),
      t.yaw(),
      c.wait_ms(),
      c.resolution(),
      c.min_x(),
      c.max_x(),
      c.min_y(),
      c.max_y(),
      c.ground_tile_size(),
      c.expected_ground_z(),
      c.ground_seed_band(),
      c.ground_max_slope(),
      c.ground_max_residual(),
      c.ground_tolerance(),
      c.obstacle_min_height(),
      c.clearance_height(),
      c.corridor_min_x(),
      c.corridor_max_x(),
      c.corridor_half_width(),
      c.semantic_min_confidence(),
      c.noise_corridor_confidence(),
      c.noise_outside_confidence(),
      c.association_margin(),
      c.connectivity_size(),
      c.support_ratio(),
      c.conflict_ratio(),
      c.evidence_score_min(),
      c.unsupported_score_min(),
      c.recall_min_size(),
      c.recall_max_size(),
      static_cast<double>(c.ground_min_points()),
      static_cast<double>(c.core_min_points()),
      static_cast<double>(c.unknown_min_points()),
      static_cast<double>(c.recall_min_points())};
  if (!std::all_of(finite_values.begin(), finite_values.end(),
                   [](double v) { return std::isfinite(v); }) ||
      c.pointcloud_channel().empty() || c.detection_channel().empty() ||
      c.segmentation_channel().empty() || c.output_channel().empty() ||
      std::set<std::string>{c.pointcloud_channel(), c.detection_channel(),
                            c.segmentation_channel(), c.output_channel()}
              .size() != 4 ||
      c.sensor_frame().empty() || c.base_frame().empty() ||
      !c.has_sensor_to_base() || t.version().empty() || !t.has_x() ||
      !t.has_y() || !t.has_z() || !t.has_roll() || !t.has_pitch() ||
      !t.has_yaw() ||
      std::max({std::abs(t.x()), std::abs(t.y()), std::abs(t.z())}) > 100 ||
      c.resolution() < 0.05 || c.connectivity_size() < 0.05 ||
      c.ground_tile_size() < c.resolution() || c.max_x() <= c.min_x() ||
      c.max_y() <= c.min_y() ||
      std::max({std::abs(c.min_x()), std::abs(c.max_x()), std::abs(c.min_y()),
                std::abs(c.max_y())}) > 1000 ||
      c.max_points() == 0 || c.max_points() > 1000000 ||
      c.max_detections() == 0 || c.max_detections() > 4096 ||
      c.queue_capacity() == 0 || c.queue_capacity() > 64 || c.wait_ms() <= 0 ||
      c.wait_ms() > 10000 || c.ground_seed_band() <= 0 ||
      c.ground_min_points() < 3 || c.ground_max_slope() < 0 ||
      !Probability(c.quality().ground_inlier_ratio()) ||
      c.quality().ground_inlier_ratio() <= 0.5 ||
      c.ground_max_residual() <= 0 || c.ground_tolerance() <= 0 ||
      c.obstacle_min_height() <= c.ground_tolerance() ||
      c.clearance_height() <= c.obstacle_min_height() ||
      c.corridor_max_x() < c.corridor_min_x() || c.corridor_half_width() < 0 ||
      c.association_margin() < 0 || c.core_min_points() == 0 ||
      c.unknown_min_points() == 0 || c.recall_min_points() == 0 ||
      c.recall_min_size() <= 0 || c.recall_max_size() < c.recall_min_size() ||
      !Probability(c.semantic_min_confidence()) ||
      !Probability(c.noise_corridor_confidence()) ||
      !Probability(c.noise_outside_confidence()) ||
      c.noise_corridor_confidence() < c.noise_outside_confidence() ||
      !Probability(c.support_ratio()) || !Probability(c.conflict_ratio()) ||
      c.support_ratio() < 0.5 || c.conflict_ratio() < 0.5 ||
      c.support_ratio() + c.conflict_ratio() <= 1 ||
      !Probability(c.evidence_score_min()) ||
      !Probability(c.unsupported_score_min()) ||
      !Probability(c.dedup_point_overlap()) || c.dedup_point_overlap() <= 0 ||
      !Probability(c.dedup_bev_iou()) || c.dedup_bev_iou() <= 0 ||
      c.unsupported_score_min() < c.evidence_score_min() ||
      c.detection_backend() ==
          lidar_object_detection::LIDAR_OBJECT_DETECTION_BACKEND_UNSPECIFIED ||
      !ValidMappings(c.detection_mapping(), true) ||
      !ValidMappings(c.semantic_mapping(), false)) {
    return Fail("invalid fusion configuration/calibration/class mappings",
                error);
  }
  const double cells = std::ceil((c.max_x() - c.min_x()) / c.resolution()) *
                       std::ceil((c.max_y() - c.min_y()) / c.resolution());
  // Match the index origin to the published grid, avoiding half-cell drift.
  const auto on_grid = [&](double value) {
    return std::abs(value / c.resolution() -
                    std::round(value / c.resolution())) < 1e-6;
  };
  if (cells > 1000000 || !on_grid(c.min_x()) || !on_grid(c.min_y())) {
    return Fail("grid exceeds one million cells or has an unaligned origin",
                error);
  }
  if (!ValidateSpatial(c, error)) {
    return false;
  }
  if (!ValidateCandidates(c)) {
    return Fail("invalid candidate options", error);
  }
  if (c.publication().transport() == CORE_POD &&
      (c.publication().max_pod_frame_bytes() == 0 ||
       c.publication().max_pod_frame_bytes() >
           static_cast<uint32_t>(std::numeric_limits<int>::max()))) {
    return Fail("CORE_POD frame byte limit must be in [1, INT_MAX]", error);
  }
  const auto& diagnostic = c.publication().diagnostic_channel();
  if (!diagnostic.empty() &&
      (c.publication().detail() != FUNCTIONAL_SCENE ||
       diagnostic == c.pointcloud_channel() ||
       diagnostic == c.detection_channel() ||
       diagnostic == c.segmentation_channel() ||
       diagnostic == c.output_channel() ||
       (c.temporal().enabled() && diagnostic == c.temporal().pose_channel()))) {
    return Fail(
        "diagnostic channel needs FUNCTIONAL_SCENE and a distinct topic",
        error);
  }
  config_ = c;
  initialized_ = true;
  return true;
}

bool SingleFrameFusion::Process(const drivers::PointCloud& cloud,
                                const DetectionFrame* detection,
                                const SemanticFrame* segmentation,
                                FusionOutput* output, std::string* error,
                                const GroundPrior* ground_prior,
                                const ObjectPrior* object_prior,
                                FusionWork* work) const {
  if (work != nullptr) {
    *work = FusionWork{};
  }
  if (!initialized_ || output == nullptr) {
    return Fail("fusion is not initialized or output is null", error);
  }
  output->Clear();
  if (cloud.point_size() == 0 ||
      cloud.point_size() > static_cast<int>(config_.max_points()) ||
      !cloud.has_measurement_time() ||
      !std::isfinite(cloud.measurement_time()) ||
      cloud.measurement_time() <= 0 ||
      cloud.frame_id() != config_.sensor_frame() ||
      (cloud.header().has_frame_id() &&
       cloud.header().frame_id() != config_.sensor_frame()) ||
      !SameSweep(cloud, cloud.header())) {
    return Fail(
        "cloud needs a valid source identity, frame and measurement time",
        error);
  }
  const auto start = std::chrono::steady_clock::now();
  auto last_stage = start;
  auto stage = [&](std::size_t id) {
    if (work != nullptr) {
      const auto now = std::chrono::steady_clock::now();
      work->stage_ms[id] =
          std::chrono::duration<double, std::milli>(now - last_stage).count();
      last_stage = now;
    }
  };
  FusionOutput heap_frame;
  FusionOutput& frame =
      output->GetArena() == nullptr
          ? heap_frame
          : *google::protobuf::Arena::Create<FusionOutput>(output->GetArena());
  *frame.mutable_header() = cloud.header();
  frame.mutable_header()->set_timestamp_sec(cloud.measurement_time());
  frame.mutable_header()->set_frame_id(config_.base_frame());
  frame.mutable_header()->set_module_name("lidar_fusion");
  frame.set_calibration_version(config_.sensor_to_base().version());
  auto* health = frame.mutable_health();
  health->set_det_reason(ValidateDetection(cloud, detection, config_));
  health->set_seg_reason(ValidateSemantic(cloud, segmentation, config_));
  health->set_det_valid(health->det_reason() == NO_REASON);
  health->set_seg_valid(health->seg_reason() == NO_REASON);
  health->set_mode(health->det_valid()
                       ? (health->seg_valid() ? NORMAL : NO_SEG)
                       : (health->seg_valid() ? NO_DET : GEO_ONLY));
  if (!health->det_valid()) {
    detection = nullptr;
    frame.add_risk_event(health->det_reason());
  } else {
    frame.set_detection_model_revision(detection->model_revision());
  }
  if (!health->seg_valid()) {
    segmentation = nullptr;
    frame.add_risk_event(health->seg_reason());
  }
  std::vector<Eigen::Vector3d> positions;
  SpatialIndex index;
  BuildEvidence(cloud, segmentation, config_, &positions, &index, &frame,
                ground_prior);
  stage(0);
  std::vector<bool> eligible(positions.size(), false);
  for (int i = 0; i < frame.point_evidence_size(); ++i) {
    const auto& p = frame.point_evidence(i);
    eligible[i] = p.finite() && InRoi(positions[i], config_) &&
                  ObstacleHeight(p, config_) && !p.is_ground_geo() &&
                  frame.point_decision(i).explanation() != EXPL_NOISE;
  }
  const ConnectedVoxels voxels(positions, eligible,
                               config_.connectivity_size());
  const auto components = voxels.Components(eligible);
  voxels.Count(work);
  stage(1);
  const auto boxes =
      Associate(detection, config_, positions, index, components, &frame);
  AssociatePrior(object_prior, config_, &frame);
  stage(2);
  CandidateResult candidates;
  candidates.objects.Swap(frame.mutable_object());
  candidates.unknowns.Swap(frame.mutable_unknown());
  BuildCandidates(positions, components, voxels, config_,
                  frame.point_evidence(), frame.mutable_point_decision(),
                  &candidates);
  stage(3);
  CandidateWork relation_work;
  RefineCandidates(config_, frame.point_evidence(), &candidates,
                   work == nullptr ? nullptr : &relation_work, &index);
  candidates.objects.Swap(frame.mutable_object());
  candidates.unknowns.Swap(frame.mutable_unknown());
  if (work != nullptr) {
    work->relation_pairs = relation_work.pairs;
    work->relation_point_transforms = relation_work.point_transforms;
  }
  stage(4);
  // Space derives from immutable evidence, never from instance retention.
  auto cells = BuildSpace(config_, frame, boxes, frame.mutable_space_map());
  stage(5);
  EnrichSpatial(config_, &frame, &cells);
  stage(6);
  CheckOutput(config_, &frame, &cells);
  cells.Publish(frame.mutable_space_map());
  if (config_.candidates().shadow_semantic_partitions() &&
      health->seg_valid() &&
      !config_.quality().semantic_identity_unverified()) {
    PartitionUnknowns(config_, frame.point_evidence(), frame.unknown(),
                      frame.mutable_shadow_partition());
  }
  stage(7);
  health->set_latency_ms(std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - start)
                             .count());
  *output = std::move(frame);
  return true;
}

bool ProjectFunctionalOutput(const FusionConfig& config,
                             const drivers::PointCloud& cloud,
                             const FusionOutput& full, FusionOutput* output,
                             std::string* error) {
  if (output == nullptr || output == &full ||
      full.publication().detail() != FULL_EVIDENCE || !full.has_health() ||
      !full.header().has_timestamp_sec() ||
      full.header().timestamp_sec() != cloud.measurement_time() ||
      full.point_evidence_size() != cloud.point_size() ||
      full.point_decision_size() != cloud.point_size() ||
      cloud.point_size() == 0 || !SameSweep(cloud, full.header()) ||
      cloud.frame_id() != config.sensor_frame() ||
      full.header().frame_id() != config.base_frame() ||
      full.calibration_version() != config.sensor_to_base().version()) {
    return Fail(
        "functional projection needs the complete matching source frame",
        error);
  }
  FusionOutput heap_scene;
  FusionOutput& scene =
      output->GetArena() == nullptr
          ? heap_scene
          : *google::protobuf::Arena::Create<FusionOutput>(output->GetArena());
  *scene.mutable_header() = full.header();
  scene.set_calibration_version(full.calibration_version());
  if (full.has_detection_model_revision()) {
    scene.set_detection_model_revision(full.detection_model_revision());
  }
  scene.mutable_hypothesis()->CopyFrom(full.hypothesis());
  scene.mutable_object()->CopyFrom(full.object());
  scene.mutable_unknown()->CopyFrom(full.unknown());
  if (full.has_space_map()) {
    *scene.mutable_space_map() = full.space_map();
  }
  if (full.has_health()) {
    *scene.mutable_health() = full.health();
  }
  scene.mutable_risk_event()->CopyFrom(full.risk_event());
  scene.mutable_point_view()->CopyFrom(full.point_view());
  scene.mutable_track()->CopyFrom(full.track());
  scene.mutable_history_cell()->CopyFrom(full.history_cell());
  if (full.has_odom_frame()) {
    scene.set_odom_frame(full.odom_frame());
  }
  scene.mutable_shadow_partition()->CopyFrom(full.shadow_partition());
  auto* context = scene.mutable_publication();
  context->set_detail(FUNCTIONAL_SCENE);
  context->set_source_point_count(cloud.point_size());
  *context->mutable_source_header() = cloud.header();
  context->set_source_measurement_time(cloud.measurement_time());
  context->set_source_pointcloud_channel(config.pointcloud_channel());
  *context->mutable_sensor_to_base() = config.sensor_to_base();
  if (config.has_spatial()) {
    *context->mutable_spatial_options() = config.spatial();
  }
  if (!config.publication().diagnostic_channel().empty()) {
    context->set_diagnostic_channel(config.publication().diagnostic_channel());
  }
  context->set_shadow_partitions_enabled(
      config.candidates().shadow_semantic_partitions() &&
      full.health().seg_valid() &&
      !config.quality().semantic_identity_unverified());
  output->Swap(&scene);
  return true;
}

}  // namespace lidar_fusion
}  // namespace apollo
