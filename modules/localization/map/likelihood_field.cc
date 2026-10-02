// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/map/likelihood_field.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

#include "pcl/kdtree/kdtree_flann.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

bool ContainsBox(const Eigen::Vector3d& outer_min,
                 const Eigen::Vector3d& outer_max,
                 const Region3d& region) {
  return !region.id.empty() && region.minimum.allFinite() &&
         region.maximum.allFinite() &&
         (region.maximum.array() > region.minimum.array()).all() &&
         (region.minimum.array() >= outer_min.array()).all() &&
         (region.maximum.array() <= outer_max.array()).all();
}

bool Overlaps(const Region3d& a, const Region3d& b) {
  return ((a.minimum.array() < b.maximum.array()) &&
          (b.minimum.array() < a.maximum.array())).all();
}

bool CoversCell(const Eigen::Vector3d& cell_minimum,
                const Eigen::Vector3d& cell_maximum,
                const Region3d& region) {
  return (cell_minimum.array() >= region.minimum.array()).all() &&
         (cell_maximum.array() <= region.maximum.array()).all();
}

}  // namespace

bool FieldIdentity::operator==(const FieldIdentity& other) const {
  return map_id == other.map_id && version == other.version &&
         frame == other.frame && calibration == other.calibration;
}

Result LikelihoodField::Build(
    const pcl::PointCloud<pcl::PointXYZ>& map, const FieldConfig& config,
    std::shared_ptr<const LikelihoodField>* output) {
  if (output == nullptr || config.identity.map_id.empty() ||
      config.identity.version.empty() || config.identity.frame.empty() ||
      config.identity.calibration.empty() || !config.minimum.allFinite() ||
      !config.maximum.allFinite() ||
      (config.maximum.array() <= config.minimum.array()).any() ||
      config.minimum.cwiseAbs().maxCoeff() > 1e6 ||
      config.maximum.cwiseAbs().maxCoeff() > 1e6 ||
      !std::isfinite(config.resolution) || config.resolution < 0.01 ||
      !std::isfinite(config.maximum_distance) ||
      config.maximum_distance <= 0.0 || config.maximum_distance > 1e6 ||
      config.maximum_cells == 0 || config.maximum_cells > 4000000 ||
      config.maximum_map_points == 0 ||
      config.maximum_map_points > 2000000 ||
      config.support_regions.empty() || config.support_regions.size() > 4096 ||
      config.observed_regions.empty() || config.observed_regions.size() > 4096) {
    return {Reason::CONFIG_INVALID, "Invalid bounded likelihood field policy"};
  }
  std::set<std::string> support_ids;
  for (size_t i = 0; i < config.support_regions.size(); ++i) {
    const auto& region = config.support_regions[i];
    if (!ContainsBox(config.minimum, config.maximum, region) ||
        !support_ids.insert(region.id).second) {
      return {Reason::CONFIG_INVALID, "Invalid or duplicate search support region"};
    }
    for (size_t j = 0; j < i; ++j) {
      if (Overlaps(region, config.support_regions[j])) {
        return {Reason::CONFIG_INVALID, "Search support regions must not overlap"};
      }
    }
  }
  std::set<std::string> observed_ids;
  for (const auto& region : config.observed_regions) {
    if (!ContainsBox(config.minimum, config.maximum, region) ||
        !observed_ids.insert(region.id).second) {
      return {Reason::CONFIG_INVALID, "Invalid or duplicate observed map region"};
    }
  }
  if (map.empty() || map.size() > config.maximum_map_points) {
    return {Reason::INVALID_INPUT, "Empty or excessive map point count"};
  }
  for (const auto& point : map) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || std::abs(point.x) > 1e6 ||
        std::abs(point.y) > 1e6 || std::abs(point.z) > 1e6) {
      return {Reason::INVALID_INPUT, "Nonfinite or out-of-range map geometry"};
    }
  }
  std::shared_ptr<LikelihoodField> next(new LikelihoodField);
  next->config_ = config;
  size_t count = 1;
  for (int axis = 0; axis < 3; ++axis) {
    const double cells = std::ceil(
        (config.maximum(axis) - config.minimum(axis)) / config.resolution);
    if (!std::isfinite(cells) || cells < 1.0 ||
        cells > static_cast<double>(config.maximum_cells)) {
      return {Reason::CONFIG_INVALID, "Field axis exceeds cell budget"};
    }
    next->dimensions_[axis] = static_cast<size_t>(cells);
    if (next->dimensions_[axis] > config.maximum_cells / count) {
      return {Reason::CONFIG_INVALID, "Field volume exceeds cell budget"};
    }
    count *= next->dimensions_[axis];
  }
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr points(
      new pcl::PointCloud<pcl::PointXYZ>(map));
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(points);
  next->distances_.resize(count);
  next->observed_.resize(count, 0);
  std::vector<int> indices(1);
  std::vector<float> squared(1);
  for (size_t z = 0; z < next->dimensions_[2]; ++z) {
    for (size_t y = 0; y < next->dimensions_[1]; ++y) {
      for (size_t x = 0; x < next->dimensions_[0]; ++x) {
        const Eigen::Vector3d center = config.minimum +
            config.resolution *
                Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5);
        const Eigen::Vector3d cell_minimum =
            config.minimum + config.resolution * Eigen::Vector3d(x, y, z);
        const Eigen::Vector3d cell_maximum = cell_minimum +
            (config.maximum - cell_minimum)
                .cwiseMin(Eigen::Vector3d::Constant(config.resolution));
        const pcl::PointXYZ query(center.x(), center.y(), center.z());
        if (tree.nearestKSearch(query, 1, indices, squared) != 1 ||
            !std::isfinite(squared[0]) || squared[0] < 0.0f) {
          return {Reason::MAP_NOT_READY, "Distance field neighbor query failed"};
        }
        const size_t index =
            (z * next->dimensions_[1] + y) * next->dimensions_[0] + x;
        next->distances_[index] = static_cast<float>(std::min(
            config.maximum_distance, std::sqrt(static_cast<double>(squared[0]))));
        next->observed_[index] = std::any_of(
            config.observed_regions.begin(), config.observed_regions.end(),
            [&cell_minimum, &cell_maximum](const Region3d& region) {
              return CoversCell(cell_minimum, cell_maximum, region);
            });
      }
    }
  }
  *output = std::move(next);
  return {};
}

bool LikelihoodField::Lookup(const Eigen::Vector3d& point,
                             double* distance) const {
  return Query(point, distance) == FieldCellStatus::OBSERVED;
}

FieldCellStatus LikelihoodField::Query(const Eigen::Vector3d& point,
                                       double* distance) const {
  if (distance == nullptr || distances_.empty() || !point.allFinite() ||
      (point.array() < config_.minimum.array()).any() ||
      (point.array() >= config_.maximum.array()).any()) {
    return FieldCellStatus::OUTSIDE;
  }
  const Eigen::Vector3d cell =
      ((point - config_.minimum) / config_.resolution).array().floor();
  std::array<size_t, 3> index;
  for (int axis = 0; axis < 3; ++axis) {
    index[axis] = static_cast<size_t>(cell(axis));
    if (index[axis] >= dimensions_[axis]) {
      return FieldCellStatus::OUTSIDE;
    }
  }
  const size_t cell_index =
      (index[2] * dimensions_[1] + index[1]) * dimensions_[0] + index[0];
  if (observed_[cell_index] == 0) {
    return FieldCellStatus::UNKNOWN;
  }
  *distance = distances_[cell_index];
  return FieldCellStatus::OBSERVED;
}

double LikelihoodField::quantization_radius() const {
  return std::sqrt(3.0) * config_.resolution * 0.5;
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
