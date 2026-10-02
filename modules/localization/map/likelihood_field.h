// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "modules/localization/core/types.h"

namespace apollo {
namespace localization {
namespace unified {

struct FieldIdentity {
  std::string map_id;
  std::string version;
  std::string frame;
  std::string calibration;
  bool operator==(const FieldIdentity& other) const;
};

struct Region3d {
  std::string id;
  Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
  Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
};

struct FieldConfig {
  FieldIdentity identity;
  // Explicit bounded score domain, NOT inferred free space or visibility.
  Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
  Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
  // Disjoint prior/search support. These are not a navigability assertion.
  std::vector<Region3d> support_regions;
  // Regions where distance-to-surface scoring has map coverage. This does not
  // imply ray visibility or that an unmodeled point is a free-space conflict.
  std::vector<Region3d> observed_regions;
  double resolution = 0.0;
  double maximum_distance = 0.0;
  size_t maximum_cells = 0;
  size_t maximum_map_points = 0;
};

enum class FieldCellStatus {
  OUTSIDE,
  UNKNOWN,
  OBSERVED,
};

class LikelihoodField {
 public:
  // Builds an immutable, bounded cell-center distance field. Failure leaves
  // output untouched; it must never be interpreted as a loaded empty map.
  static Result Build(const pcl::PointCloud<pcl::PointXYZ>& map,
                      const FieldConfig& config,
                      std::shared_ptr<const LikelihoodField>* output);
  FieldCellStatus Query(const Eigen::Vector3d& point,
                        double* distance) const;
  bool Lookup(const Eigen::Vector3d& point, double* distance) const;
  const FieldConfig& config() const { return config_; }
  // Geometric half-cell diagonal; excludes map/sensor/numerical uncertainty.
  double quantization_radius() const;

 private:
  FieldConfig config_;
  std::array<size_t, 3> dimensions_{{0, 0, 0}};
  std::vector<float> distances_;
  std::vector<uint8_t> observed_;
};

}  // namespace unified
}  // namespace localization
}  // namespace apollo
