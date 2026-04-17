/******************************************************************************
 * Copyright 2026 The WheelOS Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <string>
#include <vector>

#include "Eigen/Dense"

#include "modules/common/util/eigen_defs.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/lib/interface/base_object_filter.h"
#include "modules/perception/pipeline/plugin.h"
#include "modules/perception/pipeline/proto/pipeline_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

// Polygon region defined by a list of 2D points
struct Polygon2D {
  std::string name;
  std::vector<Eigen::Vector2d> points;
};

class PolygonRegionObjectFilter : public BaseObjectFilter {
 public:
  using PluginConfig = pipeline::PluginConfig;

  PolygonRegionObjectFilter() { name_ = "PolygonRegionObjectFilter"; }

  explicit PolygonRegionObjectFilter(const PluginConfig& plugin_config) {
    name_ = "PolygonRegionObjectFilter";
    Init(plugin_config);
  }

  virtual ~PolygonRegionObjectFilter() = default;

  bool Init(const ObjectFilterInitOptions& options =
                ObjectFilterInitOptions()) override;

  bool Filter(const ObjectFilterOptions& options, LidarFrame* frame) override;

  bool Init(const PluginConfig& plugin_config) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  // Check if a point is inside a polygon using ray casting algorithm
  bool IsPointInPolygon(const Eigen::Vector2d& point,
                        const Polygon2D& polygon) const;

  // Check if an object is inside any of the given polygons
  bool IsObjectInPolygon(const base::ObjectPtr& object,
                         const std::vector<Polygon2D>& polygons,
                         const Eigen::Affine3d& lidar2world) const;

  // Check if any point of object polygon is inside the given polygon
  bool IsObjectPolygonInPolygon(const base::ObjectPtr& object,
                                const Polygon2D& polygon,
                                const Eigen::Affine3d& lidar2world) const;

  // Check if object center is inside the given polygon
  bool IsObjectCenterInPolygon(const base::ObjectPtr& object,
                               const Polygon2D& polygon,
                               const Eigen::Affine3d& lidar2world) const;

  // Load polygon regions from config
  bool LoadPolygonRegions(const PolygonRegionFilterConfig& config);

 private:
  std::vector<Polygon2D> exclude_regions_;
  std::vector<Polygon2D> include_regions_;
  bool use_center_point_only_ = false;
  bool use_world_coordinate_ = true;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
