/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/pipeline/proto/stage/hdmap_roi_filter_config.pb.h"

#include "modules/common/util/eigen_defs.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/lib/interface/base_roi_filter.h"
#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/bitmap2d.h"
#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/polygon_scan_cvter.h"
#include "modules/perception/lidar/lib/scene_manager/roi_service/roi_service.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace lidar {

class HdmapROIFilterTest;

// Point-level ROI extractor. It converts HDMap polygons into the lidar local
// frame and populates roi_indices for downstream stages.
class HdmapPointCloudRoiFilter : public BaseROIFilter {
 public:
  using DirectionMajor = Bitmap2D::DirectionMajor;
  using PolygonDType = base::PolygonDType;

  // Use simple vector for internal buffers
  template <typename T>
  using Polygon = typename PolygonScanCvter<T>::Polygon;

 public:
  HdmapPointCloudRoiFilter()
      : BaseROIFilter(),
        range_(120.0),
        cell_size_(0.25),
        extend_dist_(0.0),
        no_edge_table_(false) {
    name_ = "HdmapPointCloudRoiFilter";
  }
  ~HdmapPointCloudRoiFilter() = default;

  bool Init(const ROIFilterInitOptions& options) override;

  bool Filter(const ROIFilterOptions& options, LidarFrame* frame) override;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  bool InternalInit(const PointCloudRoiFilterConfig& config);

  // Core Helper: Transform polygons from World to Local frame
  void TransformPolygonsToLocal(
      const Eigen::Affine3d& lidar2world_pose,
      const std::vector<PolygonDType*>& polygons_world,
      std::vector<PolygonDType>& polygons_local);

  // Core Helper: Rasterize polygons into bitmap
  bool PreparePolygonMask(const std::vector<PolygonDType>& polygons,
                          std::vector<Polygon<double>>* raw_polygons,
                          Bitmap2D* bitmap);

  // Core Helper: Query bitmap for points
  bool Bitmap2dFilter(const base::PointFCloudPtr& in_cloud,
                      const Bitmap2D& bitmap, base::PointIndices* roi_indices);

  void UpdateService(const Eigen::Affine3d& lidar2world_pose);

  // Parameters
  double range_ = 120.0;
  double cell_size_ = 0.25;
  double extend_dist_ = 0.0;
  bool no_edge_table_ = false;
  bool set_roi_service_ = false;

  // Internal Reusable Buffers (avoid alloc per frame)
  std::vector<PolygonDType*> polygons_world_ptr_;
  std::vector<PolygonDType> polygons_local_;
  std::vector<Polygon<double>> raw_polygons_;  // For Bitmap drawer

  Bitmap2D bitmap_;
  ROIServiceContent roi_service_content_;
  PointCloudRoiFilterConfig pointcloud_roi_filter_config_;

  friend class HdmapROIFilterTest;
  friend class LidarLibROIServiceTest;
};

using HdmapROIFilter = HdmapPointCloudRoiFilter;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
