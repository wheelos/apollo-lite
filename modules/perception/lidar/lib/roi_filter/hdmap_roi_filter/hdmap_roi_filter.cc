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

#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"

#include <algorithm>
#include <limits>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_point_label.h"
#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/polygon_mask.h"
#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetAbsolutePath;
using apollo::cyber::common::GetProtoFromFile;
using apollo::cyber::common::PathExists;

bool HdmapPointCloudRoiFilter::Init(const ROIFilterInitOptions& options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));

  const std::string work_root = config_manager->work_root();
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));

  const std::string config_dir = GetAbsolutePath(work_root, root_path);
  std::string config_file =
      GetAbsolutePath(config_dir, "pointcloud_roi_filter.conf");
  if (!PathExists(config_file)) {
    config_file = GetAbsolutePath(config_dir, "hdmap_roi_filter.conf");
  }

  PointCloudRoiFilterConfig config;
  ACHECK(GetProtoFromFile(config_file, &config));

  return InternalInit(config);
}

bool HdmapPointCloudRoiFilter::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) return false;

  if (!stage_config.has_pointcloud_roi_filter_config()) {
    AERROR << "HdmapPointCloudRoiFilter config missing in StageConfig";
    return false;
  }
  return InternalInit(stage_config.pointcloud_roi_filter_config());
}

bool HdmapPointCloudRoiFilter::InternalInit(
    const PointCloudRoiFilterConfig& config) {
  pointcloud_roi_filter_config_ = config;
  range_ = config.range();
  cell_size_ = config.cell_size();
  extend_dist_ = config.extend_dist();
  no_edge_table_ = config.no_edge_table();
  set_roi_service_ = config.set_roi_service();

  // 1. Memory Pre-allocation
  // Estimate: ~200 polygons per scene is typical for junctions
  polygons_world_ptr_.reserve(500);
  polygons_local_.reserve(500);
  raw_polygons_.reserve(500);

  // 2. Initialize Bitmap Grid
  // The Bitmap size is static based on range/cell_size
  Eigen::Vector2d min_range(-range_, -range_);
  Eigen::Vector2d max_range(range_, range_);
  Eigen::Vector2d cell_size(cell_size_, cell_size_);

  bitmap_.Init(min_range, max_range, cell_size);

  AINFO << "HdmapPointCloudRoiFilter Inited. Range:" << range_
        << " Cell:" << cell_size_ << " Grid:" << bitmap_.map_size().transpose();

  return true;
}

bool HdmapPointCloudRoiFilter::Process(DataFrame* data_frame) {
  if (!data_frame || !data_frame->lidar_frame) return false;
  ROIFilterOptions options;  // Default options
  return Filter(options, data_frame->lidar_frame);
}

bool HdmapPointCloudRoiFilter::Filter(const ROIFilterOptions& options,
                                      LidarFrame* frame) {
  if (!frame || !frame->cloud) {
    AERROR << "Input cloud is null.";
    return false;
  }

  // [日志1] 输入统计
  size_t input_size = frame->cloud->size();
  AINFO << "[ROI-Filter] Input points: " << input_size;

  // Guard: If map is missing (MapManager failed), we pass through everything
  // or return false depending on safety policy.
  // Here we assume if hdmap_struct exists but is empty, it means "No Road".
  if (!frame->hdmap_struct) {
    AWARN << "No HDMap struct. Skipping ROI filter (Pass-Through).";
    // Fill all indices
    frame->roi_indices.indices.resize(frame->cloud->size());
    std::iota(frame->roi_indices.indices.begin(),
              frame->roi_indices.indices.end(), 0);
    AINFO << "[ROI-Filter] Result: PASS-THROUGH (no map), "
          << frame->roi_indices.indices.size() << " points";
    return true;
  }

  // 1. Collect Valid Polygons
  polygons_world_ptr_.clear();
  const auto& road_polys = frame->hdmap_struct->road_polygons;
  const auto& junc_polys = frame->hdmap_struct->junction_polygons;

  // [日志2] 多边形统计
  AINFO << "[ROI-Filter] HDMap polygons: "
        << road_polys.size() << " roads + "
        << junc_polys.size() << " junctions";

  if (road_polys.empty() && junc_polys.empty()) {
    AWARN << "No polygons in ROI. Filter out all points.";
    frame->roi_indices.indices.clear();
    AINFO << "[ROI-Filter] Result: EMPTY (no polygons)";
    return true;
  }

  // Pointer collection avoids deep copy of polygon vertices
  for (const auto& poly : road_polys)
    polygons_world_ptr_.push_back(const_cast<PolygonDType*>(&poly));
  for (const auto& poly : junc_polys)
    polygons_world_ptr_.push_back(const_cast<PolygonDType*>(&poly));

  // 2. Transform World Polygons -> Local Frame
  // This is O(N_poly_vertices), much cheaper than O(N_points)
  TransformPolygonsToLocal(frame->lidar2world_pose, polygons_world_ptr_,
                           polygons_local_);

  // [日志3] 坐标转换后
  AINFO << "[ROI-Filter] Transformed " << polygons_local_.size()
        << " polygons to local frame";

  // 3. Rasterize Polygons into Bitmap
  if (!PreparePolygonMask(polygons_local_, &raw_polygons_, &bitmap_)) {
    AWARN << "Failed to build polygon mask.";
    return false;
  }

  // [日志4] 位图信息
  AINFO << "[ROI-Filter] Bitmap size: " << bitmap_.map_size().transpose()
        << " major_dir: " << (static_cast<Bitmap2D::DirectionMajor>(bitmap_.dir_major()) == Bitmap2D::DirectionMajor::XMAJOR ? "X" : "Y");

  // 4. Query Bitmap (Filter Points)
  // This is O(N_points) but extremely fast (array lookup)
  Bitmap2dFilter(frame->cloud, bitmap_, &(frame->roi_indices));

  // [日志5] 最终结果
  size_t output_size = frame->roi_indices.indices.size();
  double keep_ratio = static_cast<double>(output_size) / input_size * 100.0;
  AINFO << "[ROI-Filter] Result: " << output_size << "/" << input_size
        << " points kept (" << keep_ratio << "%)";

  // 5. Labeling (for debug/visualization)
  auto* labels_local = frame->cloud->mutable_points_label();
  auto* labels_world = frame->world_cloud->mutable_points_label();
  const uint8_t roi_label = static_cast<uint8_t>(LidarPointLabel::ROI);

  // Direct access for speed
  const auto& indices = frame->roi_indices.indices;
  for (int idx : indices) {
    (*labels_local)[idx] = roi_label;
    // Check world cloud size safety, though usually synced
    if (static_cast<size_t>(idx) < labels_world->size()) {
      (*labels_world)[idx] = roi_label;
    }
  }

  // 6. Update Service (Optional)
  if (set_roi_service_) {
    UpdateService(frame->lidar2world_pose);
  }

  return true;
}

void HdmapPointCloudRoiFilter::TransformPolygonsToLocal(
    const Eigen::Affine3d& lidar2world_pose,
    const std::vector<PolygonDType*>& polygons_world,
    std::vector<PolygonDType>& polygons_local) {
  Eigen::Affine3d world2lidar = lidar2world_pose.inverse();

  polygons_local.clear();
  // Ensure capacity
  if (polygons_local.capacity() < polygons_world.size()) {
    polygons_local.reserve(polygons_world.size());
  }

  for (const auto* poly_ptr : polygons_world) {
    const auto& poly_w = *poly_ptr;

    // Grow local vector
    polygons_local.emplace_back();
    auto& poly_l = polygons_local.back();
    poly_l.resize(poly_w.size());

    // Batch Transform
    for (size_t i = 0; i < poly_w.size(); ++i) {
      // Use Eigen vector ops
      Eigen::Vector3d pt_w(poly_w[i].x, poly_w[i].y, poly_w[i].z);
      Eigen::Vector3d pt_l = world2lidar * pt_w;

      poly_l[i].x = pt_l.x();
      poly_l[i].y = pt_l.y();
      poly_l[i].z = pt_l.z();
    }
  }
}

bool HdmapPointCloudRoiFilter::PreparePolygonMask(
    const std::vector<PolygonDType>& polygons,
    std::vector<Polygon<double>>* raw_polygons, Bitmap2D* bitmap) {
  // Reset raw polygons buffer
  raw_polygons->clear();
  if (raw_polygons->capacity() < polygons.size()) {
    raw_polygons->reserve(polygons.size());
  }

  // Determine bounds for Major Axis selection
  double min_x = range_;
  double max_x = -range_;
  double min_y = range_;
  double max_y = -range_;

  for (const auto& poly : polygons) {
    raw_polygons->emplace_back();
    auto& raw_poly = raw_polygons->back();
    raw_poly.resize(poly.size());

    for (size_t i = 0; i < poly.size(); ++i) {
      // Flatten 3D polygon to 2D
      raw_poly[i].x() = poly[i].x;
      raw_poly[i].y() = poly[i].y;

      min_x = std::min(raw_poly[i].x(), min_x);
      max_x = std::max(raw_poly[i].x(), max_x);
      min_y = std::min(raw_poly[i].y(), min_y);
      max_y = std::max(raw_poly[i].y(), max_y);
    }
  }

  // Heuristic: Scanline algo is faster along the shorter dimension
  DirectionMajor major_dir = DirectionMajor::XMAJOR;
  if ((max_y - min_y) < (max_x - min_x)) {
    major_dir = DirectionMajor::YMAJOR;
  }

  bitmap->SetUp(major_dir);

  // Rasterize
  return DrawPolygonsMask<double>(*raw_polygons, bitmap, extend_dist_,
                                  no_edge_table_);
}

bool HdmapPointCloudRoiFilter::Bitmap2dFilter(const base::PointFCloudPtr& cloud,
                                              const Bitmap2D& bitmap,
                                              base::PointIndices* roi_indices) {
  roi_indices->indices.clear();
  roi_indices->indices.reserve(cloud->size());

  size_t size = cloud->size();
  for (size_t i = 0; i < size; ++i) {
    const auto& pt = cloud->at(i);
    // Boundary check is inside Bitmap2D::Check usually
    if (bitmap.Check(Eigen::Vector2d(pt.x, pt.y))) {
      roi_indices->indices.push_back(static_cast<int>(i));
    }
  }
  return true;
}

void HdmapPointCloudRoiFilter::UpdateService(
    const Eigen::Affine3d& lidar2world_pose) {
  auto roi_service = SceneManager::Instance().Service("ROIService");
  if (!roi_service) return;

  roi_service_content_.range_ = range_;
  roi_service_content_.cell_size_ = cell_size_;
  roi_service_content_.map_size_ = bitmap_.map_size();
  roi_service_content_.bitmap_ = bitmap_.bitmap();
  roi_service_content_.major_dir_ =
      static_cast<ROIServiceContent::DirectionMajor>(bitmap_.dir_major());
  roi_service_content_.transform_ = lidar2world_pose.translation();

  roi_service->UpdateServiceContent(roi_service_content_);
}

PERCEPTION_REGISTER_ROIFILTER(HdmapPointCloudRoiFilter);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
