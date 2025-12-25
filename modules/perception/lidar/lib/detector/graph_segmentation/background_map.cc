/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar/lib/detector/graph_segmentation/background_map.h"

#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace lidar {

bool BackgroundMap::Init(int width, int height, float resolution,
                         float height_threshold) {
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  height_threshold_ = height_threshold;
  return true;
}

bool BackgroundMap::Init(float xmin, float xmax, float ymin, float ymax,
                         float resolution, float height_threshold,
                         float car_xmin, float car_xmax, float car_ymin,
                         float car_ymax, float car_zmax, float min_radius,
                         float z_min_from_ground) {
  resolution_ = resolution;
  xmin_ = xmin;
  xmax_ = xmax;
  ymin_ = ymin;
  ymax_ = ymax;
  height_threshold_ = height_threshold;
  car_xmin_ = car_xmin;
  car_xmax_ = car_xmax;
  car_ymin_ = car_ymin;
  car_ymax_ = car_ymax;
  car_zmax_ = car_zmax;
  min_radius_ = min_radius;
  z_min_from_ground_ = z_min_from_ground;

  // in pointcloud-cooridinate system
  width_ = static_cast<int>((ymax - ymin) * 1.0f / resolution);
  height_ = static_cast<int>((xmax - xmin) * 1.0f / resolution);
  return true;
}

bool BackgroundMap::Reset(size_t point_number) {
  // clear all data
  bg_nodes_.clear();
  point_idx_.clear();
  point_importance_.clear();
  point_mask_.clear();
  point_number_ = point_number;
  valid_node_number_ = 0;

  // reset data
  bg_nodes_.resize(width_ * height_);
  point_idx_.resize(point_number_, 0);
  point_mask_.resize(point_number_, false);
  point_importance_.resize(point_number_, false);
  return true;
}

bool BackgroundMap::UpdateMap(LidarFrame* frame) {
  // update idx and mask
  ACHECK(UpdateMask(frame));
  // update node
  ACHECK(UpdateNodes(frame));
  return true;
}

bool BackgroundMap::UpdateMask(LidarFrame* frame) {
  // Prefer non-ground indices. Fallback to ROI, then to all points.
  const std::vector<int>* indices = &frame->non_ground_indices.indices;
  std::vector<int> all_indices;
  if (indices->empty()) {
    if (!frame->non_ground_indices.indices.empty()) {
      indices = &frame->non_ground_indices.indices;
    } else if (!frame->roi_indices.indices.empty()) {
      indices = &frame->roi_indices.indices;
    } else {
      AINFO << "UpdateMask: no secondary/non_ground/roi indices, using all "
               "points.";
      all_indices.resize(static_cast<size_t>(point_number_));
      for (size_t i = 0; i < point_number_; ++i) {
        all_indices[static_cast<size_t>(i)] = static_cast<int>(i);
      }
      indices = &all_indices;
    }
  }

  // Logging input sizes for analysis
  AINFO << "UpdateMask input sizes, cloud: " << point_number_
        << ", non_ground: " << frame->non_ground_indices.indices.size()
        << ", roi: " << frame->roi_indices.indices.size()
        << ", secondary: " << frame->secondary_indices.indices.size();

  // Counters for debugging/analysis
  size_t considered_cnt = indices->size();
  size_t pass_height_cnt = 0;
  size_t pass_grid_cnt = 0;
  size_t final_mask_cnt = 0;

  // iterate chosen indices
  for (auto index : *indices) {
    if (index < 0 || static_cast<size_t>(index) >= point_number_) {
      continue;
    }
    auto pt = frame->cloud->at(index);
    // filter point beyond height_threshold_
    Eigen::Vector4d trans_point(pt.x, pt.y, pt.z, 1);
    trans_point = frame->lidar2novatel_extrinsics * trans_point;
    if (trans_point(2) >= height_threshold_) {
      continue;
    }
    float r = std::hypot(pt.x, pt.y);
    if (r < min_radius_) {
      continue;
    }

    Eigen::Vector4d p_nv(pt.x, pt.y, pt.z, 1);
    p_nv = frame->lidar2novatel_extrinsics * p_nv;
    if (p_nv(0) > car_xmin_ && p_nv(0) < car_xmax_ && p_nv(1) > car_ymin_ &&
        p_nv(1) < car_ymax_ && p_nv(2) < car_zmax_) {
      continue;
    }

    if (p_nv(2) <= z_min_from_ground_) {
      continue;
    }

    ++pass_height_cnt;
    // filter point outside grid
    int grid_x = -1;
    int grid_y = -1;
    GetNodeCoord(pt.x, pt.y, &grid_x, &grid_y);
    if (grid_y < 0 || grid_y >= width_ || grid_x < 0 || grid_x >= height_) {
      continue;
    }
    ++pass_grid_cnt;
    // valid point
    size_t grid_idx =
        static_cast<size_t>(grid_x) * static_cast<size_t>(width_) +
        static_cast<size_t>(grid_y);
    point_mask_.at(static_cast<size_t>(index)) = true;
    point_idx_.at(static_cast<size_t>(index)) = grid_idx;
    ++final_mask_cnt;
  }

  AINFO << "UpdateMask stats, considered: " << considered_cnt
        << ", pass_height: " << pass_height_cnt
        << ", pass_grid: " << pass_grid_cnt << ", masked: " << final_mask_cnt;

  return true;
}

bool BackgroundMap::UpdateNodes(LidarFrame* frame) {
  auto original_cloud = frame->cloud;
  size_t masked_points = 0;
  for (size_t i = 0; i < point_mask_.size(); ++i) {
    if (!point_mask_.at(i)) {
      continue;
    }
    ++masked_points;
    auto pt = original_cloud->at(i);
    // update node state
    size_t grid_idx = point_idx_.at(i);
    bg_nodes_.at(grid_idx).is_valid = true;
    // importance not used in 8.0
    bg_nodes_.at(grid_idx).is_important = bg_nodes_.at(grid_idx).is_important;

    if (bg_nodes_.at(grid_idx).point_number == 0) {
      bg_nodes_.at(grid_idx).id = valid_node_number_;
      valid_node_number_ += 1;
    }
    bg_nodes_.at(grid_idx).point_number += 1;
    // update node feature
    int pt_number = bg_nodes_.at(grid_idx).point_number;
    // use ground-relative height if available; fallback to raw z
    float h_rel = original_cloud->points_height(i);
    bool has_ground_height = (h_rel != std::numeric_limits<float>::max());
    float z_feat = has_ground_height ? h_rel : pt.z;
    // mean height, mean intensity
    if (pt_number == 1) {
      bg_nodes_.at(grid_idx).mean_height = z_feat;
      bg_nodes_.at(grid_idx).mean_intensity = pt.intensity;
    } else {
      bg_nodes_.at(grid_idx).mean_height =
          bg_nodes_.at(grid_idx).mean_height +
          (z_feat - bg_nodes_.at(grid_idx).mean_height) /
              static_cast<float>(pt_number);
      bg_nodes_.at(grid_idx).mean_intensity =
          bg_nodes_.at(grid_idx).mean_intensity +
          (pt.intensity - bg_nodes_.at(grid_idx).mean_intensity) /
              static_cast<float>(pt_number);
    }
    // max height
    bg_nodes_.at(grid_idx).max_height =
        std::max(bg_nodes_.at(grid_idx).max_height, z_feat);

    // semantic/motion labels not used in 8.0
  }
  AINFO << "UpdateNodes stats, masked_points: " << masked_points
        << ", valid_nodes: " << valid_node_number_;
  return true;
}

void BackgroundMap::GetNodeCoord(float x, float y, int* grid_x, int* grid_y) {
  *grid_x = static_cast<int>((x - xmin_) / resolution_);
  *grid_y = static_cast<int>((y - ymin_) / resolution_);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
