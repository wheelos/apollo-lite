/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <map>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>

#include "modules/perception/base/blob.h"
#include "modules/perception/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace lidar {

// CUDA helper context to run CenterPointTRT pre/post-process kernels without
// including pipeline/proto headers in NVCC compilation.
struct CenterPointTrtCudaContext {
  // Feature generation
  base::Blob<float>* canvas_feature_blob_ = nullptr;
  base::Blob<int>* grid2pointnum_blob_ = nullptr;
  base::Blob<int>* point2grid_blob_ = nullptr;
  base::Blob<float>* voxels_blob_ = nullptr;
  base::Blob<float>* pfn_pillar_feature_blob_ = nullptr;

  // Backbone blobs
  base::Blob<float>* output_box_blob_ = nullptr;
  base::Blob<float>* output_cls_blob_ = nullptr;
  base::Blob<float>* output_dir_blob_ = nullptr;

  // Decode/NMS blobs
  base::Blob<float>* all_res_box_blob_ = nullptr;
  base::Blob<float>* all_res_conf_blob_ = nullptr;
  base::Blob<int>* all_res_cls_blob_ = nullptr;
  base::Blob<float>* res_box_blob_ = nullptr;
  base::Blob<float>* res_conf_blob_ = nullptr;
  base::Blob<int>* res_cls_blob_ = nullptr;
  base::Blob<int>* res_box_num_blob_ = nullptr;
  base::Blob<float>* score_class_map_blob_ = nullptr;

  base::Blob<int>* res_sorted_indices_blob_ = nullptr;
  base::Blob<float>* box_for_nms_blob_ = nullptr;
  base::Blob<float>* box_corner_blob_ = nullptr;
  base::Blob<float>* remain_conf_blob_ = nullptr;
  base::Blob<bool>* rotate_overlapped_blob_ = nullptr;
  base::Blob<int>* kept_indices_blob_ = nullptr;

  // Assign points
  base::Blob<float>* box_corners_blob_ = nullptr;
  base::Blob<float>* box_rects_blob_ = nullptr;
  base::Blob<int>* valid_point_num_blob_ = nullptr;
  base::Blob<int>* valid_point_indices_blob_ = nullptr;
  base::Blob<int>* valid_point2boxid_blob_ = nullptr;

  // Point clouds (host)
  const base::PointFCloud* cur_cloud_ptr_ = nullptr;
  const base::PointFCloud* original_cloud_ = nullptr;

  // Cached device buffer for current point cloud (owned by stage)
  base::PointF* pc_gpu_ = nullptr;

  // Common config
  float x_min_range_ = 0.f;
  float x_max_range_ = 0.f;
  float y_min_range_ = 0.f;
  float y_max_range_ = 0.f;
  float z_min_range_ = 0.f;
  float z_max_range_ = 0.f;
  float voxel_x_size_ = 0.f;
  float voxel_y_size_ = 0.f;
  float x_offset_ = 0.f;
  float y_offset_ = 0.f;
  int grid_x_size_ = 0;
  int grid_y_size_ = 0;
  int map_size_ = 0;
  bool enable_rotate_45degree_ = false;
  bool use_input_norm_ = false;

  // CNNSEG features
  bool use_cnnseg_features_ = false;
  int cnnseg_feature_dim_ = 0;
  float height_bin_min_height_ = 0.f;
  float height_bin_voxel_size_ = 0.f;
  int height_bin_dim_ = 0;

  // PFN
  int voxel_feature_dim_ = 0;
  int pillar_feature_dim_ = 0;

  // Head/post-process
  int downsample_size_ = 0;
  int num_tasks_ = 0;
  int head_x_size_ = 0;
  int head_y_size_ = 0;
  int head_map_size_ = 0;
  int nms_pre_max_size_ = 0;
  int nms_post_max_size_ = 0;
  float nms_overlap_thresh_ = 0.f;
  int max_candidate_num_ = 0;
  int num_classes_ = 0;

  // Assign points params
  float top_enlarge_value_ = 0.f;
  float bottom_enlarge_value_ = 0.f;
  float width_enlarge_value_ = 0.f;
  float length_enlarge_value_ = 0.f;
  int total_cloud_size_ = 0;
  int max_valid_point_size_ = 0;

  // Per-task maps (copied from stage at call time)
  std::vector<int> num_classes_in_task_;
  std::vector<float> score_thresh_map_;
  std::map<std::string, int> head_map_;
  std::map<std::string, int> feature_offset_;

  cudaStream_t stream_ = nullptr;

  void GeneratePfnFeatureGPU();
  void GenerateBackboneFeatureGPU(const base::Blob<float>* pillar_feature_blob);
  void DecodeValidObjects(std::vector<int>* kept_indices);
 void SimpleAssignPoints2Boxid(const std::vector<int>& kept_indices);

 private:
  void AssignPoints2Boxid(const std::vector<int>& kept_indices);
  void ApplyRotateNms(const bool* rotate_overlapped, int valid_box_num,
                      const int* all_sorted_indices,
                      std::vector<int>* box_reverve_flag,
                      bool skip_suppressed);
  int ApplyNmsGPU(int box_num_pre, int all_res_num,
                  std::vector<int>* kept_indices);
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
