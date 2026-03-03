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

#include <memory>

#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/lib/detector/center_point_trt/voxel_downsample.h"

namespace apollo {
namespace perception {
namespace lidar {

class GpuDownSample {
 public:
  GpuDownSample();
  ~GpuDownSample();

  bool Init(double voxel_x, double voxel_y, double voxel_z, bool use_centroid);

  bool Process(base::PointFCloudPtr& cloud_ptr) const;

 private:
  bool downSample(base::PointF* point_cloud, int& point_cloud_size,
                  float resolution_x, float resolution_y, float resolution_z,
                  bool use_centroid_downsample) const;

 private:
  std::unique_ptr<VoxelDownSampleCuda> voxel_downsample_;
  double downsample_voxel_size_x_ = 0.09;
  double downsample_voxel_size_y_ = 0.09;
  double downsample_voxel_size_z_ = 0.09;
  bool downsample_use_centroid_ = false;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
