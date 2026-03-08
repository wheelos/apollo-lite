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

#include "modules/perception/lidar/lib/detector/center_point_trt/gpu_down_sample.h"

#include <cstdlib>

#include <cuda_runtime_api.h>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace lidar {

GpuDownSample::GpuDownSample()
    : voxel_downsample_(std::make_unique<VoxelDownSampleCuda>()) {}

GpuDownSample::~GpuDownSample() = default;

bool GpuDownSample::Init(double voxel_x, double voxel_y, double voxel_z,
                         bool use_centroid) {
  downsample_voxel_size_x_ = voxel_x;
  downsample_voxel_size_y_ = voxel_y;
  downsample_voxel_size_z_ = voxel_z;
  downsample_use_centroid_ = use_centroid;
  AINFO << "GpuDownSample init success.";
  return true;
}

static int GetNumberOfAvailableThreads() {
  cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, 0);
  if (prop.major == 2) {
    return prop.maxThreadsPerBlock / 2;
  }
  if (prop.major > 2) {
    return prop.maxThreadsPerBlock;
  }
  return 0;
}

bool GpuDownSample::Process(base::PointFCloudPtr& cloud_ptr) const {
  if (cloud_ptr == nullptr || cloud_ptr->empty()) {
    return true;
  }

  base::PointF* h_point_cloud = nullptr;
  base::PointF* d_point_cloud = nullptr;
  cudaError_t err = ::cudaSuccess;

  err = cudaHostAlloc(reinterpret_cast<void**>(&h_point_cloud),
                      cloud_ptr->size() * sizeof(base::PointF),
                      cudaHostAllocMapped);
  if (err != ::cudaSuccess) {
    AERROR << "cudaHostAlloc failed: " << cudaGetErrorString(err);
    return false;
  }

  err = cudaHostGetDevicePointer(reinterpret_cast<void**>(&d_point_cloud),
                                 h_point_cloud, 0);
  if (err != ::cudaSuccess) {
    AERROR << "cudaHostGetDevicePointer failed: " << cudaGetErrorString(err);
    cudaFreeHost(h_point_cloud);
    return false;
  }

  for (size_t i = 0; i < cloud_ptr->size(); ++i) {
    h_point_cloud[i].x = cloud_ptr->at(i).x;
    h_point_cloud[i].y = cloud_ptr->at(i).y;
    h_point_cloud[i].z = cloud_ptr->at(i).z;
    h_point_cloud[i].intensity = cloud_ptr->at(i).intensity;
  }

  int filtered_point_count = static_cast<int>(cloud_ptr->size());
  if (!downSample(d_point_cloud, filtered_point_count,
                  static_cast<float>(downsample_voxel_size_x_),
                  static_cast<float>(downsample_voxel_size_y_),
                  static_cast<float>(downsample_voxel_size_z_),
                  downsample_use_centroid_)) {
    AERROR << "GpuDownSample::downSample failed.";
    cudaFreeHost(h_point_cloud);
    return false;
  }
  cudaDeviceSynchronize();

  cloud_ptr->resize(filtered_point_count);
  for (int i = 0; i < filtered_point_count; ++i) {
    cloud_ptr->at(i).x = h_point_cloud[i].x;
    cloud_ptr->at(i).y = h_point_cloud[i].y;
    cloud_ptr->at(i).z = h_point_cloud[i].z;
    cloud_ptr->at(i).intensity = h_point_cloud[i].intensity;
  }

  cudaFreeHost(h_point_cloud);
  return true;
}

bool GpuDownSample::downSample(base::PointF* point_cloud, int& point_cloud_size,
                               float resolution_x, float resolution_y,
                               float resolution_z,
                               bool use_centroid_downsample) const {
  gridParameters rgd_params;
  hashElement* d_hash_table = nullptr;
  voxel* d_voxel = nullptr;
  bool* d_markers = nullptr;
  bool* h_markers = nullptr;
  int* count = nullptr;

  int threads = GetNumberOfAvailableThreads();
  if (threads == 0) {
    AERROR << "No available CUDA threads.";
    return false;
  }

  cudaError_t err = voxel_downsample_->cudaCalculateGridParams(
      point_cloud, point_cloud_size, resolution_x, resolution_y, resolution_z,
      rgd_params);
  if (err != ::cudaSuccess) {
    return false;
  }

  err = cudaMalloc(reinterpret_cast<void**>(&d_hash_table),
                   point_cloud_size * sizeof(hashElement));
  if (err != ::cudaSuccess) {
    return false;
  }
  err = cudaMalloc(reinterpret_cast<void**>(&count), sizeof(int));
  if (err != ::cudaSuccess) {
    cudaFree(d_hash_table);
    return false;
  }

  if (use_centroid_downsample) {
    err = cudaMalloc(reinterpret_cast<void**>(&d_voxel),
                     point_cloud_size * sizeof(voxel));
    if (err != ::cudaSuccess) {
      cudaFree(d_hash_table);
      cudaFree(count);
      return false;
    }
  }

  err = voxel_downsample_->cudaCalculateGrid(
      threads, point_cloud, d_hash_table, point_cloud_size, rgd_params, count);
  if (err != ::cudaSuccess) {
    cudaFree(d_hash_table);
    cudaFree(count);
    if (d_voxel) cudaFree(d_voxel);
    return false;
  }

  if (!use_centroid_downsample) {
    err = cudaMalloc(reinterpret_cast<void**>(&d_markers),
                     point_cloud_size * sizeof(bool));
    if (err != ::cudaSuccess) {
      cudaFree(d_hash_table);
      cudaFree(count);
      return false;
    }
  }

  err = voxel_downsample_->cudaDownSample(
      threads, point_cloud, d_markers, d_hash_table, d_voxel, point_cloud_size,
      rgd_params, use_centroid_downsample, count);
  if (err != ::cudaSuccess) {
    cudaFree(d_hash_table);
    cudaFree(count);
    if (d_voxel) cudaFree(d_voxel);
    if (d_markers) cudaFree(d_markers);
    return false;
  }

  if (use_centroid_downsample) {
    int host_count = 0;
    err = cudaMemcpy(&host_count, count, sizeof(int), cudaMemcpyDeviceToHost);
    if (err != ::cudaSuccess) {
      cudaFree(d_hash_table);
      cudaFree(count);
      cudaFree(d_voxel);
      return false;
    }
    point_cloud_size = host_count;
  } else {
    h_markers =
        reinterpret_cast<bool*>(malloc(point_cloud_size * sizeof(bool)));
    err = cudaMemcpy(h_markers, d_markers, point_cloud_size * sizeof(bool),
                     cudaMemcpyDeviceToHost);
    if (err != ::cudaSuccess) {
      free(h_markers);
      cudaFree(d_hash_table);
      cudaFree(count);
      cudaFree(d_markers);
      return false;
    }
    int valid_size = 0;
    for (int i = 0; i < point_cloud_size; ++i) {
      if (h_markers[i]) {
        point_cloud[valid_size] = point_cloud[i];
        valid_size += 1;
      }
    }
    point_cloud_size = valid_size;
    free(h_markers);
  }

  cudaFree(d_hash_table);
  cudaFree(count);
  if (d_voxel) cudaFree(d_voxel);
  if (d_markers) cudaFree(d_markers);
  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
