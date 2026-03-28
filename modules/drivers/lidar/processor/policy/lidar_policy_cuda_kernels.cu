#include "modules/drivers/lidar/processor/policy/lidar_policy_cuda_kernels.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace apollo {
namespace drivers {
namespace lidar {

namespace {

constexpr uint64_t kSecondToNano = 1000000000ULL;
constexpr uint64_t kEmptyVoxel = std::numeric_limits<uint64_t>::max();
constexpr uint64_t kPointInfThreshold = 1000ULL;

__host__ __device__ inline float MatAt(const CudaMatrix4f* m, int r, int c) {
  return m->data[r * 4 + c];
}

__host__ __device__ inline float3 ApplyAffine(const CudaMatrix4f* m,
                                              const CudaPointXYZIT& p) {
  float3 out;
  out.x = MatAt(m, 0, 0) * p.x + MatAt(m, 0, 1) * p.y + MatAt(m, 0, 2) * p.z +
          MatAt(m, 0, 3);
  out.y = MatAt(m, 1, 0) * p.x + MatAt(m, 1, 1) * p.y + MatAt(m, 1, 2) * p.z +
          MatAt(m, 1, 3);
  out.z = MatAt(m, 2, 0) * p.x + MatAt(m, 2, 1) * p.y + MatAt(m, 2, 2) * p.z +
          MatAt(m, 2, 3);
  return out;
}

__device__ inline int ResolveNearestPoseIndex(double point_ts,
                                              const double* sample_times,
                                              int sample_count) {
  if (sample_count <= 1) {
    return 0;
  }
  int best_idx = 0;
  double best_dist = fabs(point_ts - sample_times[0]);
  for (int i = 1; i < sample_count; ++i) {
    const double dist = fabs(point_ts - sample_times[i]);
    if (dist < best_dist) {
      best_idx = i;
      best_dist = dist;
    }
  }
  return best_idx;
}

__global__ void TimestampRangeKernel(const uint64_t* timestamps, size_t count,
                                     uint64_t* min_value, uint64_t* max_value) {
  const size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= count) {
    return;
  }
  const uint64_t ts = timestamps[idx];
  atomicMin(reinterpret_cast<unsigned long long*>(min_value),
            static_cast<unsigned long long>(ts));
  atomicMax(reinterpret_cast<unsigned long long*>(max_value),
            static_cast<unsigned long long>(ts));
}

__global__ void FusionKernel(const CudaPointXYZIT* in_points, size_t point_count,
                             const double* sample_times, int sample_count,
                             const CudaMatrix4f* poses,
                             const CudaMatrix4f* world2base,
                             double measurement_time,
                             CudaPointXYZIT* out_points, uint8_t* keep_flags) {
  const size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= point_count) {
    return;
  }

  const CudaPointXYZIT in = in_points[idx];
  if (!isfinite(in.x) || !isfinite(in.y) || !isfinite(in.z) ||
      fabsf(in.x) > static_cast<float>(kPointInfThreshold) ||
      fabsf(in.y) > static_cast<float>(kPointInfThreshold) ||
      fabsf(in.z) > static_cast<float>(kPointInfThreshold)) {
    keep_flags[idx] = 0;
    return;
  }

  const double point_ts = in.timestamp == 0U
                              ? measurement_time
                              : static_cast<double>(in.timestamp) /
                                    static_cast<double>(kSecondToNano);
  const int pose_idx = ResolveNearestPoseIndex(point_ts, sample_times,
                                               sample_count);
  const float3 world_p = ApplyAffine(&poses[pose_idx], in);

  CudaPointXYZIT world_point = in;
  world_point.x = world_p.x;
  world_point.y = world_p.y;
  world_point.z = world_p.z;
  const float3 base_p = ApplyAffine(world2base, world_point);

  CudaPointXYZIT out = in;
  out.x = base_p.x;
  out.y = base_p.y;
  out.z = base_p.z;
  if (out.timestamp == 0U) {
    out.timestamp =
        static_cast<uint64_t>(measurement_time * static_cast<double>(kSecondToNano));
  }
  out_points[idx] = out;
  keep_flags[idx] = 1;
}

__global__ void EgoFilterKernel(const CudaPointXYZIT* in_points, size_t point_count,
                                CudaEgoFilterParams params,
                                CudaPointXYZIT* out_points,
                                unsigned int* out_count,
                                unsigned int capacity) {
  const size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= point_count) {
    return;
  }

  const CudaPointXYZIT point = in_points[idx];
  bool in_ego_box = false;
  if (params.enable) {
    in_ego_box = point.x < params.forward_x && point.x > params.backward_x &&
                 point.y < params.forward_y && point.y > params.backward_y;
  }
  if (in_ego_box) {
    return;
  }

  const unsigned int write_idx = atomicAdd(out_count, 1U);
  if (write_idx < capacity) {
    out_points[write_idx] = point;
  }
}

__device__ inline uint64_t PackVoxelKey(int vx, int vy, int vz) {
  constexpr int kBias = 1 << 20;
  const uint64_t x = static_cast<uint64_t>(vx + kBias) & 0x1FFFFFULL;
  const uint64_t y = static_cast<uint64_t>(vy + kBias) & 0x1FFFFFULL;
  const uint64_t z = static_cast<uint64_t>(vz + kBias) & 0x1FFFFFULL;
  return x | (y << 21U) | (z << 42U);
}

__device__ inline uint64_t Hash64(uint64_t x) {
  x ^= x >> 33U;
  x *= 0xff51afd7ed558ccdULL;
  x ^= x >> 33U;
  x *= 0xc4ceb9fe1a85ec53ULL;
  x ^= x >> 33U;
  return x;
}

__global__ void InitHashKernel(uint64_t* hash_table, size_t table_size) {
  const size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < table_size) {
    hash_table[idx] = kEmptyVoxel;
  }
}

__global__ void VoxelFilterKernel(const CudaPointXYZIT* in_points,
                                  size_t point_count, float voxel_size,
                                  CudaPointXYZIT* out_points,
                                  unsigned int* out_count,
                                  unsigned int out_capacity,
                                  uint64_t* voxel_hash_table,
                                  size_t hash_table_size) {
  const size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= point_count || voxel_size <= 1e-4f) {
    if (idx < point_count && voxel_size <= 1e-4f) {
      const unsigned int write_idx = atomicAdd(out_count, 1U);
      if (write_idx < out_capacity) {
        out_points[write_idx] = in_points[idx];
      }
    }
    return;
  }

  const CudaPointXYZIT p = in_points[idx];
  const int vx = static_cast<int>(floorf(p.x / voxel_size));
  const int vy = static_cast<int>(floorf(p.y / voxel_size));
  const int vz = static_cast<int>(floorf(p.z / voxel_size));
  const uint64_t key = PackVoxelKey(vx, vy, vz);

  size_t slot = static_cast<size_t>(Hash64(key) % hash_table_size);
  for (size_t probe = 0; probe < hash_table_size; ++probe) {
    const uint64_t prev = atomicCAS(
        reinterpret_cast<unsigned long long*>(&voxel_hash_table[slot]),
        static_cast<unsigned long long>(kEmptyVoxel),
        static_cast<unsigned long long>(key));

    if (prev == kEmptyVoxel) {
      const unsigned int write_idx = atomicAdd(out_count, 1U);
      if (write_idx < out_capacity) {
        out_points[write_idx] = p;
      }
      return;
    }
    if (prev == key) {
      return;
    }
    slot = (slot + 1U) % hash_table_size;
  }
}

bool CheckCuda(cudaError_t error) { return error == cudaSuccess; }

}  // namespace

bool CudaComputeTimestampRange(const uint64_t* host_timestamps, size_t count,
                               int device_id, uint64_t* min_timestamp,
                               uint64_t* max_timestamp) {
  if (host_timestamps == nullptr || min_timestamp == nullptr ||
      max_timestamp == nullptr || count == 0) {
    return false;
  }

  if (!CheckCuda(cudaSetDevice(device_id))) {
    return false;
  }

  uint64_t* d_ts = nullptr;
  uint64_t* d_min = nullptr;
  uint64_t* d_max = nullptr;

  const uint64_t init_min = std::numeric_limits<uint64_t>::max();
  const uint64_t init_max = 0ULL;

  if (!CheckCuda(cudaMalloc(&d_ts, count * sizeof(uint64_t))) ||
      !CheckCuda(cudaMalloc(&d_min, sizeof(uint64_t))) ||
      !CheckCuda(cudaMalloc(&d_max, sizeof(uint64_t)))) {
    cudaFree(d_ts);
    cudaFree(d_min);
    cudaFree(d_max);
    return false;
  }

  bool ok = CheckCuda(cudaMemcpy(d_ts, host_timestamps, count * sizeof(uint64_t),
                                 cudaMemcpyHostToDevice)) &&
            CheckCuda(cudaMemcpy(d_min, &init_min, sizeof(uint64_t),
                                 cudaMemcpyHostToDevice)) &&
            CheckCuda(cudaMemcpy(d_max, &init_max, sizeof(uint64_t),
                                 cudaMemcpyHostToDevice));

  if (ok) {
    const int threads = 256;
    const int blocks = static_cast<int>((count + threads - 1U) / threads);
    TimestampRangeKernel<<<blocks, threads>>>(d_ts, count, d_min, d_max);
    ok = CheckCuda(cudaGetLastError()) && CheckCuda(cudaDeviceSynchronize()) &&
         CheckCuda(cudaMemcpy(min_timestamp, d_min, sizeof(uint64_t),
                              cudaMemcpyDeviceToHost)) &&
         CheckCuda(cudaMemcpy(max_timestamp, d_max, sizeof(uint64_t),
                              cudaMemcpyDeviceToHost));
  }

  cudaFree(d_ts);
  cudaFree(d_min);
  cudaFree(d_max);
  return ok;
}

size_t CudaFuseFrameToBaseLink(const CudaPointXYZIT* host_input_points,
                               size_t input_count,
                               const double* host_sample_times,
                               size_t sample_count,
                               const CudaMatrix4f* host_world_from_sensor,
                               const CudaMatrix4f* host_world2base,
                               double measurement_time,
                               CudaPointXYZIT* host_output_points,
                               size_t output_capacity, int device_id) {
  if (host_input_points == nullptr || host_sample_times == nullptr ||
      host_world_from_sensor == nullptr || host_world2base == nullptr ||
      host_output_points == nullptr || input_count == 0 || sample_count == 0 ||
      output_capacity == 0) {
    return 0;
  }

  if (!CheckCuda(cudaSetDevice(device_id))) {
    return 0;
  }

  CudaPointXYZIT* d_in = nullptr;
  CudaPointXYZIT* d_out = nullptr;
  double* d_times = nullptr;
  CudaMatrix4f* d_poses = nullptr;
  CudaMatrix4f* d_world2base = nullptr;
  uint8_t* d_flags = nullptr;

  if (!CheckCuda(cudaMalloc(&d_in, input_count * sizeof(CudaPointXYZIT))) ||
      !CheckCuda(cudaMalloc(&d_out, input_count * sizeof(CudaPointXYZIT))) ||
      !CheckCuda(cudaMalloc(&d_times, sample_count * sizeof(double))) ||
      !CheckCuda(cudaMalloc(&d_poses, sample_count * sizeof(CudaMatrix4f))) ||
      !CheckCuda(cudaMalloc(&d_world2base, sizeof(CudaMatrix4f))) ||
      !CheckCuda(cudaMalloc(&d_flags, input_count * sizeof(uint8_t)))) {
    cudaFree(d_in);
    cudaFree(d_out);
    cudaFree(d_times);
    cudaFree(d_poses);
    cudaFree(d_world2base);
    cudaFree(d_flags);
    return 0;
  }

  bool ok = CheckCuda(cudaMemcpy(d_in, host_input_points,
                                 input_count * sizeof(CudaPointXYZIT),
                                 cudaMemcpyHostToDevice)) &&
            CheckCuda(cudaMemcpy(d_times, host_sample_times,
                                 sample_count * sizeof(double),
                                 cudaMemcpyHostToDevice)) &&
            CheckCuda(cudaMemcpy(d_poses, host_world_from_sensor,
                                 sample_count * sizeof(CudaMatrix4f),
                                 cudaMemcpyHostToDevice)) &&
            CheckCuda(cudaMemcpy(d_world2base, host_world2base,
                                 sizeof(CudaMatrix4f), cudaMemcpyHostToDevice));

  if (ok) {
    const int threads = 256;
    const int blocks = static_cast<int>((input_count + threads - 1U) / threads);
    FusionKernel<<<blocks, threads>>>(d_in, input_count, d_times,
                                      static_cast<int>(sample_count), d_poses,
                                      d_world2base, measurement_time, d_out,
                                      d_flags);
    ok = CheckCuda(cudaGetLastError()) && CheckCuda(cudaDeviceSynchronize());
  }

  std::vector<CudaPointXYZIT> temp_points(input_count);
  std::vector<uint8_t> temp_flags(input_count, 0U);
  if (ok) {
    ok = CheckCuda(cudaMemcpy(temp_points.data(), d_out,
                              input_count * sizeof(CudaPointXYZIT),
                              cudaMemcpyDeviceToHost)) &&
         CheckCuda(cudaMemcpy(temp_flags.data(), d_flags,
                              input_count * sizeof(uint8_t),
                              cudaMemcpyDeviceToHost));
  }

  size_t out_count = 0;
  if (ok) {
    for (size_t i = 0; i < input_count && out_count < output_capacity; ++i) {
      if (temp_flags[i] != 0U) {
        host_output_points[out_count++] = temp_points[i];
      }
    }
  }

  cudaFree(d_in);
  cudaFree(d_out);
  cudaFree(d_times);
  cudaFree(d_poses);
  cudaFree(d_world2base);
  cudaFree(d_flags);
  return ok ? out_count : 0;
}

size_t CudaApplyEgoFilter(const CudaPointXYZIT* host_input_points,
                          size_t input_count,
                          const CudaEgoFilterParams& params,
                          CudaPointXYZIT* host_output_points,
                          size_t output_capacity, int device_id) {
  if (host_input_points == nullptr || host_output_points == nullptr ||
      output_capacity == 0 || input_count == 0) {
    return 0;
  }

  if (!CheckCuda(cudaSetDevice(device_id))) {
    return 0;
  }

  CudaPointXYZIT* d_in = nullptr;
  CudaPointXYZIT* d_out = nullptr;
  unsigned int* d_count = nullptr;
  unsigned int h_count = 0U;

  if (!CheckCuda(cudaMalloc(&d_in, input_count * sizeof(CudaPointXYZIT))) ||
      !CheckCuda(cudaMalloc(&d_out, output_capacity * sizeof(CudaPointXYZIT))) ||
      !CheckCuda(cudaMalloc(&d_count, sizeof(unsigned int)))) {
    cudaFree(d_in);
    cudaFree(d_out);
    cudaFree(d_count);
    return 0;
  }

  bool ok = CheckCuda(cudaMemcpy(d_in, host_input_points,
                                 input_count * sizeof(CudaPointXYZIT),
                                 cudaMemcpyHostToDevice)) &&
            CheckCuda(cudaMemcpy(d_count, &h_count, sizeof(unsigned int),
                                 cudaMemcpyHostToDevice));

  if (ok) {
    const int threads = 256;
    const int blocks = static_cast<int>((input_count + threads - 1U) / threads);
    EgoFilterKernel<<<blocks, threads>>>(d_in, input_count, params, d_out,
                                         d_count,
                                         static_cast<unsigned int>(output_capacity));
    ok = CheckCuda(cudaGetLastError()) && CheckCuda(cudaDeviceSynchronize()) &&
         CheckCuda(cudaMemcpy(&h_count, d_count, sizeof(unsigned int),
                              cudaMemcpyDeviceToHost));
  }

  const size_t copy_count =
      std::min(static_cast<size_t>(h_count), output_capacity);
  if (ok && copy_count > 0) {
    ok = CheckCuda(cudaMemcpy(host_output_points, d_out,
                              copy_count * sizeof(CudaPointXYZIT),
                              cudaMemcpyDeviceToHost));
  }

  cudaFree(d_in);
  cudaFree(d_out);
  cudaFree(d_count);
  return ok ? copy_count : 0;
}

size_t CudaApplyVoxelDownsample(const CudaPointXYZIT* host_input_points,
                                size_t input_count, float voxel_size,
                                CudaPointXYZIT* host_output_points,
                                size_t output_capacity, int device_id) {
  if (host_input_points == nullptr || host_output_points == nullptr ||
      output_capacity == 0 || input_count == 0) {
    return 0;
  }

  if (!CheckCuda(cudaSetDevice(device_id))) {
    return 0;
  }

  CudaPointXYZIT* d_in = nullptr;
  CudaPointXYZIT* d_out = nullptr;
  unsigned int* d_count = nullptr;
  uint64_t* d_hash = nullptr;
  unsigned int h_count = 0U;
  const size_t hash_size = std::max<size_t>(32, input_count * 2 + 1);

  if (!CheckCuda(cudaMalloc(&d_in, input_count * sizeof(CudaPointXYZIT))) ||
      !CheckCuda(cudaMalloc(&d_out, output_capacity * sizeof(CudaPointXYZIT))) ||
      !CheckCuda(cudaMalloc(&d_count, sizeof(unsigned int))) ||
      !CheckCuda(cudaMalloc(&d_hash, hash_size * sizeof(uint64_t)))) {
    cudaFree(d_in);
    cudaFree(d_out);
    cudaFree(d_count);
    cudaFree(d_hash);
    return 0;
  }

  bool ok = CheckCuda(cudaMemcpy(d_in, host_input_points,
                                 input_count * sizeof(CudaPointXYZIT),
                                 cudaMemcpyHostToDevice)) &&
            CheckCuda(cudaMemcpy(d_count, &h_count, sizeof(unsigned int),
                                 cudaMemcpyHostToDevice));

  if (ok) {
    const int threads = 256;
    const int init_blocks = static_cast<int>((hash_size + threads - 1U) / threads);
    InitHashKernel<<<init_blocks, threads>>>(d_hash, hash_size);
    ok = CheckCuda(cudaGetLastError()) && CheckCuda(cudaDeviceSynchronize());
  }

  if (ok) {
    const int threads = 256;
    const int blocks = static_cast<int>((input_count + threads - 1U) / threads);
    VoxelFilterKernel<<<blocks, threads>>>(d_in, input_count, voxel_size, d_out,
                                           d_count,
                                           static_cast<unsigned int>(output_capacity),
                                           d_hash, hash_size);
    ok = CheckCuda(cudaGetLastError()) && CheckCuda(cudaDeviceSynchronize()) &&
         CheckCuda(cudaMemcpy(&h_count, d_count, sizeof(unsigned int),
                              cudaMemcpyDeviceToHost));
  }

  const size_t copy_count =
      std::min(static_cast<size_t>(h_count), output_capacity);
  if (ok && copy_count > 0) {
    ok = CheckCuda(cudaMemcpy(host_output_points, d_out,
                              copy_count * sizeof(CudaPointXYZIT),
                              cudaMemcpyDeviceToHost));
  }

  cudaFree(d_in);
  cudaFree(d_out);
  cudaFree(d_count);
  cudaFree(d_hash);
  return ok ? copy_count : 0;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
