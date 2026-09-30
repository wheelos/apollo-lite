// Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <vector>

#if APOLLO_LIDAR_USE_NEON && (defined(__ARM_NEON) || defined(__ARM_NEON__))
#include <arm_neon.h>
#define APOLLO_LIDAR_NEON_ENABLED 1
#else
#define APOLLO_LIDAR_NEON_ENABLED 0
#endif

#include "cyber/cyber.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_cpu.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool CpuLidarFusionPolicy::Init(const LidarUnifiedComponentConfig& config,
                                apollo::transform::BufferInterface* tf_buffer) {
  config_ = config;
  tf_buffer_ = tf_buffer;
  return tf_buffer_ != nullptr;
}

bool CpuLidarFusionPolicy::FuseToBaseLink(
    double reference_timestamp_sec, const Eigen::Affine3d& map2base_ref,
    const std::vector<SensorFrameContext>& frames,
    PointCloudBuffer* output_buffer) {
  PointXYZIT* output_points = GetHostPoints(output_buffer);
  if (output_points == nullptr || tf_buffer_ == nullptr) {
    return false;
  }
  (void)reference_timestamp_sec;

  output_buffer->unfiltered_valid_count = 0;
  output_buffer->prefiltered_ego_count = 0;
  const bool apply_ego_filter = config_.enable_ego_query_filter();
  output_buffer->ego_filter_applied = apply_ego_filter;
  size_t write_idx = 0;
  for (const auto& frame : frames) {
    if (frame.point_cloud == nullptr) {
      continue;
    }

    const auto& sample_times = frame.motion_sample_times;
    const auto& poses = frame.motion_poses;
    if (sample_times.empty() || poses.empty() ||
        sample_times.size() != poses.size()) {
      if (frame.is_primary) {
        return false;
      }
      AWARN << "Skip auxiliary frame due to invalid motion compensation data: "
            << frame.sensor_id;
      continue;
    }

    if (poses.size() == 1U) {
      const Eigen::Affine3d base_from_sensor_d = map2base_ref * poses.front();
      const Eigen::Matrix3f rotation =
          base_from_sensor_d.linear().cast<float>();
      const Eigen::Vector3f translation =
          base_from_sensor_d.translation().cast<float>();
      const bool fast_timestamp_path =
          frame.all_points_have_timestamps && frame.timestamp_offset_ns == 0;
      const float ego_forward_x =
          static_cast<float>(config_.ego_box_forward_x());
      const float ego_backward_x =
          static_cast<float>(config_.ego_box_backward_x());
      const float ego_forward_y =
          static_cast<float>(config_.ego_box_forward_y());
      const float ego_backward_y =
          static_cast<float>(config_.ego_box_backward_y());

      const float r00 = rotation(0, 0);
      const float r01 = rotation(0, 1);
      const float r02 = rotation(0, 2);
      const float r10 = rotation(1, 0);
      const float r11 = rotation(1, 1);
      const float r12 = rotation(1, 2);
      const float r20 = rotation(2, 0);
      const float r21 = rotation(2, 1);
      const float r22 = rotation(2, 2);
      const float tx = translation.x();
      const float ty = translation.y();
      const float tz = translation.z();


      const auto transform_static_point = [&](const PointXYZITPod& point,
                                              uint64_t timestamp) {
        if (output_buffer->unfiltered_valid_count >= output_buffer->capacity) {
          return false;
        }
        if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
            !std::isfinite(point.z()) ||
            std::fabs(point.x()) > kPointInfThreshold ||
            std::fabs(point.y()) > kPointInfThreshold ||
            std::fabs(point.z()) > kPointInfThreshold) {
          return true;
        }

        ++output_buffer->unfiltered_valid_count;
        const float x = point.x();
        const float y = point.y();
        const float z = point.z();
        const float transformed_x = r00 * x + r01 * y + r02 * z + tx;
        const float transformed_y = r10 * x + r11 * y + r12 * z + ty;
        if (apply_ego_filter && transformed_x < ego_forward_x &&
            transformed_x > ego_backward_x && transformed_y < ego_forward_y &&
            transformed_y > ego_backward_y) {
          ++output_buffer->prefiltered_ego_count;
          return true;
        }

        PointXYZIT* output_point = &output_points[write_idx++];
        output_point->set_x(transformed_x);
        output_point->set_y(transformed_y);
        output_point->set_z(r20 * x + r21 * y + r22 * z + tz);
        output_point->set_intensity(point.intensity());
        output_point->set_timestamp(timestamp);
        return true;
      };

#if APOLLO_LIDAR_NEON_ENABLED
      const auto* raw_points = frame.point_cloud->raw_points_data();
      const size_t point_count =
          static_cast<size_t>(frame.point_cloud->point_size());
      if (raw_points != nullptr && fast_timestamp_path && point_count >= 4U) {
        const float32x4_t vr00 = vdupq_n_f32(r00);
        const float32x4_t vr01 = vdupq_n_f32(r01);
        const float32x4_t vr02 = vdupq_n_f32(r02);
        const float32x4_t vr10 = vdupq_n_f32(r10);
        const float32x4_t vr11 = vdupq_n_f32(r11);
        const float32x4_t vr12 = vdupq_n_f32(r12);
        const float32x4_t vr20 = vdupq_n_f32(r20);
        const float32x4_t vr21 = vdupq_n_f32(r21);
        const float32x4_t vr22 = vdupq_n_f32(r22);
        const float32x4_t vtx = vdupq_n_f32(tx);
        const float32x4_t vty = vdupq_n_f32(ty);
        const float32x4_t vtz = vdupq_n_f32(tz);
        const float32x4_t vforward_x = vdupq_n_f32(ego_forward_x);
        const float32x4_t vbackward_x = vdupq_n_f32(ego_backward_x);
        const float32x4_t vforward_y = vdupq_n_f32(ego_forward_y);
        const float32x4_t vbackward_y = vdupq_n_f32(ego_backward_y);
        const float32x4_t vinf = vdupq_n_f32(kPointInfThreshold);

        size_t point_index = 0U;
        for (; point_index + 4U <= point_count &&
               output_buffer->unfiltered_valid_count + 4U <=
                   output_buffer->capacity;
             point_index += 4U) {
          const auto* p0 = &raw_points[point_index];
          const auto* p1 = &raw_points[point_index + 1U];
          const auto* p2 = &raw_points[point_index + 2U];
          const auto* p3 = &raw_points[point_index + 3U];
          const float xs[4] = {p0->x(), p1->x(), p2->x(), p3->x()};
          const float ys[4] = {p0->y(), p1->y(), p2->y(), p3->y()};
          const float zs[4] = {p0->z(), p1->z(), p2->z(), p3->z()};
          const float32x4_t vx = vld1q_f32(xs);
          const float32x4_t vy = vld1q_f32(ys);
          const float32x4_t vz = vld1q_f32(zs);
          const float32x4_t out_x = vmlaq_f32(
              vmlaq_f32(vmlaq_f32(vtx, vr00, vx), vr01, vy), vr02, vz);
          const float32x4_t out_y = vmlaq_f32(
              vmlaq_f32(vmlaq_f32(vty, vr10, vx), vr11, vy), vr12, vz);
          const float32x4_t out_z = vmlaq_f32(
              vmlaq_f32(vmlaq_f32(vtz, vr20, vx), vr21, vy), vr22, vz);
          const uint32x4_t invalid =
              vorrq_u32(vorrq_u32(vcgtq_f32(vabsq_f32(vx), vinf),
                                  vcgtq_f32(vabsq_f32(vy), vinf)),
                        vcgtq_f32(vabsq_f32(vz), vinf));
          const uint32x4_t in_ego_box =
              vandq_u32(vandq_u32(vcltq_f32(out_x, vforward_x),
                                  vcgtq_f32(out_x, vbackward_x)),
                        vandq_u32(vcltq_f32(out_y, vforward_y),
                                  vcgtq_f32(out_y, vbackward_y)));

          float transformed_x[4];
          float transformed_y[4];
          float transformed_z[4];
          uint32_t invalid_flags[4];
          uint32_t ego_flags[4];
          vst1q_f32(transformed_x, out_x);
          vst1q_f32(transformed_y, out_y);
          vst1q_f32(transformed_z, out_z);
          vst1q_u32(invalid_flags, invalid);
          vst1q_u32(ego_flags, in_ego_box);
          const PointXYZITPod* points[4] = {p0, p1, p2, p3};
          for (size_t lane = 0U; lane < 4U; ++lane) {
            if (invalid_flags[lane] != 0U ||
                !std::isfinite(transformed_x[lane]) ||
                !std::isfinite(transformed_y[lane]) ||
                !std::isfinite(transformed_z[lane])) {
              continue;
            }
            ++output_buffer->unfiltered_valid_count;
            if (apply_ego_filter && ego_flags[lane] != 0U) {
              ++output_buffer->prefiltered_ego_count;
              continue;
            }
            PointXYZIT* output_point = &output_points[write_idx++];
            output_point->set_x(transformed_x[lane]);
            output_point->set_y(transformed_y[lane]);
            output_point->set_z(transformed_z[lane]);
            output_point->set_intensity(points[lane]->intensity());
            output_point->set_timestamp(points[lane]->timestamp());
          }
        }

        for (; point_index < point_count; ++point_index) {
          if (!transform_static_point(raw_points[point_index],
                                      raw_points[point_index].timestamp())) {
            AWARN << "Output point buffer is full, truncating fused cloud at "
                  << output_buffer->unfiltered_valid_count << " points";
            output_buffer->valid_count = write_idx;
            return true;
          }
        }
        continue;
      }
#endif

      if (fast_timestamp_path) {
        for (const auto& point : frame.point_cloud->point()) {
          if (!transform_static_point(point, point.timestamp())) {
            AWARN << "Output point buffer is full, truncating fused cloud at "
                  << output_buffer->unfiltered_valid_count << " points";
            output_buffer->valid_count = write_idx;
            return true;
          }
        }
      } else {
        for (const auto& point : frame.point_cloud->point()) {
          const uint64_t raw_timestamp = point.timestamp() == 0U
                                             ? frame.fallback_timestamp_ns
                                             : point.timestamp();
          uint64_t timestamp = 0;
          if (!AddTimestampOffset(raw_timestamp, frame.timestamp_offset_ns,
                                  &timestamp)) {
            continue;
          }
          if (!transform_static_point(point, timestamp)) {
            AWARN << "Output point buffer is full, truncating fused cloud at "
                  << output_buffer->unfiltered_valid_count << " points";
            output_buffer->valid_count = write_idx;
            return true;
          }
        }
      }
      continue;
    }

    std::vector<Eigen::Affine3d> base_from_sensor_poses;
    base_from_sensor_poses.reserve(poses.size());
    for (const auto& pose : poses) {
      base_from_sensor_poses.push_back(map2base_ref * pose);
    }
    UniformPoseInterpolation uniform_interpolation;
    const bool use_uniform_interpolation = BuildUniformPoseInterpolation(
        sample_times, base_from_sensor_poses, &uniform_interpolation);

    for (const auto& point : frame.point_cloud->point()) {
      if (output_buffer->unfiltered_valid_count >= output_buffer->capacity) {
        AWARN << "Output point buffer is full, truncating fused cloud at "
              << output_buffer->unfiltered_valid_count << " points";
        output_buffer->valid_count = write_idx;
        return true;
      }

      PointXYZIT transformed_point;
      const bool transformed =
          use_uniform_interpolation
              ? TransformPointWithUniformInterpolatedPoses(
                    point, frame.fallback_timestamp_ns,
                    frame.timestamp_offset_ns, sample_times,
                    base_from_sensor_poses, uniform_interpolation,
                    &transformed_point)
              : TransformPointWithInterpolatedPoses(
                    point, frame.fallback_timestamp_ns,
                    frame.timestamp_offset_ns, sample_times,
                    base_from_sensor_poses, &transformed_point);
      if (!transformed) {
        continue;
      }
      ++output_buffer->unfiltered_valid_count;
      if (apply_ego_filter &&
          transformed_point.x() < config_.ego_box_forward_x() &&
          transformed_point.x() > config_.ego_box_backward_x() &&
          transformed_point.y() < config_.ego_box_forward_y() &&
          transformed_point.y() > config_.ego_box_backward_y()) {
        ++output_buffer->prefiltered_ego_count;
        continue;
      }
      output_points[write_idx++] = transformed_point;
    }
  }

  output_buffer->valid_count = write_idx;
  return true;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
