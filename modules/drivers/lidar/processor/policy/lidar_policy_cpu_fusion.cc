// Copyright 2026 WheelOS All Rights Reserved.
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

#include "cyber/cyber.h"
#include "modules/drivers/lidar/processor/policy/cpu_lidar_policy.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"

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
    const std::vector<std::vector<Eigen::Affine3d>>& frames_motion_poses,
    const std::vector<std::vector<double>>& frames_motion_times,
    PointCloudBuffer* output_buffer) {
  PointXYZIT* output_points = GetHostPoints(output_buffer);
  if (output_points == nullptr || tf_buffer_ == nullptr ||
      frames.size() != frames_motion_poses.size() ||
      frames.size() != frames_motion_times.size()) {
    return false;
  }
  (void)reference_timestamp_sec;

  output_buffer->unfiltered_valid_count = 0;
  output_buffer->prefiltered_ego_count = 0;
  const bool apply_ego_filter = config_.enable_ego_query_filter();
  output_buffer->ego_filter_applied = apply_ego_filter;
  size_t write_idx = 0;
  for (size_t frame_idx = 0; frame_idx < frames.size(); ++frame_idx) {
    const auto& frame = frames[frame_idx];
    if (frame.point_cloud == nullptr) {
      continue;
    }

    const auto& sample_times = frames_motion_times[frame_idx];
    const auto& poses = frames_motion_poses[frame_idx];
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
      const Eigen::Affine3d base_from_sensor = map2base_ref * poses.front();
      const Eigen::Matrix3d rotation = base_from_sensor.linear();
      const Eigen::Vector3d translation = base_from_sensor.translation();
      const bool fast_timestamp_path =
          frame.all_points_have_timestamps && frame.timestamp_offset_ns == 0;

      const auto transform_static_point =
          [&](const PointXYZIT& point, uint64_t timestamp) {
            if (output_buffer->unfiltered_valid_count >=
                output_buffer->capacity) {
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
            const double x = point.x();
            const double y = point.y();
            const double z = point.z();
            const float transformed_x = static_cast<float>(
                rotation(0, 0) * x + rotation(0, 1) * y +
                rotation(0, 2) * z + translation.x());
            const float transformed_y = static_cast<float>(
                rotation(1, 0) * x + rotation(1, 1) * y +
                rotation(1, 2) * z + translation.y());
            if (apply_ego_filter &&
                transformed_x < config_.ego_box_forward_x() &&
                transformed_x > config_.ego_box_backward_x() &&
                transformed_y < config_.ego_box_forward_y() &&
                transformed_y > config_.ego_box_backward_y()) {
              ++output_buffer->prefiltered_ego_count;
              return true;
            }

            PointXYZIT* output_point = &output_points[write_idx++];
            output_point->set_x(transformed_x);
            output_point->set_y(transformed_y);
            output_point->set_z(static_cast<float>(
                rotation(2, 0) * x + rotation(2, 1) * y +
                rotation(2, 2) * z + translation.z()));
            output_point->set_intensity(point.intensity());
            output_point->set_timestamp(timestamp);
            return true;
          };

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
          const uint64_t timestamp = static_cast<uint64_t>(
              static_cast<int64_t>(raw_timestamp) +
              frame.timestamp_offset_ns);
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
