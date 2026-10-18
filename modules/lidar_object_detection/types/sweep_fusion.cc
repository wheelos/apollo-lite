// Copyright 2026 WheelOS. All Rights Reserved.
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

#include "modules/lidar_object_detection/types/sweep_fusion.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>

namespace apollo {
namespace lidar_object_detection {

namespace {

constexpr double kRotationTolerance = 1.0e-3;
constexpr double kClosePointRangeMeters = 1.0;
constexpr double kSecondsToNanoseconds = 1.0e9;

void SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

bool IsValidPose(const Eigen::Affine3d& pose) {
  if (!pose.matrix().allFinite()) {
    return false;
  }
  const Eigen::Matrix3d& rotation = pose.linear();
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  return (rotation.transpose() * rotation - identity).norm() <=
             kRotationTolerance &&
         std::abs(rotation.determinant() - 1.0) <= kRotationTolerance;
}

}  // namespace

bool PointCloudSweepFusion::Init(std::size_t sweep_count,
                                 std::size_t max_point_count,
                                 std::string* error) {
  if (sweep_count == 0U) {
    SetError("CenterPoint sweep count must be greater than zero", error);
    return false;
  }
  if (max_point_count == 0U ||
      max_point_count >
          static_cast<std::size_t>(std::numeric_limits<uint32_t>::max())) {
    SetError("CenterPoint maximum point count is invalid", error);
    return false;
  }
  Reset();
  sweep_count_ = sweep_count;
  max_point_count_ = max_point_count;
  initialized_ = true;
  return true;
}

bool PointCloudSweepFusion::AddFrame(
    const std::shared_ptr<const apollo::drivers::PointCloud>& cloud,
    const Eigen::Affine3d& sensor_to_world,
    apollo::drivers::PointCloud* fused_cloud, std::string* error) {
  if (!initialized_) {
    SetError("CenterPoint sweep fusion is not initialized", error);
    return false;
  }
  if (cloud == nullptr || fused_cloud == nullptr) {
    SetError("CenterPoint sweep fusion received a null input or output", error);
    return false;
  }
  const double current_time = cloud->measurement_time();
  if (!std::isfinite(current_time)) {
    SetError("PointCloud measurement_time is not finite", error);
    return false;
  }
  if (!IsValidPose(sensor_to_world)) {
    SetError("PointCloud sensor-to-world pose is invalid", error);
    return false;
  }
  if (has_last_measurement_time_ && current_time <= last_measurement_time_) {
    SetError("PointCloud measurement_time is not strictly increasing", error);
    return false;
  }

  SweepFrame current_frame;
  current_frame.cloud = cloud;
  current_frame.sensor_to_world = sensor_to_world;
  current_frame.measurement_time = current_time;

  apollo::drivers::PointCloud output;
  if (cloud->has_header()) {
    *output.mutable_header() = cloud->header();
  }
  if (cloud->has_frame_id()) {
    output.set_frame_id(cloud->frame_id());
  }
  output.set_is_dense(true);
  output.set_measurement_time(0.0);
  output.set_height(1U);
  output.mutable_point()->Reserve(static_cast<int>(std::min(
      max_point_count_, static_cast<std::size_t>(cloud->point_size()))));

  const Eigen::Affine3d world_to_current =
      sensor_to_world.inverse(Eigen::Affine);
  for (std::size_t sweep_index = 0U; sweep_index < sweep_count_;
       ++sweep_index) {
    const bool is_current_sweep = sweep_index == 0U;
    const SweepFrame* sweep = &current_frame;
    if (!is_current_sweep && !frames_.empty()) {
      const std::size_t frame_index =
          sweep_index <= frames_.size() ? frames_.size() - sweep_index : 0U;
      sweep = &frames_[frame_index];
    }
    const Eigen::Affine3d sweep_to_current =
        world_to_current * sweep->sensor_to_world;
    const double time_lag =
        current_frame.measurement_time - sweep->measurement_time;
    if (!std::isfinite(time_lag) || time_lag < 0.0) {
      SetError("CenterPoint sweep time lag is invalid", error);
      return false;
    }
    const long double lag_ns =
        static_cast<long double>(time_lag) * kSecondsToNanoseconds;
    if (lag_ns >
        static_cast<long double>(std::numeric_limits<uint64_t>::max())) {
      SetError("CenterPoint sweep time lag exceeds uint64 nanoseconds", error);
      return false;
    }
    const uint64_t time_lag_ns = static_cast<uint64_t>(std::round(lag_ns));

    for (const auto& point : sweep->cloud->point()) {
      if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
          !std::isfinite(point.z())) {
        continue;
      }
      if (!is_current_sweep && std::abs(point.x()) < kClosePointRangeMeters &&
          std::abs(point.y()) < kClosePointRangeMeters) {
        continue;
      }
      const Eigen::Vector3d transformed =
          sweep_to_current * Eigen::Vector3d(point.x(), point.y(), point.z());
      if (!transformed.allFinite()) {
        continue;
      }
      if (static_cast<std::size_t>(output.point_size()) >= max_point_count_) {
        SetError("Fused PointCloud exceeds the CenterPoint point limit", error);
        return false;
      }
      auto* fused_point = output.add_point();
      fused_point->set_x(static_cast<float>(transformed.x()));
      fused_point->set_y(static_cast<float>(transformed.y()));
      fused_point->set_z(static_cast<float>(transformed.z()));
      fused_point->set_intensity(point.intensity());
      fused_point->set_timestamp(is_current_sweep ? 0U : time_lag_ns);
    }
  }

  if (output.point_size() == 0) {
    SetError("Fused PointCloud contains no finite points", error);
    return false;
  }
  output.set_width(static_cast<uint32_t>(output.point_size()));
  *fused_cloud = std::move(output);

  frames_.push_back(std::move(current_frame));
  while (frames_.size() >= sweep_count_) {
    frames_.pop_front();
  }
  last_measurement_time_ = current_time;
  has_last_measurement_time_ = true;
  return true;
}

void PointCloudSweepFusion::Reset() {
  frames_.clear();
  sweep_count_ = 0U;
  max_point_count_ = 0U;
  last_measurement_time_ = 0.0;
  has_last_measurement_time_ = false;
  initialized_ = false;
}

}  // namespace lidar_object_detection
}  // namespace apollo
