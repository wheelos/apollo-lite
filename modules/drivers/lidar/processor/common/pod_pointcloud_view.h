// Copyright 2026 The WheelOS Authors. All Rights Reserved.
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

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace drivers {
namespace lidar {

struct PointXYZITPod {
  float x_ = 0.0F;
  float y_ = 0.0F;
  float z_ = 0.0F;
  uint32_t intensity_ = 0U;
  uint64_t timestamp_ = 0U;

  float x() const { return x_; }
  float y() const { return y_; }
  float z() const { return z_; }
  uint32_t intensity() const { return intensity_; }
  uint64_t timestamp() const { return timestamp_; }

  void set_x(float value) { x_ = value; }
  void set_y(float value) { y_ = value; }
  void set_z(float value) { z_ = value; }
  void set_intensity(uint32_t value) { intensity_ = value; }
  void set_timestamp(uint64_t value) { timestamp_ = value; }
};

static_assert(std::is_standard_layout<PointXYZITPod>::value);
static_assert(std::is_trivially_copyable<PointXYZITPod>::value);
static_assert(sizeof(PointXYZITPod) == 24U);
static_assert(offsetof(PointXYZITPod, x_) == 0U);
static_assert(offsetof(PointXYZITPod, y_) == 4U);
static_assert(offsetof(PointXYZITPod, z_) == 8U);
static_assert(offsetof(PointXYZITPod, intensity_) == 12U);
static_assert(offsetof(PointXYZITPod, timestamp_) == 16U);

class PointCloudView {
 public:
  using Message = ::apollo::drivers::PointCloud;

  explicit PointCloudView(std::shared_ptr<const Message> message);

  bool valid() const { return message_ != nullptr; }
  bool has_header() const { return valid() && message_->has_header(); }
  const ::apollo::common::Header& header() const;
  const std::string& frame_id() const;
  bool is_dense() const { return valid() && message_->is_dense(); }
  double measurement_time() const {
    return valid() ? message_->measurement_time() : 0.0;
  }
  uint32_t width() const { return valid() ? message_->width() : 0U; }
  uint32_t height() const { return valid() ? message_->height() : 0U; }
  int point_size() const { return static_cast<int>(points_.size()); }
  const std::vector<PointXYZITPod>& point() const { return points_; }
  const PointXYZITPod& point(int index) const {
    return points_[static_cast<size_t>(index)];
  }
  const PointXYZITPod* raw_points_data() const {
    return points_.empty() ? nullptr : points_.data();
  }
  const std::shared_ptr<const Message>& message() const { return message_; }

 private:
  std::shared_ptr<const Message> message_;
  std::vector<PointXYZITPod> points_;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
