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

#include "modules/drivers/lidar/processor/common/pod_pointcloud_view.h"

#include <utility>

namespace apollo {
namespace drivers {
namespace lidar {

PointCloudView::PointCloudView(std::shared_ptr<const Message> message)
    : message_(std::move(message)) {
  if (message_ == nullptr) {
    return;
  }
  points_.reserve(static_cast<size_t>(message_->point_size()));
  for (const auto& point : message_->point()) {
    PointXYZITPod pod_point;
    pod_point.set_x(point.x());
    pod_point.set_y(point.y());
    pod_point.set_z(point.z());
    pod_point.set_intensity(point.intensity());
    pod_point.set_timestamp(point.timestamp());
    points_.push_back(pod_point);
  }
}

const ::apollo::common::Header& PointCloudView::header() const {
  static const ::apollo::common::Header empty_header;
  return has_header() ? message_->header() : empty_header;
}

const std::string& PointCloudView::frame_id() const {
  static const std::string empty_frame_id;
  return valid() ? message_->frame_id() : empty_frame_id;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
