/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <string>

#include "Eigen/Geometry"

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace localization {
namespace common {

bool LookupStaticTransform(const std::string& target_frame_id,
                           const std::string& source_frame_id,
                           Eigen::Affine3d* transform,
                           float timeout_sec = 1.0f);

std::string GetPointCloudFrameId(const drivers::PointCloud& point_cloud);

}  // namespace common
}  // namespace localization
}  // namespace apollo