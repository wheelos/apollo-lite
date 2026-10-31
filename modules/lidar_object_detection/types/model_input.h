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

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "modules/lidar_object_detection/proto/lidar_object_detection.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace lidar_object_detection {

struct ModelInput {
  std::vector<float> points;
  std::vector<uint32_t> source_point_indices;

  std::size_t point_count() const { return source_point_indices.size(); }
};

class ModelInputBuilder {
 public:
  bool Init(const CenterPointModelConfig& config, std::string* error);

  bool Build(const apollo::drivers::PointCloud& cloud, ModelInput* input,
             std::string* error) const;

 private:
  std::vector<PointFeature> features_;
  bool initialized_ = false;
};

}  // namespace lidar_object_detection
}  // namespace apollo
