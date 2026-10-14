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

#include <string>

#include "modules/lidar_object_detection/inference/centerpoint_backend.h"
#include "modules/lidar_object_detection/types/model_input.h"

namespace apollo {
namespace lidar_object_detection {

class CenterPointRunner {
 public:
  explicit CenterPointRunner(CenterPointBackend* backend)
      : backend_(backend) {}

  bool Init(const NvidiaCenterPointConfig& config, std::string* error);

  bool Run(const apollo::drivers::PointCloud& cloud,
           const std::string& source_path, const std::string& sensor_name,
           CenterPointFrameResult* result, std::string* error) const;

 private:
  CenterPointBackend* backend_ = nullptr;
  ModelInputBuilder input_builder_;
  NvidiaCenterPointConfig config_;
  bool initialized_ = false;
};

}  // namespace lidar_object_detection
}  // namespace apollo
