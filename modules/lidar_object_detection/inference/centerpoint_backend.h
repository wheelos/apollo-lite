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
#include <vector>

#include "modules/lidar_object_detection/proto/lidar_object_detection.pb.h"
#include "modules/lidar_object_detection/types/model_input.h"

namespace apollo {
namespace lidar_object_detection {

struct InferenceTimings {
  double voxelization_ms = 0.0;
  double sparse_backbone_ms = 0.0;
  double rpn_centerhead_ms = 0.0;
  double decode_nms_ms = 0.0;
  double total_inference_ms = 0.0;
};

class CenterPointBackend {
 public:
  virtual ~CenterPointBackend() = default;

  virtual bool Init(const NvidiaCenterPointConfig& config,
                    std::string* error) = 0;
  virtual bool Infer(const ModelInput& input,
                     std::vector<CenterPointDetection>* detections,
                     InferenceTimings* timings, std::string* error) = 0;
};

}  // namespace lidar_object_detection
}  // namespace apollo
