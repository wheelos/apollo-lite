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

#include <memory>
#include <string>

#include "modules/prediction/evaluator/vehicle/hivt_scene.h"

namespace apollo {
namespace prediction {

class HiVTTensorRtExecutor final {
 public:
  HiVTTensorRtExecutor();
  ~HiVTTensorRtExecutor();

  HiVTTensorRtExecutor(const HiVTTensorRtExecutor&) = delete;
  HiVTTensorRtExecutor& operator=(const HiVTTensorRtExecutor&) = delete;

  bool Init(const std::string& engine_path, int device_id);
  bool Run(const HiVTSceneInput& input, HiVTSceneOutput* output);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace prediction
}  // namespace apollo
