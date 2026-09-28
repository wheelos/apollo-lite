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

#include "cyber/common/log.h"
#include "modules/prediction/evaluator/vehicle/hivt_tensorrt_executor.h"

namespace apollo {
namespace prediction {

class HiVTTensorRtExecutor::Impl {};

HiVTTensorRtExecutor::HiVTTensorRtExecutor()
    : impl_(std::make_unique<Impl>()) {}

HiVTTensorRtExecutor::~HiVTTensorRtExecutor() = default;

bool HiVTTensorRtExecutor::Init(const std::string&, int) {
  AERROR << "HiVT TensorRT inference requires a GPU build";
  return false;
}

bool HiVTTensorRtExecutor::Run(const HiVTSceneInput&, HiVTSceneOutput*) {
  AERROR << "HiVT TensorRT inference requires a GPU build";
  return false;
}

}  // namespace prediction
}  // namespace apollo
