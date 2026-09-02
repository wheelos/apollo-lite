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
#include <vector>

#include "modules/camera_semantic_segmentation/inference/segformer.h"

namespace apollo {
namespace camera_semantic_segmentation {

class TensorRtSegFormerExecutor final : public SegFormerExecutor {
 public:
  TensorRtSegFormerExecutor();
  ~TensorRtSegFormerExecutor() override;

  TensorRtSegFormerExecutor(const TensorRtSegFormerExecutor&) = delete;
  TensorRtSegFormerExecutor& operator=(
      const TensorRtSegFormerExecutor&) = delete;

  bool Init(const SegFormerModelOptions& options) override;
  bool Run(const std::vector<float>& input, SegFormerTensor* output) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace camera_semantic_segmentation
}  // namespace apollo
