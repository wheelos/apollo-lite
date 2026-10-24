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
#include <vector>

#include "modules/bevfusion/proto/bevfusion.pb.h"

#include "modules/bevfusion/inference/bevfusion_runtime_api.h"
#include "modules/bevfusion/types/bevfusion_input.h"

namespace apollo {
namespace bevfusion {

struct BevFusionRuntimeDetection {
  float center_x = 0.0F;
  float center_y = 0.0F;
  float center_z = 0.0F;
  float width = 0.0F;
  float length = 0.0F;
  float height = 0.0F;
  float velocity_x = 0.0F;
  float velocity_y = 0.0F;
  float yaw = 0.0F;
  float score = 0.0F;
  int32_t class_id = 0;
};

struct BevFusionInferenceResult {
  std::uint64_t timestamp_ns = 0;
  std::size_t input_point_count = 0;
  double inference_ms = 0.0;
  std::vector<BevFusionRuntimeDetection> detections;
};

class BevFusionRuntimeBackend {
 public:
  BevFusionRuntimeBackend() = default;
  ~BevFusionRuntimeBackend();

  BevFusionRuntimeBackend(const BevFusionRuntimeBackend&) = delete;
  BevFusionRuntimeBackend& operator=(const BevFusionRuntimeBackend&) = delete;

  bool Init(const BevFusionModelConfig& config,
            std::uint64_t max_timestamp_skew_ns, std::string* error);
  bool Infer(const BevFusionInputView& input, BevFusionInferenceResult* result,
             std::string* error);

 private:
  using AbiVersionFunction = uint32_t (*)();
  using CreateFunction = int (*)(const ApolloBevFusionRuntimeConfigV1*, void**,
                                 char*, size_t);
  using DestroyFunction = void (*)(void*);
  using InferFunction = int (*)(void*, const ApolloBevFusionInputV1*,
                                ApolloBevFusionDetectionV1*, size_t, size_t*,
                                char*, size_t);

  bool LoadSymbol(const char* symbol, void** function, std::string* error);
  void Reset();

  void* library_ = nullptr;
  void* runtime_ = nullptr;
  AbiVersionFunction abi_version_ = nullptr;
  CreateFunction create_ = nullptr;
  DestroyFunction destroy_ = nullptr;
  InferFunction infer_ = nullptr;
  InputLimits limits_;
  BevFusionModelConfig config_;
};

}  // namespace bevfusion
}  // namespace apollo
