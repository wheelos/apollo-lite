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

#include <dlfcn.h>

#include <array>
#include <cstdlib>
#include <cstring>
#include <string>

#include "gtest/gtest.h"

#include "modules/bevfusion/inference/bevfusion_runtime_api.h"

namespace {

TEST(BevFusionRuntimeAbiTest, ExportsVersionedCFunctions) {
  const char* runtime_path = std::getenv("BEVFUSION_RUNTIME_DSO");
  ASSERT_NE(runtime_path, nullptr);
  void* handle = dlopen(runtime_path, RTLD_NOW | RTLD_LOCAL);
  ASSERT_NE(handle, nullptr) << dlerror();
  const std::array<const char*, 4> symbols = {
      "apollo_bevfusion_runtime_abi_version", "apollo_bevfusion_runtime_create",
      "apollo_bevfusion_runtime_destroy", "apollo_bevfusion_runtime_infer"};
  for (const char* symbol : symbols) {
    dlerror();
    void* address = dlsym(handle, symbol);
    const char* error = dlerror();
    EXPECT_NE(address, nullptr) << "Missing runtime ABI symbol " << symbol
                                << ": " << (error == nullptr ? "" : error);
  }
  using AbiVersionFunction = std::uint32_t (*)();
  using CreateFunction = int (*)(const ApolloBevFusionRuntimeConfigV1*, void**,
                                 char*, std::size_t);
  auto abi_version = reinterpret_cast<AbiVersionFunction>(
      dlsym(handle, "apollo_bevfusion_runtime_abi_version"));
  auto create = reinterpret_cast<CreateFunction>(
      dlsym(handle, "apollo_bevfusion_runtime_create"));
  ASSERT_NE(abi_version, nullptr);
  ASSERT_NE(create, nullptr);
  EXPECT_EQ(abi_version(), APOLLO_BEVFUSION_RUNTIME_ABI_VERSION);

  ApolloBevFusionRuntimeConfigV1 config{};
  config.struct_size = sizeof(config);
  config.abi_version = APOLLO_BEVFUSION_RUNTIME_ABI_VERSION;
  config.max_input_point_count = 300000;
  config.max_timestamp_skew_ns = 1000000;
  config.confidence_threshold = 0.12F;
  config.camera_backbone_engine_path = "missing-camera.plan";
  config.lidar_scn_onnx_path = "missing-scn.onnx";
  config.camera_vtransform_engine_path = "missing-vtransform.plan";
  config.transfusion_engine_path = "missing-fuser.plan";
  config.bbox_engine_path = "missing-bbox.plan";
  void* runtime = reinterpret_cast<void*>(1);
  std::array<char, 512> error{};
  EXPECT_NE(create(&config, &runtime, error.data(), error.size()), 0);
  EXPECT_EQ(runtime, nullptr);
  EXPECT_NE(std::string(error.data()).find("missing or empty"),
            std::string::npos);
  EXPECT_EQ(dlclose(handle), 0);
}

}  // namespace
