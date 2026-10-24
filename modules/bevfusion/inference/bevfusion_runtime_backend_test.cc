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

#include "modules/bevfusion/inference/bevfusion_runtime_backend.h"

#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace bevfusion {
namespace {

TEST(BevFusionRuntimeBackendTest, RejectsUnsetRuntimeAndArtifactPaths) {
  BevFusionRuntimeBackend backend;
  BevFusionModelConfig config;
  std::string error;

  EXPECT_FALSE(backend.Init(config, 1000000, &error));
  EXPECT_NE(error.find("all five"), std::string::npos);
}

TEST(BevFusionRuntimeBackendTest, RejectsUnavailableRuntimeLibrary) {
  BevFusionRuntimeBackend backend;
  BevFusionModelConfig config;
  config.set_runtime_library_path(
      "/definitely-not-a-bevfusion-runtime/libbevfusion_runtime.so");
  config.set_camera_backbone_engine_path("camera.plan");
  config.set_lidar_scn_onnx_path("scn.onnx");
  config.set_camera_vtransform_engine_path("vtransform.plan");
  config.set_transfusion_engine_path("fuser.plan");
  config.set_bbox_engine_path("bbox.plan");
  config.set_model_revision("test-model-revision");
  config.set_confidence_threshold(0.12F);
  std::string error;

  EXPECT_FALSE(backend.Init(config, 1000000, &error));
  EXPECT_NE(error.find("unable to load"), std::string::npos);
}

}  // namespace
}  // namespace bevfusion
}  // namespace apollo
