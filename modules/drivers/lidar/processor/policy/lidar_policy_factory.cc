// Copyright 2026 WheelOS All Rights Reserved.
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

#include <memory>
#include <string>

#include "cyber/cyber.h"
#include "modules/drivers/lidar/processor/policy/cpu_lidar_policy.h"
#include "modules/drivers/lidar/processor/policy/gpu_lidar_policy.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_interface.h"

namespace apollo {
namespace drivers {
namespace lidar {

std::unique_ptr<LidarDeskewPolicy> LidarPolicyFactory::CreateDeskewPolicy(
    const std::string& mode) {
#ifdef APOLLO_LIDAR_POLICY_FORCE_CPU
  if (mode != "cpu") {
    return nullptr;
  }
#endif
#ifdef APOLLO_LIDAR_POLICY_FORCE_GPU
  if (mode != "gpu") {
    return nullptr;
  }
#endif

  if (mode == "cpu") {
    return std::make_unique<CpuLidarDeskewPolicy>();
  }
  if (mode == "gpu") {
#ifndef APOLLO_LIDAR_POLICY_GPU_ENABLED
    AERROR << "GPU deskew policy requested but CUDA backend is disabled at "
              "compile time";
    return nullptr;
#else
    return std::make_unique<GpuLidarDeskewPolicy>();
#endif
  }
  return nullptr;
}

std::unique_ptr<LidarFusionPolicy> LidarPolicyFactory::CreateFusionPolicy(
    const std::string& mode) {
#ifdef APOLLO_LIDAR_POLICY_FORCE_CPU
  if (mode != "cpu") {
    return nullptr;
  }
#endif
#ifdef APOLLO_LIDAR_POLICY_FORCE_GPU
  if (mode != "gpu") {
    return nullptr;
  }
#endif

  if (mode == "cpu") {
    return std::make_unique<CpuLidarFusionPolicy>();
  }
  if (mode == "gpu") {
#ifndef APOLLO_LIDAR_POLICY_GPU_ENABLED
    AERROR << "GPU fusion policy requested but CUDA backend is disabled at "
              "compile time";
    return nullptr;
#else
    return std::make_unique<GpuLidarFusionPolicy>();
#endif
  }
  return nullptr;
}

std::unique_ptr<LidarFilterPolicy> LidarPolicyFactory::CreateFilterPolicy(
    const std::string& mode) {
#ifdef APOLLO_LIDAR_POLICY_FORCE_CPU
  if (mode != "cpu") {
    return nullptr;
  }
#endif
#ifdef APOLLO_LIDAR_POLICY_FORCE_GPU
  if (mode != "gpu") {
    return nullptr;
  }
#endif

  if (mode == "cpu") {
    return std::make_unique<CpuLidarFilterPolicy>();
  }
  if (mode == "gpu") {
#ifndef APOLLO_LIDAR_POLICY_GPU_ENABLED
    AERROR << "GPU filter policy requested but CUDA backend is disabled at "
              "compile time";
    return nullptr;
#else
    return std::make_unique<GpuLidarFilterPolicy>();
#endif
  }
  return nullptr;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
