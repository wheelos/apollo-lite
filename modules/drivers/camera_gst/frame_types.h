/******************************************************************************
 * Copyright 2026 The WheelOS Team. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct NvBufSurface;

namespace apollo {
namespace drivers {
namespace camera_gst {

struct PublishedFrame {
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t step = 0;
  double measurement_time = 0.0;
  std::string encoding = "rgb8";
  std::string data;
  uint64_t sequence = 0;
};

struct SourceStats {
  std::string source_name;
  uint64_t cpu_frames = 0;
  uint64_t gpu_frames = 0;
  uint64_t cpu_rate_limited_frames = 0;
  uint64_t cpu_drop_frames = 0;
  uint64_t gpu_drop_frames = 0;
};

struct StreamStats {
  uint64_t pipeline_warning_count = 0;
  uint64_t pipeline_error_count = 0;
  uint64_t pipeline_restart_count = 0;
  bool stream_attached = false;
  std::vector<SourceStats> source_stats;
};

struct GpuFrameHandle {
  NvBufSurface* surface = nullptr;
  uint64_t pts = 0;
};

struct GpuFrame {
  std::string source_name;
  uint32_t width = 0;
  uint32_t height = 0;
  double measurement_time = 0.0;
  std::string format;
  std::string memory_type = "NVMM";
  std::shared_ptr<GpuFrameHandle> handle;

  bool empty() const { return handle == nullptr || handle->surface == nullptr; }
};

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
