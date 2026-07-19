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

#include <cstddef>
#include <string>
#include <vector>

#include "modules/drivers/camera_gst/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

struct PipelineLayoutSlot {
  std::string source_name;
  size_t pad_index = 0;
  int row = 0;
  int col = 0;
};

class CameraGstPipelineBuilder {
 public:
  CameraGstPipelineBuilder(const config::Config& config,
                           const std::vector<PipelineLayoutSlot>& layout_slots,
                           bool source_publish_enabled,
                           bool stitched_publish_enabled, bool stream_enabled,
                           bool gpu_frame_enabled);

  std::string BuildPipelineDescription() const;
  std::string BuildDefaultStreamBranch() const;
  std::vector<std::string> RequiredFactories() const;
  bool ValidateRequiredFactories() const;
  std::string SourcePublishSinkName(size_t source_index) const;
  std::string SourceGpuSinkName(size_t source_index) const;

 private:
  std::string BuildCompositorDescription() const;
  std::string BuildSourceDescription(
      size_t source_index, const config::CameraSourceConfig& source_config,
      const PipelineLayoutSlot* layout_slot) const;
  std::string BuildSourceHead(
      const config::CameraSourceConfig& source_config) const;
  const PipelineLayoutSlot* FindLayoutSlot(
      const std::string& source_name) const;
  std::string SourceTeeName(size_t source_index) const;
  std::string VideoConvertElement() const;

  const config::Config& config_;
  const std::vector<PipelineLayoutSlot>& layout_slots_;
  bool source_publish_enabled_ = false;
  bool stitched_publish_enabled_ = false;
  bool stream_enabled_ = false;
  bool gpu_frame_enabled_ = false;
  int output_width_ = 0;
  int output_height_ = 0;
};

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
