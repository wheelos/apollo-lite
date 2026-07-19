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

#include "modules/drivers/camera_gst/frame_extractor.h"

#include "cyber/time/time.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

namespace {

double MeasurementTimeFromBuffer(GstBuffer* buffer) {
  return GST_BUFFER_PTS_IS_VALID(buffer)
             ? static_cast<double>(GST_BUFFER_PTS(buffer)) / GST_SECOND
             : apollo::cyber::Time::Now().ToSecond();
}

bool ReadFrameShape(GstCaps* caps, int* width, int* height,
                    std::string* format) {
  if (caps == nullptr || width == nullptr || height == nullptr) {
    return false;
  }
  GstStructure* structure = gst_caps_get_structure(caps, 0);
  if (structure == nullptr ||
      !gst_structure_get_int(structure, "width", width) ||
      !gst_structure_get_int(structure, "height", height) || *width <= 0 ||
      *height <= 0) {
    return false;
  }
  if (format != nullptr) {
    const char* caps_format = gst_structure_get_string(structure, "format");
    *format = caps_format == nullptr ? "" : caps_format;
  }
  return true;
}

bool HasNvmmMemory(GstCaps* caps) {
  if (caps == nullptr || gst_caps_get_size(caps) == 0) {
    return false;
  }
  GstCapsFeatures* features = gst_caps_get_features(caps, 0);
  return features != nullptr &&
         gst_caps_features_contains(features, "memory:NVMM");
}

}  // namespace

PublishedFrame ExtractCpuFrame(GstSample* sample) {
  PublishedFrame frame;
  if (sample == nullptr) {
    return frame;
  }

  GstBuffer* buffer = gst_sample_get_buffer(sample);
  GstCaps* caps = gst_sample_get_caps(sample);
  if (buffer == nullptr || caps == nullptr) {
    return frame;
  }

  int width = 0;
  int height = 0;
  std::string format;
  if (!ReadFrameShape(caps, &width, &height, &format)) {
    return frame;
  }

  GstMapInfo map_info;
  if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
    return frame;
  }
  if (map_info.size == 0 || map_info.size % static_cast<size_t>(height) != 0) {
    gst_buffer_unmap(buffer, &map_info);
    return frame;
  }

  frame.width = static_cast<uint32_t>(width);
  frame.height = static_cast<uint32_t>(height);
  frame.step =
      static_cast<uint32_t>(map_info.size / static_cast<size_t>(height));
  frame.measurement_time = MeasurementTimeFromBuffer(buffer);
  frame.encoding = format == "BGR" ? "bgr8" : "rgb8";
  frame.data.assign(reinterpret_cast<const char*>(map_info.data),
                    map_info.size);

  gst_buffer_unmap(buffer, &map_info);
  return frame;
}

GpuFrame ExtractNvmmFrame(GstSample* sample, const std::string& source_name) {
  GpuFrame frame;
  frame.source_name = source_name;
  if (sample == nullptr) {
    return frame;
  }

  GstBuffer* buffer = gst_sample_get_buffer(sample);
  GstCaps* caps = gst_sample_get_caps(sample);
  if (buffer == nullptr || caps == nullptr || !HasNvmmMemory(caps)) {
    return frame;
  }

  int width = 0;
  int height = 0;
  std::string format;
  if (!ReadFrameShape(caps, &width, &height, &format)) {
    return frame;
  }

  GstMemory* memory = gst_buffer_peek_memory(buffer, 0);
  if (memory == nullptr) {
    return frame;
  }

  GstMapInfo map_info;
  if (!gst_memory_map(memory, &map_info, GST_MAP_READ)) {
    return frame;
  }
  if (map_info.data == nullptr) {
    gst_memory_unmap(memory, &map_info);
    return frame;
  }

  gst_buffer_ref(buffer);
  struct CleanupInfo {
    GstMemory* memory = nullptr;
    GstMapInfo map_info = {};
    GstBuffer* buffer = nullptr;
  };
  auto* cleanup = new CleanupInfo{memory, map_info, buffer};
  auto handle = std::shared_ptr<GpuFrameHandle>(
      new GpuFrameHandle(), [cleanup](GpuFrameHandle* handle) {
        gst_memory_unmap(cleanup->memory, &cleanup->map_info);
        gst_buffer_unref(cleanup->buffer);
        delete cleanup;
        delete handle;
      });

  handle->surface = reinterpret_cast<NvBufSurface*>(map_info.data);
  handle->pts = GST_BUFFER_PTS_IS_VALID(buffer) ? GST_BUFFER_PTS(buffer) : 0;

  frame.width = static_cast<uint32_t>(width);
  frame.height = static_cast<uint32_t>(height);
  frame.measurement_time = MeasurementTimeFromBuffer(buffer);
  frame.format = format;
  frame.handle = std::move(handle);
  return frame;
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
