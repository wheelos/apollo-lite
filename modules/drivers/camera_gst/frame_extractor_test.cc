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

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace camera_gst {
namespace {

void EnsureGstInitialized() {
  if (!gst_is_initialized()) {
    int argc = 0;
    char** argv = nullptr;
    gst_init(&argc, &argv);
  }
}

GstSample* MakeMockNvmmSample(GstBuffer** backing_buffer) {
  EnsureGstInitialized();
  GstCaps* caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING,
                                      "NV12", "width", G_TYPE_INT, 2, "height",
                                      G_TYPE_INT, 2, nullptr);
  gst_caps_set_features(caps, 0, gst_caps_features_new("memory:NVMM", nullptr));
  *backing_buffer = gst_buffer_new_allocate(nullptr, 64, nullptr);
  GST_BUFFER_PTS(*backing_buffer) = 125 * GST_MSECOND;
  GstSample* sample = gst_sample_new(*backing_buffer, caps, nullptr, nullptr);
  gst_caps_unref(caps);
  return sample;
}

TEST(FrameExtractorTest, PreservesNvmmFrameMetadataAndBackingBufferLifetime) {
  GstBuffer* backing_buffer = nullptr;
  GstSample* sample = MakeMockNvmmSample(&backing_buffer);
  ASSERT_NE(sample, nullptr);

  GpuFrame frame = ExtractNvmmFrame(sample, "front");
  gst_sample_unref(sample);
  gst_buffer_unref(backing_buffer);

  ASSERT_FALSE(frame.empty());
  EXPECT_EQ(frame.source_name, "front");
  EXPECT_EQ(frame.width, 2U);
  EXPECT_EQ(frame.height, 2U);
  EXPECT_EQ(frame.format, "NV12");
  EXPECT_EQ(frame.memory_type, "NVMM");
  EXPECT_DOUBLE_EQ(frame.measurement_time, 0.125);
  ASSERT_NE(frame.handle, nullptr);
  EXPECT_EQ(frame.handle->pts, 125 * GST_MSECOND);
}

TEST(FrameExtractorTest, RejectsSystemMemorySample) {
  EnsureGstInitialized();
  GstCaps* caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING,
                                      "NV12", "width", G_TYPE_INT, 2, "height",
                                      G_TYPE_INT, 2, nullptr);
  GstBuffer* buffer = gst_buffer_new_allocate(nullptr, 64, nullptr);
  GstSample* sample = gst_sample_new(buffer, caps, nullptr, nullptr);

  const GpuFrame frame = ExtractNvmmFrame(sample, "front");

  EXPECT_TRUE(frame.empty());
  EXPECT_EQ(frame.source_name, "front");
  gst_sample_unref(sample);
  gst_buffer_unref(buffer);
  gst_caps_unref(caps);
}

}  // namespace
}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
