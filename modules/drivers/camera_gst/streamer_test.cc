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

#include <cstdlib>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "modules/drivers/camera_gst/pipeline_builder.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

namespace {

config::CameraSourceConfig* AddSource(config::Config* config,
                                      const std::string& name,
                                      const std::string& uri) {
  auto* source = config->add_sources();
  source->set_name(name);
  source->set_uri(uri);
  source->set_width(1920);
  source->set_height(1080);
  source->set_fps(30.0);
  return source;
}

PipelineLayoutSlot MakeSlot(const std::string& source_name, size_t pad_index,
                            int row, int col) {
  PipelineLayoutSlot slot;
  slot.source_name = source_name;
  slot.pad_index = pad_index;
  slot.row = row;
  slot.col = col;
  return slot;
}

}  // namespace

TEST(CameraGstPipelineBuilderTest, BuildsDirectNvmmGpuAndStreamBranches) {
  config::Config config;
  auto* source = AddSource(&config, "front", "/dev/video0");
  source->set_fourcc("UYVY");
  source->set_capture_backend("NVV4L2_DMABUF");
  config.set_publish_gpu_channel(true);
  config.set_rows(1);
  config.set_cols(1);
  config.set_tile_width(1920);
  config.set_tile_height(1080);

  const std::vector<PipelineLayoutSlot> layout_slots = {
      MakeSlot("front", 0, 0, 0)};
  CameraGstPipelineBuilder builder(config, layout_slots, true, true);

  const std::string pipeline = builder.BuildPipelineDescription();
  EXPECT_NE(pipeline.find("nvv4l2camerasrc device=\"/dev/video0\""),
            std::string::npos);
  EXPECT_NE(pipeline.find("video/x-raw(memory:NVMM)"), std::string::npos);
  EXPECT_NE(pipeline.find("videorate drop-only=true"), std::string::npos);
  EXPECT_NE(pipeline.find("framerate=(fraction)30/1"), std::string::npos);
  EXPECT_NE(pipeline.find("appsink name=source_gpu_sink_0"), std::string::npos);
  EXPECT_EQ(pipeline.find("nvcompositor name=comp"), std::string::npos);
  EXPECT_NE(pipeline.find("tee name=stitched_tee allow-not-linked=true"),
            std::string::npos);
}

TEST(CameraGstPipelineBuilderTest, BuildsMjpegHardwareDecodeSource) {
  config::Config config;
  auto* source = AddSource(&config, "usb", "/dev/video0");
  source->set_fourcc("MJPG");
  source->set_capture_backend("V4L2_MMAP");

  CameraGstPipelineBuilder builder(config, {}, false, true);

  const std::string pipeline = builder.BuildPipelineDescription();
  EXPECT_NE(pipeline.find("v4l2src device=\"/dev/video0\" io-mode=2"),
            std::string::npos);
  EXPECT_NE(pipeline.find("jpegparse ! nvv4l2decoder mjpeg=1"),
            std::string::npos);
  EXPECT_NE(pipeline.find("appsink name=source_gpu_sink_0"), std::string::npos);
}

TEST(CameraGstPipelineBuilderTest, BuildsV4l2MmapForYuyv) {
  config::Config config;
  auto* source = AddSource(&config, "front", "/dev/video0");
  source->set_fourcc("YUYV");
  source->set_capture_backend("V4L2_MMAP");
  config.set_publish_gpu_channel(true);

  CameraGstPipelineBuilder builder(config, {}, false, true);

  const std::string pipeline = builder.BuildPipelineDescription();
  EXPECT_NE(pipeline.find("v4l2src device=\"/dev/video0\" io-mode=2"),
            std::string::npos);
  EXPECT_NE(pipeline.find("format=(string)YUY2"), std::string::npos);
  EXPECT_EQ(pipeline.find("nvv4l2camerasrc"), std::string::npos);
}

TEST(CameraGstPipelineBuilderTest, UsesNvv4l2CameraSrcForSupportedV4l2Yuv) {
  config::Config config;
  auto* source = AddSource(&config, "video3", "/dev/video3");
  source->set_fourcc("UYVY");
  source->set_capture_backend("NVV4L2_DMABUF");

  CameraGstPipelineBuilder builder(config, {}, false, false);

  const std::string pipeline = builder.BuildPipelineDescription();
  EXPECT_NE(pipeline.find("nvv4l2camerasrc device="), std::string::npos);
  EXPECT_NE(pipeline.find("memory:NVMM"), std::string::npos);
}

TEST(CameraGstPipelineBuilderTest, RejectsYuyvOnNvv4l2Backend) {
  config::Config config;
  auto* source = AddSource(&config, "video3", "/dev/video3");
  source->set_fourcc("YUYV");
  source->set_capture_backend("NVV4L2_DMABUF");

  CameraGstPipelineBuilder builder(config, {}, false, false);

  EXPECT_TRUE(builder.BuildPipelineDescription().empty());
}

TEST(CameraGstPipelineBuilderTest, UsesCompatibleMmapProtoDefaults) {
  config::Config config;
  AddSource(&config, "video2", "/dev/video2");

  CameraGstPipelineBuilder builder(config, {}, false, false);

  const std::string pipeline = builder.BuildPipelineDescription();
  EXPECT_NE(pipeline.find("v4l2src device=\"/dev/video2\" io-mode=2"),
            std::string::npos);
  EXPECT_NE(pipeline.find("jpegparse ! nvv4l2decoder mjpeg=1"),
            std::string::npos);
}

TEST(CameraGstPipelineBuilderTest, BuildsDefaultNvencRtpStreamBranch) {
  config::Config config;
  config.mutable_stream()->set_host("192.0.2.10");
  config.mutable_stream()->set_port(5600);
  config.mutable_stream()->set_bitrate(8000000);
  config.mutable_stream()->set_rtp_payload_type(98);

  CameraGstPipelineBuilder builder(config, {}, true, false);

  const std::string branch = builder.BuildDefaultStreamBranch();
  EXPECT_NE(branch.find("nvv4l2h264enc name=stream_encoder"),
            std::string::npos);
  EXPECT_NE(branch.find("bitrate=8000000"), std::string::npos);
  EXPECT_NE(branch.find("rtph264pay config-interval=1 pt=98"),
            std::string::npos);
  EXPECT_NE(branch.find("udpsink host=\"192.0.2.10\" port=5600"),
            std::string::npos);
}

TEST(CameraGstPipelineBuilderTest, BuildsPerceptionTranscodePublishBranch) {
  config::Config config;
  auto* source = AddSource(&config, "video2", "/dev/video2");
  source->set_fourcc("UYVY");
  config.mutable_stream()->set_enable(true);
  config.mutable_stream()->set_host("192.0.2.20");
  config.mutable_stream()->set_port(5600);
  config.set_publish_gpu_channel(true);
  config.mutable_stream()->set_perception_transcode_before_publish(true);
  config.mutable_stream()->set_perception_bitrate(6000000);

  source->set_capture_backend("NVV4L2_DMABUF");
  CameraGstPipelineBuilder builder(config, {}, true, true);

  const std::string pipeline = builder.BuildPipelineDescription();
  EXPECT_NE(
      pipeline.find("queue name=perception_codec_queue_0 leaky=downstream"),
      std::string::npos);
  EXPECT_NE(pipeline.find("nvv4l2h264enc name=perception_codec_encoder_0"),
            std::string::npos);
  EXPECT_NE(pipeline.find("bitrate=6000000"), std::string::npos);
  EXPECT_NE(pipeline.find("h264parse ! nvv4l2decoder"), std::string::npos);
  EXPECT_EQ(pipeline.find("perception_stream_encoder"), std::string::npos);
  EXPECT_NE(pipeline.find("appsink name=source_gpu_sink_0"), std::string::npos);
}

TEST(CameraGstPipelineBuilderTest, UsesNativeVideorateForStitchedOutput) {
  config::Config config;
  AddSource(&config, "front", "/dev/video0");
  AddSource(&config, "rear", "/dev/video1");
  config.set_rows(1);
  config.set_cols(2);
  config.set_tile_width(640);
  config.set_tile_height(360);

  const std::vector<PipelineLayoutSlot> layout_slots = {
      MakeSlot("front", 0, 0, 0), MakeSlot("rear", 1, 0, 1)};
  CameraGstPipelineBuilder builder(config, layout_slots, true, false);

  const std::string pipeline = builder.BuildPipelineDescription();
  EXPECT_NE(pipeline.find("videorate drop-only=true"), std::string::npos);
  EXPECT_NE(pipeline.find(
                "framerate=(fraction)30/1 ! tee name=stitched_tee"),
            std::string::npos);
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  // Avoid environment-specific GStreamer/GLib shutdown crashes after tests.
  const int result = RUN_ALL_TESTS();
  _Exit(result);
}
