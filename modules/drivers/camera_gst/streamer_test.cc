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

TEST(CameraGstPipelineBuilderTest, BuildsGpuHandleAndCpuPublishBranches) {
  config::Config config;
  auto* source = AddSource(&config, "front", "csi://0");
  source->mutable_publish()->set_channel_name("/apollo/sensor/camera/front");
  source->mutable_publish()->set_output_width(1280);
  source->mutable_publish()->set_output_height(720);
  source->mutable_publish()->set_output_fps(15.0);
  config.set_rows(1);
  config.set_cols(1);
  config.set_tile_width(1920);
  config.set_tile_height(1080);

  const std::vector<PipelineLayoutSlot> layout_slots = {
      MakeSlot("front", 0, 0, 0)};
  CameraGstPipelineBuilder builder(config, layout_slots, true, false, true,
                                   true);

  const std::string pipeline = builder.BuildPipelineDescription();
  EXPECT_NE(pipeline.find("nvarguscamerasrc sensor-id=0"), std::string::npos);
  EXPECT_NE(pipeline.find("video/x-raw(memory:NVMM)"), std::string::npos);
  EXPECT_NE(pipeline.find("appsink name=source_publish_sink_0"),
            std::string::npos);
  EXPECT_NE(pipeline.find("appsink name=source_gpu_sink_0"), std::string::npos);
  EXPECT_NE(pipeline.find("nvcompositor name=comp"), std::string::npos);
  EXPECT_NE(pipeline.find("video/x-raw,width=(int)1280,height=(int)720"),
            std::string::npos);
  EXPECT_NE(pipeline.find("videorate ! video/x-raw,format=(string)RGB,"
                          "framerate=(fraction)15/1"),
            std::string::npos);
}

TEST(CameraGstPipelineBuilderTest, BuildsMjpegHardwareDecodeSource) {
  config::Config config;
  auto* source = AddSource(&config, "usb", "/dev/video0");
  source->set_fourcc("MJPG");

  CameraGstPipelineBuilder builder(config, {}, false, false, false, true);

  const std::string pipeline = builder.BuildPipelineDescription();
  EXPECT_NE(pipeline.find("v4l2src device=\"/dev/video0\""), std::string::npos);
  EXPECT_NE(pipeline.find("jpegparse ! nvv4l2decoder mjpeg=1"),
            std::string::npos);
  EXPECT_NE(pipeline.find("appsink name=source_gpu_sink_0"), std::string::npos);
}

TEST(CameraGstPipelineBuilderTest, BuildsDefaultNvencRtpStreamBranch) {
  config::Config config;
  config.mutable_stream()->set_host("192.0.2.10");
  config.mutable_stream()->set_port(5600);
  config.mutable_stream()->set_bitrate(8000000);
  config.mutable_stream()->set_rtp_payload_type(98);

  CameraGstPipelineBuilder builder(config, {}, false, false, true, false);

  const std::string branch = builder.BuildDefaultStreamBranch();
  EXPECT_NE(branch.find("nvv4l2h264enc name=stream_encoder"),
            std::string::npos);
  EXPECT_NE(branch.find("bitrate=8000000"), std::string::npos);
  EXPECT_NE(branch.find("rtph264pay config-interval=1 pt=98"),
            std::string::npos);
  EXPECT_NE(branch.find("udpsink host=\"192.0.2.10\" port=5600"),
            std::string::npos);
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
