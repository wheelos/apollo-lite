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

#include "modules/drivers/camera/backend/camera_device.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <linux/videodev2.h>  // NOLINT(build/include_order)

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace camera {
namespace {

struct FakeCaptureState {
  uint32_t width = 2;
  uint32_t height = 1;
  uint32_t pixel_format = V4L2_PIX_FMT_YUYV;
  int wait_result = 1;
  std::vector<uint8_t> frame = {16, 128, 235, 128};
  std::vector<uint32_t> queued_indices;
};

class FakeCaptureDevice : public CameraCaptureDevice {
 public:
  explicit FakeCaptureDevice(std::shared_ptr<FakeCaptureState> state)
      : state_(std::move(state)) {}

  void Configure(uint32_t, uint32_t, uint32_t, int) override {}
  bool SetParameter(uint32_t, int32_t) override { return true; }
  void StartStreaming() override {}
  void StopStreaming() noexcept override {}
  int WaitForData(long, long) override {  // NOLINT(runtime/int)
    return state_->wait_result;
  }

  V4L2Buffer DequeueBuffer() override {
    V4L2Buffer buffer;
    buffer.start = state_->frame.data();
    buffer.length = state_->frame.size();
    buffer.index = 7;
    buffer.flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    buffer.timestamp.tv_sec = 10;
    buffer.timestamp.tv_usec = 250000;
    return buffer;
  }

  void QueueBuffer(uint32_t index) override {
    state_->queued_indices.push_back(index);
  }

  uint32_t GetWidth() const override { return state_->width; }
  uint32_t GetHeight() const override { return state_->height; }
  uint32_t GetPixelFormat() const override { return state_->pixel_format; }

 private:
  std::shared_ptr<FakeCaptureState> state_;
};

CameraDevice::ConfPtr MakeConfig() {
  auto config = std::make_shared<config::Config>();
  config->set_camera_dev("fake-camera");
  config->set_pixel_format("yuyv");
  config->set_width(2);
  config->set_height(1);
  config->set_frame_rate(30);
  config->set_frame_id("front");
  config->set_output_type(config::YUYV);
  return config;
}

CameraDevice::DeviceFactory MakeFactory(
    const std::shared_ptr<FakeCaptureState>& state) {
  return [state](CameraCaptureDevice::ConfPtr) {
    return std::make_unique<FakeCaptureDevice>(state);
  };
}

TEST(CameraDeviceTest, PollPublishesFrameContractAndRequeuesBuffer) {
  auto state = std::make_shared<FakeCaptureState>();
  CameraDevice camera(MakeConfig(), MakeFactory(state));
  ASSERT_TRUE(camera.IsCapturing());

  auto image = std::make_shared<Image>();
  ASSERT_TRUE(camera.Poll(image));

  EXPECT_EQ(image->frame_id(), "front");
  EXPECT_EQ(image->measurement_time(), 10.25);
  EXPECT_EQ(image->encoding(), "yuyv");
  EXPECT_EQ(image->width(), 2U);
  EXPECT_EQ(image->height(), 1U);
  EXPECT_EQ(image->step(), 4U);
  EXPECT_EQ(image->data(),
            std::string(reinterpret_cast<const char*>(state->frame.data()),
                        state->frame.size()));
  EXPECT_EQ(state->queued_indices, std::vector<uint32_t>({7}));
}

TEST(CameraDeviceTest, PollRejectsTruncatedFrameAndStillRequeuesBuffer) {
  auto state = std::make_shared<FakeCaptureState>();
  state->frame = {16, 128};
  CameraDevice camera(MakeConfig(), MakeFactory(state));
  ASSERT_TRUE(camera.IsCapturing());

  auto image = std::make_shared<Image>();
  EXPECT_FALSE(camera.Poll(image));
  EXPECT_EQ(state->queued_indices, std::vector<uint32_t>({7}));
}

TEST(CameraDeviceTest, PollRetriesAfterInitialDeviceFailure) {
  auto state = std::make_shared<FakeCaptureState>();
  int attempts = 0;
  CameraDevice camera(MakeConfig(), [&](CameraDevice::ConfPtr) {
    if (++attempts == 1) {
      throw std::runtime_error("simulated disconnect");
    }
    return std::make_unique<FakeCaptureDevice>(state);
  });
  ASSERT_FALSE(camera.IsCapturing());
  EXPECT_FALSE(camera.Poll(std::make_shared<Image>()));
  EXPECT_TRUE(camera.IsCapturing());
  EXPECT_EQ(attempts, 2);
}

}  // namespace
}  // namespace camera
}  // namespace drivers
}  // namespace apollo
