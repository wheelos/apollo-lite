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

#include "modules/drivers/camera_gst/streamer.h"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

namespace {

class PublishedFrameCollector {
 public:
  void OnFrame(CameraGstStreamer::PublishedFrame&& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.push_back(std::move(frame));
    condition_.notify_all();
  }

  bool WaitForFrames(size_t expected) {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::seconds(2), [this, expected]() {
      return frames_.size() >= expected;
    });
  }

  CameraGstStreamer::PublishedFrame LatestFrame() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_.back();
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_.size();
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<CameraGstStreamer::PublishedFrame> frames_;
};

cv::Mat MakeRgbFrame(uint8_t r, uint8_t g, uint8_t b) {
  return cv::Mat(1, 1, CV_8UC3, cv::Scalar(r, g, b)).clone();
}

config::StreamConfig BuildConfig(bool enable, bool auto_start,
                                 const std::string& branch_pipeline) {
  config::StreamConfig config;
  config.set_enable(enable);
  config.set_auto_start(auto_start);
  config.set_queue_size(3);
  config.set_publish_queue_size(1);
  config.set_ingest_queue_size(2);
  config.set_emit_eos_on_detach(true);
  config.set_force_keyframe_on_attach(true);
  config.set_keyframe_element_name("stream_identity");
  config.set_branch_pipeline(branch_pipeline);
  return config;
}

}  // namespace

TEST(CameraGstStreamerTest, PublishesSubmittedFramesThroughAppsinkBranch) {
  PublishedFrameCollector collector;
  CameraGstStreamer streamer;

  ASSERT_TRUE(streamer.Start(
      1, 1, 30.0, BuildConfig(false, false, ""),
      [&collector](CameraGstStreamer::PublishedFrame&& frame) {
        collector.OnFrame(std::move(frame));
      }));
  ASSERT_TRUE(streamer.Submit(MakeRgbFrame(150, 100, 50), 12.25));
  ASSERT_TRUE(collector.WaitForFrames(1));

  const auto frame = collector.LatestFrame();
  ASSERT_EQ(frame.image_rgb.rows, 1);
  ASSERT_EQ(frame.image_rgb.cols, 1);
  EXPECT_NEAR(frame.measurement_time, 12.25, 1e-3);

  streamer.Stop();
}

TEST(CameraGstStreamerTest, AttachesAndDetachesDynamicStreamBranchSafely) {
  PublishedFrameCollector collector;
  CameraGstStreamer streamer;

  ASSERT_TRUE(streamer.Start(
      1, 1, 30.0,
      BuildConfig(true, false,
                  "queue name=stream_queue leaky=downstream "
                  "max-size-buffers=1 ! identity name=stream_identity silent=true "
                  "! fakesink name=stream_sink sync=false async=false"),
      [&collector](CameraGstStreamer::PublishedFrame&& frame) {
        collector.OnFrame(std::move(frame));
      }));
  EXPECT_FALSE(streamer.streaming_active());

  ASSERT_TRUE(streamer.StartStreaming());
  EXPECT_TRUE(streamer.streaming_active());
  ASSERT_TRUE(streamer.Submit(MakeRgbFrame(30, 20, 10), 1.0));
  ASSERT_TRUE(collector.WaitForFrames(1));

  ASSERT_TRUE(streamer.StopStreaming());
  EXPECT_FALSE(streamer.streaming_active());
  ASSERT_TRUE(streamer.StartStreaming());
  EXPECT_TRUE(streamer.streaming_active());

  streamer.Stop();
}

TEST(CameraGstStreamerTest, RunsStreamOnlyPipelineWithoutPublishCallback) {
  CameraGstStreamer streamer;

  ASSERT_TRUE(streamer.Start(
      1, 1, 30.0,
      BuildConfig(true, false,
                  "queue name=stream_queue leaky=downstream "
                  "max-size-buffers=1 ! identity name=stream_identity silent=true "
                  "! fakesink name=stream_sink sync=false async=false"),
      CameraGstStreamer::PublishCallback()));
  EXPECT_FALSE(streamer.streaming_active());
  ASSERT_TRUE(streamer.StartStreaming());
  EXPECT_TRUE(streamer.streaming_active());
  ASSERT_TRUE(streamer.Submit(MakeRgbFrame(1, 2, 3), 2.0));

  streamer.Stop();
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
