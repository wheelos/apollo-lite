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

#include "modules/drivers/camera_gst/frame_stitcher.h"

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

namespace {

config::Config BuildConfig() {
  config::Config config;
  config.set_rows(1);
  config.set_cols(2);
  config.set_tile_width(2);
  config.set_tile_height(2);

  auto* left = config.add_sources();
  left->set_name("left");
  left->set_uri("/dev/video0");
  auto* right = config.add_sources();
  right->set_name("right");
  right->set_uri("/dev/video1");

  auto* left_slot = config.add_layout_slots();
  left_slot->set_source_name("left");
  left_slot->set_row(0);
  left_slot->set_col(0);
  auto* right_slot = config.add_layout_slots();
  right_slot->set_source_name("right");
  right_slot->set_row(0);
  right_slot->set_col(1);
  return config;
}

CapturedFrame MakeFrame(const std::string& name, const cv::Scalar& color) {
  CapturedFrame frame;
  frame.source_name = name;
  frame.image_rgb = cv::Mat(2, 2, CV_8UC3, color).clone();
  return frame;
}

}  // namespace

TEST(GridFrameStitcherTest, StitchesFramesByConfiguredLayout) {
  const GridFrameStitcher stitcher(BuildConfig());
  ASSERT_TRUE(stitcher.valid());

  cv::Mat stitched;
  ASSERT_TRUE(stitcher.Stitch(
      {MakeFrame("left", cv::Scalar(10, 20, 30)),
       MakeFrame("right", cv::Scalar(40, 50, 60))},
      &stitched));
  ASSERT_EQ(stitched.rows, 2);
  ASSERT_EQ(stitched.cols, 4);
  EXPECT_EQ(stitched.at<cv::Vec3b>(0, 0)[0], 10);
  EXPECT_EQ(stitched.at<cv::Vec3b>(0, 1)[1], 20);
  EXPECT_EQ(stitched.at<cv::Vec3b>(0, 2)[0], 40);
  EXPECT_EQ(stitched.at<cv::Vec3b>(0, 3)[2], 60);
}

TEST(GridFrameStitcherTest, RejectsMissingLayoutFrame) {
  const GridFrameStitcher stitcher(BuildConfig());
  ASSERT_TRUE(stitcher.valid());

  cv::Mat stitched;
  EXPECT_FALSE(
      stitcher.Stitch({MakeFrame("left", cv::Scalar(1, 2, 3))}, &stitched));
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
