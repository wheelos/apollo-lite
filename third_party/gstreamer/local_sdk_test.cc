// Copyright 2026 The Apollo Authors. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gst/app/gstappsrc.h>
#include <gst/video/video-info.h>

#include "gtest/gtest.h"

TEST(GStreamerLocalSdkTest, AppAndVideoLibrariesAreLinked) {
  ASSERT_NE(gst_app_src_get_type(), 0u);
  GstVideoInfo info;
  gst_video_info_init(&info);
  ASSERT_TRUE(gst_video_info_set_format(&info, GST_VIDEO_FORMAT_RGB, 128, 72));
  EXPECT_EQ(GST_VIDEO_INFO_WIDTH(&info), 128);
  EXPECT_EQ(GST_VIDEO_INFO_HEIGHT(&info), 72);
}
