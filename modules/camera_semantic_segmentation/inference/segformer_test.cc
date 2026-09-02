// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/camera_semantic_segmentation/inference/segformer.h"

#include <cmath>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace camera_semantic_segmentation {

class MockSegFormerExecutor : public SegFormerExecutor {
 public:
  bool Init(const SegFormerModelOptions& options) override {
    options_ = options;
    return true;
  }

  bool Run(const std::vector<float>& input,
           SegFormerTensor* output) override {
    if (output == nullptr) {
      return false;
    }
    const uint32_t h = options_.preprocess.height;
    const uint32_t w = options_.preprocess.width;
    const uint32_t c = options_.num_classes;
    output->shape = {1, static_cast<int64_t>(c), static_cast<int64_t>(h),
                     static_cast<int64_t>(w)};
    output->values.assign(static_cast<std::size_t>(h * w * c), 0.0F);

    // Set class 13 (CAR in Cityscapes) highest for the top-left pixel
    // in NCHW layout: (cls * h + y) * w + x
    const std::size_t car_offset = (13U * h + 0U) * w + 0U;
    output->values[car_offset] = 10.0F;

    // Set class 0 (ROAD) highest for pixel (1, 1)
    const std::size_t road_offset = (0U * h + 1U) * w + 1U;
    output->values[road_offset] = 5.0F;

    return true;
  }

 private:
  SegFormerModelOptions options_;
};

TEST(SegFormerTest, PreprocessAndDecode) {
  SegFormerModelOptions options;
  options.preprocess.width = 4;
  options.preprocess.height = 4;
  options.num_classes = 19;
  options.output_layout = "NCHW";
  options.camera_name = "front_6mm";
  options.source_topic = "/apollo/sensor/camera/front_6mm/image";

  MockSegFormerExecutor mock_executor;
  SegFormerSegmenter segmenter(&mock_executor);

  std::string error;
  ASSERT_TRUE(segmenter.Init(options, &error)) << error;

  std::vector<uint8_t> dummy_image(8 * 8 * 3, 128);
  ImageView view;
  view.bytes = dummy_image.data();
  view.byte_count = dummy_image.size();
  view.width = 8;
  view.height = 8;
  view.encoding = ImageEncoding::kRgb8;
  view.camera_name = "front_6mm";
  view.timestamp_sec = 100.0;
  view.frame_id = "camera_front";

  CameraSemanticSegmentationResult result;
  ASSERT_TRUE(segmenter.Segment(view, &result, &error)) << error;

  EXPECT_EQ(result.width(), 4U);
  EXPECT_EQ(result.height(), 4U);
  EXPECT_EQ(result.num_classes(), 19U);
  EXPECT_EQ(result.mask().size(), 16U);
  EXPECT_EQ(result.pixel_labels_size(), 16);

  // Check top-left pixel is class 13 (CAR)
  EXPECT_EQ(result.pixel_labels(0), 13U);
  EXPECT_GT(result.pixel_confidences(0), 0.9F);

  // Check pixel (1, 1) -> index 1 * 4 + 1 = 5 is class 0 (ROAD)
  EXPECT_EQ(result.pixel_labels(5), 0U);
  EXPECT_GT(result.pixel_confidences(5), 0.5F);
}

TEST(SegFormerTest, DecoderNHWC) {
  SegFormerModelOptions options;
  options.preprocess.width = 2;
  options.preprocess.height = 2;
  options.num_classes = 3;
  options.output_layout = "NHWC";

  SegFormerDecoder decoder;
  std::string error;
  ASSERT_TRUE(decoder.Init(options, &error)) << error;

  SegFormerTensor tensor;
  tensor.shape = {1, 2, 2, 3};
  // NHWC layout: (y * w + x) * c + cls
  tensor.values = {
      0.0F, 10.0F, 0.0F,  // (0,0) -> class 1
      5.0F, 0.0F,  0.0F,  // (0,1) -> class 0
      0.0F, 0.0F,  8.0F,  // (1,0) -> class 2
      1.0F, 2.0F,  1.0F   // (1,1) -> class 1
  };

  SegmentationMask mask;
  ASSERT_TRUE(decoder.Decode(tensor, &mask, &error)) << error;
  EXPECT_EQ(mask.labels[0], 1U);
  EXPECT_EQ(mask.labels[1], 0U);
  EXPECT_EQ(mask.labels[2], 2U);
  EXPECT_EQ(mask.labels[3], 1U);
}

}  // namespace camera_semantic_segmentation
}  // namespace apollo
