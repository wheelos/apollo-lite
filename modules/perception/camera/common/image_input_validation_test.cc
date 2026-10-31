// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/perception/camera/common/image_input_validation.h"

#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace camera {
namespace {

drivers::Image Image(const std::string& encoding, int channels) {
  drivers::Image image;
  image.set_width(4);
  image.set_height(2);
  image.set_encoding(encoding);
  image.set_step(4 * channels);
  image.set_data(std::string(8 * channels, '\0'));
  return image;
}

TEST(ImageInputValidationTest, AcceptsOnlySupportedPackedImages) {
  for (const std::string encoding : {"rgb8", "bgr8", "gray", "y"}) {
    const int channels = encoding == "rgb8" || encoding == "bgr8" ? 3 : 1;
    EXPECT_TRUE(ValidateImageInput(Image(encoding, channels), 2, 4));
  }
  EXPECT_FALSE(ValidateImageInput(Image("rgba8", 4), 2, 4));
  EXPECT_FALSE(ValidateImageInput(Image("bgr8", 3), 0, 4));
}

TEST(ImageInputValidationTest, RejectsResolutionStrideAndPayloadMismatch) {
  const auto valid = Image("bgr8", 3);
  EXPECT_FALSE(ValidateImageInput(valid, 2, 5));
  EXPECT_FALSE(ValidateImageInput(valid, 3, 4));
  for (int variant = 0; variant < 5; ++variant) {
    auto image = valid;
    switch (variant) {
      case 0:
        image.set_step(16);
        image.set_data(std::string(32, '\0'));
        break;
      case 1:
        image.clear_step();
        break;
      case 2:
        image.set_step(11);
        break;
      case 3:
        image.mutable_data()->pop_back();
        break;
      case 4:
        image.mutable_data()->push_back('\0');
        break;
    }
    EXPECT_FALSE(ValidateImageInput(image, 2, 4)) << variant;
  }
  drivers::Image missing;
  EXPECT_FALSE(ValidateImageInput(missing, 2, 4));
}

}  // namespace
}  // namespace camera
}  // namespace perception
}  // namespace apollo
