#include "modules/lidar_semantic_segmentation/types/range_projection.h"

#include <cmath>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace lidar_semantic_segmentation {

namespace {

void AddPoint(float x, float y, float z, uint32_t intensity,
              apollo::drivers::PointCloud* cloud) {
  auto* point = cloud->add_point();
  point->set_x(x);
  point->set_y(y);
  point->set_z(z);
  point->set_intensity(intensity);
}

RangeImageProjectionOptions TestOptions() {
  RangeImageProjectionOptions options;
  options.width = 8U;
  options.height = 4U;
  options.fov_up_degrees = 10.0F;
  options.fov_down_degrees = -10.0F;
  options.channel_mean.assign(kRangeRetInputChannels, 0.0F);
  options.channel_std.assign(kRangeRetInputChannels, 1.0F);
  return options;
}

}  // namespace

TEST(RangeImageProjectorTest, ProjectsPointCloudToRangeRetTensor) {
  RangeImageProjector projector;
  std::string error;
  ASSERT_TRUE(projector.Init(TestOptions(), &error)) << error;

  apollo::drivers::PointCloud cloud;
  AddPoint(1.0F, 0.0F, 0.0F, 7U, &cloud);
  RangeImage image;
  ASSERT_TRUE(projector.Project(cloud, &image, &error)) << error;

  ASSERT_EQ(image.width, 8U);
  ASSERT_EQ(image.height, 4U);
  ASSERT_EQ(image.points.size(), 1U);
  EXPECT_TRUE(image.points[0].valid);
  EXPECT_EQ(image.points[0].x, 4U);
  EXPECT_EQ(image.points[0].y, 2U);
  const std::size_t pixel = 2U * 8U + 4U;
  EXPECT_EQ(image.pixel_to_point_index[pixel], 0);
  EXPECT_FLOAT_EQ(image.input_chw[pixel], 1.0F);
  EXPECT_FLOAT_EQ(image.input_chw[4U * image.PixelCount() + pixel], 7.0F);
}

TEST(RangeImageProjectorTest, KeepsNearestPointForRangeImagePixel) {
  RangeImageProjector projector;
  std::string error;
  ASSERT_TRUE(projector.Init(TestOptions(), &error)) << error;

  apollo::drivers::PointCloud cloud;
  AddPoint(2.0F, 0.0F, 0.0F, 2U, &cloud);
  AddPoint(1.0F, 0.0F, 0.0F, 1U, &cloud);
  RangeImage image;
  ASSERT_TRUE(projector.Project(cloud, &image, &error)) << error;

  const std::size_t pixel = 2U * 8U + 4U;
  EXPECT_EQ(image.pixel_to_point_index[pixel], 1);
  EXPECT_FLOAT_EQ(image.projected_range[pixel], 1.0F);
  EXPECT_EQ(image.points[0].x, image.points[1].x);
  EXPECT_EQ(image.points[0].y, image.points[1].y);
}

TEST(RangeImageProjectorTest, ScalesApolloIntensityForRangeRet) {
  RangeImageProjectionOptions options = TestOptions();
  options.intensity_scale = 1.0F / 255.0F;
  RangeImageProjector projector;
  std::string error;
  ASSERT_TRUE(projector.Init(options, &error)) << error;

  apollo::drivers::PointCloud cloud;
  AddPoint(1.0F, 0.0F, 0.0F, 255U, &cloud);
  RangeImage image;
  ASSERT_TRUE(projector.Project(cloud, &image, &error)) << error;

  const std::size_t pixel = 2U * 8U + 4U;
  EXPECT_FLOAT_EQ(image.input_chw[4U * image.PixelCount() + pixel], 1.0F);
}

TEST(RangeImageProjectorTest, RejectsInvalidConfiguration) {
  RangeImageProjectionOptions options = TestOptions();
  options.channel_std[2] = 0.0F;
  RangeImageProjector projector;
  std::string error;
  EXPECT_FALSE(projector.Init(options, &error));
  EXPECT_FALSE(error.empty());
}

TEST(RangeImageProjectorTest, RejectsInvalidIntensityScale) {
  RangeImageProjectionOptions options = TestOptions();
  options.intensity_scale = 0.0F;
  RangeImageProjector projector;
  std::string error;
  EXPECT_FALSE(projector.Init(options, &error));
  EXPECT_FALSE(error.empty());
}

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
