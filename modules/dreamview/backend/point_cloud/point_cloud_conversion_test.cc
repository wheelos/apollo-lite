// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/dreamview/backend/point_cloud/point_cloud_conversion.h"

#include <memory>

#include "gtest/gtest.h"

#include "modules/common/vehicle_calibration/test_bundle.h"
#include "modules/transform/buffer.h"
#include "modules/transform/calibration/static_transform_loader.h"
#include "modules/transform/ground_reference.h"
#include "modules/transform/tests/buffer_test_peer.h"
#include "modules/transform/transform_query.h"

namespace apollo {
namespace dreamview {
namespace {

TEST(PointCloudConversionTest,
     LidarAndUnifiedBaseCloudsShareGroundCoordinates) {
  namespace vc = apollo::common::vehicle_calibration;
  vc::TestBundle bundle;
  bundle.AddSensor(vc::LIDAR, "lidar", "lidar_link");
  bundle.Write("lidar_link.yaml",
               "header: {frame_id: base_link}\nchild_frame_id: lidar_link\n"
               "transform:\n  translation: {x: 1, y: 0, z: 1.5}\n"
               "  rotation: {x: 0, y: 0.7071067811865476, z: 0, "
               "w: 0.7071067811865476}\n");
  ASSERT_TRUE(bundle.Publish());
  vc::Registry registry;
  ASSERT_TRUE(registry.LoadFromEnvironment());
  transform::TransformStampeds transforms;
  ASSERT_TRUE(transform::LoadStaticTransforms(registry, &transforms));
  transform::Buffer buffer(false);
  transform::BufferTestPeer::ReceiveStatic(
      buffer, std::make_shared<transform::TransformStampeds>(transforms));
  transform::TransformQuery query(&buffer);
  Eigen::Affine3d ground_from_lidar;
  Eigen::Affine3d ground_from_base;
  ASSERT_TRUE(transform::QueryGroundTransform(
      query, "lidar_link", 1.8, "lidar_link", &ground_from_lidar));
  ASSERT_TRUE(transform::QueryGroundTransform(query, "lidar_link", 1.8,
                                              "base_link", &ground_from_base));
  drivers::PointCloud lidar_cloud;
  lidar_cloud.set_width(1);
  lidar_cloud.set_height(1);
  auto* point = lidar_cloud.add_point();
  point->set_x(1);
  point->set_y(0);
  point->set_z(0);
  drivers::PointCloud unified_cloud;
  point = unified_cloud.add_point();
  point->set_x(1);
  point->set_y(0);
  point->set_z(0.5);
  const auto lidar = ConvertPCLPointCloud(lidar_cloud, ground_from_lidar);
  const auto unified = ConvertPCLPointCloud(unified_cloud, ground_from_base);
  ASSERT_EQ(lidar->size(), 1u);
  ASSERT_EQ(unified->size(), 1u);
  EXPECT_NEAR(lidar->points[0].x, 1, 1e-6);
  EXPECT_NEAR(lidar->points[0].z, 0.8, 1e-6);
  EXPECT_NEAR(lidar->points[0].x, unified->points[0].x, 1e-6);
  EXPECT_NEAR(lidar->points[0].y, unified->points[0].y, 1e-6);
  EXPECT_NEAR(lidar->points[0].z, unified->points[0].z, 1e-6);
  EXPECT_EQ(unified->width * unified->height, unified->size());
  EXPECT_FALSE(transform::QueryGroundTransform(query, "lidar_link", 1.8,
                                               "unknown", &ground_from_base));
  EXPECT_FALSE(transform::QueryGroundTransform(query, "lidar_link", 1.8, "odom",
                                               &ground_from_base));
}

}  // namespace
}  // namespace dreamview
}  // namespace apollo
