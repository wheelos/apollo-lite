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

#include "modules/lane/calibration/static_lane_calibrator.h"

#include "gtest/gtest.h"

namespace apollo {
namespace lane {
namespace {

StaticLaneCalibrator MakeCalibrator() {
  StaticCameraCalibration calibration;
  calibration.focal_x = 100.0F;
  calibration.focal_y = 100.0F;
  calibration.principal_x = 100.0F;
  calibration.principal_y = 100.0F;
  calibration.camera_height_meters = 1.5F;
  StaticLaneCalibrator calibrator;
  EXPECT_TRUE(calibrator.Init(calibration));
  return calibrator;
}

Eigen::Isometry3d MakeVehicleFromCameraTransform() {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
  transform.translation().z() = 1.5;
  return transform;
}

TEST(StaticLaneCalibratorTest, ProjectsImagePointsOntoGroundAndBackToCamera) {
  LaneDetectionResult result;
  result.lanes.resize(1);
  result.lanes[0].image_points = {{100.0F, 200.0F}, {150.0F, 200.0F}};

  ASSERT_TRUE(MakeCalibrator().PopulateCameraCoordinates(
      &result, 200, 200, MakeVehicleFromCameraTransform()));
  ASSERT_EQ(result.lanes[0].camera_points.size(), 2U);
  EXPECT_NEAR(result.lanes[0].camera_points[0].x, 0.0F, 1.0e-5F);
  EXPECT_NEAR(result.lanes[0].camera_points[0].y, 1.5F, 1.0e-5F);
  EXPECT_NEAR(result.lanes[0].camera_points[0].z, 1.5F, 1.0e-5F);
  EXPECT_NEAR(result.lanes[0].camera_points[1].x, 0.75F, 1.0e-5F);
  EXPECT_NEAR(result.lanes[0].camera_points[1].y, 1.5F, 1.0e-5F);
  EXPECT_NEAR(result.lanes[0].camera_points[1].z, 1.5F, 1.0e-5F);
}

TEST(StaticLaneCalibratorTest, SkipsRaysThatDoNotIntersectGroundInFront) {
  LaneDetectionResult result;
  result.lanes.resize(1);
  result.lanes[0].image_points = {{100.0F, 100.0F}, {100.0F, 200.0F}};

  ASSERT_TRUE(MakeCalibrator().PopulateCameraCoordinates(
      &result, 200, 200, MakeVehicleFromCameraTransform()));
  ASSERT_EQ(result.lanes[0].camera_points.size(), 1U);
  EXPECT_NEAR(result.lanes[0].camera_points[0].z, 1.5F, 1.0e-5F);
}

}  // namespace
}  // namespace lane
}  // namespace apollo
