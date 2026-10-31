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

#include <cmath>

#include "gtest/gtest.h"

namespace apollo {
namespace lane {
namespace {

TEST(StaticLaneCalibratorTest, UndistortsImagePointsBeforeGroundProjection) {
  constexpr float kFocalLength = 100.0F;
  constexpr float kNormalizedX = 0.4F;
  constexpr float kNormalizedY = 0.5F;
  constexpr float kRadiusSquared =
      kNormalizedX * kNormalizedX + kNormalizedY * kNormalizedY;
  constexpr float kK1 = 0.1F;
  const float radial = 1.0F + kK1 * kRadiusSquared;

  StaticCameraCalibration calibration;
  calibration.focal_x = kFocalLength;
  calibration.focal_y = kFocalLength;
  calibration.principal_x = 320.0F;
  calibration.principal_y = 240.0F;
  calibration.camera_height_meters = 1.5F;
  calibration.radial_distortion = {kK1, 0.0F, 0.0F};
  StaticLaneCalibrator calibrator;
  ASSERT_TRUE(calibrator.Init(calibration));

  LaneDetectionResult result;
  LaneLineResult lane;
  lane.image_points.push_back(
      {calibration.principal_x + kFocalLength * kNormalizedX * radial,
       calibration.principal_y + kFocalLength * kNormalizedY * radial});
  result.lanes.push_back(lane);

  ASSERT_TRUE(calibrator.PopulateCameraCoordinates(&result));
  ASSERT_EQ(result.lanes.front().camera_points.size(), 1U);
  EXPECT_NEAR(result.lanes.front().camera_points.front().x, 1.2F, 1.0e-4F);
  EXPECT_NEAR(result.lanes.front().camera_points.front().y, 1.5F, 1.0e-4F);
  EXPECT_NEAR(result.lanes.front().camera_points.front().z, 3.0F, 1.0e-4F);
}

TEST(StaticLaneCalibratorTest, ZeroDistortionUsesPinholeProjection) {
  StaticCameraCalibration calibration;
  calibration.focal_x = 100.0F;
  calibration.focal_y = 100.0F;
  calibration.principal_x = 320.0F;
  calibration.principal_y = 240.0F;
  calibration.camera_height_meters = 1.5F;
  StaticLaneCalibrator calibrator;
  ASSERT_TRUE(calibrator.Init(calibration));

  LaneDetectionResult result;
  LaneLineResult lane;
  lane.image_points.push_back({360.0F, 290.0F});
  result.lanes.push_back(lane);

  ASSERT_TRUE(calibrator.PopulateCameraCoordinates(&result));
  ASSERT_EQ(result.lanes.front().camera_points.size(), 1U);
  EXPECT_NEAR(result.lanes.front().camera_points.front().x, 1.2F, 1.0e-5F);
  EXPECT_NEAR(result.lanes.front().camera_points.front().y, 1.5F, 1.0e-5F);
  EXPECT_NEAR(result.lanes.front().camera_points.front().z, 3.0F, 1.0e-5F);
}

TEST(StaticLaneCalibratorTest, RejectsHorizonAndAboveHorizonRays) {
  StaticCameraCalibration calibration;
  calibration.focal_x = 100.0F;
  calibration.focal_y = 100.0F;
  calibration.principal_x = 320.0F;
  calibration.principal_y = 240.0F;
  calibration.camera_height_meters = 1.5F;
  StaticLaneCalibrator calibrator;
  ASSERT_TRUE(calibrator.Init(calibration));

  LaneDetectionResult result;
  LaneLineResult lane;
  lane.image_points = {{320.0F, 240.0F}, {320.0F, 230.0F},
                       {320.0F, 290.0F}};
  result.lanes.push_back(lane);

  ASSERT_TRUE(calibrator.PopulateCameraCoordinates(&result));
  ASSERT_EQ(result.lanes.front().camera_points.size(), 1U);
  EXPECT_NEAR(result.lanes.front().camera_points.front().z, 3.0F, 1.0e-5F);
}

TEST(StaticLaneCalibratorTest,
     EstimatesPitchFromEgoLanePointsBeforeGroundProjection) {
  StaticCameraCalibration calibration;
  calibration.focal_x = 100.0F;
  calibration.focal_y = 100.0F;
  calibration.principal_x = 320.0F;
  calibration.principal_y = 240.0F;
  calibration.camera_height_meters = 1.5F;
  StaticLaneCalibrator calibrator;
  ASSERT_TRUE(calibrator.Init(calibration));

  LaneDetectionResult result;
  LaneLineResult left_lane;
  left_lane.position = LanePosition::kEgoLeft;
  LaneLineResult right_lane;
  right_lane.position = LanePosition::kEgoRight;
  for (int index = 0; index < 24; ++index) {
    const float y = 230.0F + static_cast<float>(index) * 10.0F;
    const float offset = (y - 225.0F) * 0.25F;
    left_lane.image_points.push_back({320.0F - offset, y});
    right_lane.image_points.push_back({320.0F + offset, y});
  }
  result.lanes = {left_lane, right_lane};

  ASSERT_TRUE(calibrator.PopulateCameraCoordinates(&result));
  const float expected_pitch = std::atan2(240.0F - 225.0F, 100.0F);
  EXPECT_NEAR(result.calibration_pitch_radians, expected_pitch, 1.0e-5F);
  ASSERT_FALSE(result.lanes.front().camera_points.empty());
  const ImagePoint& image_point = result.lanes.front().image_points.front();
  const CameraPoint& camera_point = result.lanes.front().camera_points.front();
  const float ray_y = (image_point.y - calibration.principal_y) /
                      calibration.focal_y;
  const float ground_direction =
      std::cos(expected_pitch) * ray_y + std::sin(expected_pitch);
  EXPECT_NEAR(camera_point.z,
              calibration.camera_height_meters / ground_direction, 1.0e-4F);
}

TEST(StaticLaneCalibratorTest,
     FallsBackToConfiguredPitchWhenEgoLanePointsAreInsufficient) {
  StaticCameraCalibration calibration;
  calibration.focal_x = 100.0F;
  calibration.focal_y = 100.0F;
  calibration.principal_x = 320.0F;
  calibration.principal_y = 240.0F;
  calibration.camera_height_meters = 1.5F;
  calibration.pitch_radians = 0.05F;
  StaticLaneCalibrator calibrator;
  ASSERT_TRUE(calibrator.Init(calibration));

  LaneDetectionResult result;
  LaneLineResult left_lane;
  left_lane.position = LanePosition::kEgoLeft;
  left_lane.image_points = {{280.0F, 300.0F}, {270.0F, 320.0F}};
  LaneLineResult right_lane;
  right_lane.position = LanePosition::kEgoRight;
  right_lane.image_points = {{360.0F, 300.0F}, {370.0F, 320.0F}};
  result.lanes = {left_lane, right_lane};

  ASSERT_TRUE(calibrator.PopulateCameraCoordinates(&result));
  EXPECT_FLOAT_EQ(result.calibration_pitch_radians,
                  calibration.pitch_radians);
}

TEST(StaticLaneCalibratorTest,
     FallsBackToConfiguredPitchForParallelEgoLanePoints) {
  StaticCameraCalibration calibration;
  calibration.focal_x = 100.0F;
  calibration.focal_y = 100.0F;
  calibration.principal_x = 320.0F;
  calibration.principal_y = 240.0F;
  calibration.camera_height_meters = 1.5F;
  calibration.pitch_radians = 0.03F;
  StaticLaneCalibrator calibrator;
  ASSERT_TRUE(calibrator.Init(calibration));

  LaneDetectionResult result;
  LaneLineResult left_lane;
  left_lane.position = LanePosition::kEgoLeft;
  LaneLineResult right_lane;
  right_lane.position = LanePosition::kEgoRight;
  for (int index = 0; index < 24; ++index) {
    const float y = 230.0F + static_cast<float>(index) * 10.0F;
    left_lane.image_points.push_back({100.0F + 0.1F * y, y});
    right_lane.image_points.push_back({300.0F + 0.1F * y, y});
  }
  result.lanes = {left_lane, right_lane};

  ASSERT_TRUE(calibrator.PopulateCameraCoordinates(&result));
  EXPECT_FLOAT_EQ(result.calibration_pitch_radians,
                  calibration.pitch_radians);
}

}  // namespace
}  // namespace lane
}  // namespace apollo
