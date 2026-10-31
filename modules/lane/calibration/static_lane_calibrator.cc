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

//  Created Date: 2026-08-28
//  Author: daohu527

#include "modules/lane/calibration/static_lane_calibrator.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace apollo {
namespace lane {

namespace {

constexpr size_t kMinPitchLanePoints = 20U;
constexpr size_t kPitchPointSampleDenominator = 20U;
constexpr float kMaxLaneYawRadians = 0.05235988F;
constexpr float kMaxLanePitchRadians = 0.17453293F;

bool UndistortImagePoint(const ImagePoint& image_point,
                         const StaticCameraCalibration& calibration,
                         ImagePoint* undistorted_point) {
  if (undistorted_point == nullptr || !std::isfinite(image_point.x) ||
      !std::isfinite(image_point.y)) {
    return false;
  }
  float ray_x =
      (image_point.x - calibration.principal_x) / calibration.focal_x;
  float ray_y =
      (image_point.y - calibration.principal_y) / calibration.focal_y;
  const float distorted_x = ray_x;
  const float distorted_y = ray_y;
  for (int iteration = 0; iteration < 12; ++iteration) {
    const float radius_squared = ray_x * ray_x + ray_y * ray_y;
    const float radius_fourth = radius_squared * radius_squared;
    const float radial = 1.0F + calibration.radial_distortion[0] *
                                     radius_squared +
                         calibration.radial_distortion[1] * radius_fourth +
                         calibration.radial_distortion[2] * radius_fourth *
                             radius_squared;
    if (!std::isfinite(radial) || radial <= 1.0e-6F) {
      return false;
    }
    ray_x = distorted_x / radial;
    ray_y = distorted_y / radial;
  }
  undistorted_point->x =
      calibration.principal_x + calibration.focal_x * ray_x;
  undistorted_point->y =
      calibration.principal_y + calibration.focal_y * ray_y;
  return std::isfinite(undistorted_point->x) &&
         std::isfinite(undistorted_point->y);
}

bool GetPitchLineSegment(const LaneLineResult& lane,
                         const StaticCameraCalibration& calibration,
                         float segment[4]) {
  if (lane.image_points.size() < kMinPitchLanePoints || segment == nullptr) {
    return false;
  }
  const size_t sample_count = std::max(
      size_t{1}, lane.image_points.size() / kPitchPointSampleDenominator);
  double sums[2][2] = {};
  for (size_t index = 0; index < sample_count; ++index) {
    const ImagePoint& top_distorted = lane.image_points[index];
    const ImagePoint& bottom_distorted =
        lane.image_points[lane.image_points.size() - sample_count + index];
    ImagePoint top;
    ImagePoint bottom;
    if (!UndistortImagePoint(top_distorted, calibration, &top) ||
        !UndistortImagePoint(bottom_distorted, calibration, &bottom)) {
      return false;
    }
    sums[0][0] += top.x;
    sums[0][1] += top.y;
    sums[1][0] += bottom.x;
    sums[1][1] += bottom.y;
  }
  const double divisor = static_cast<double>(sample_count);
  segment[0] = static_cast<float>(sums[1][0] / divisor);
  segment[1] = static_cast<float>(sums[1][1] / divisor);
  segment[2] = static_cast<float>(sums[0][0] / divisor);
  segment[3] = static_cast<float>(sums[0][1] / divisor);
  return std::isfinite(segment[0]) && std::isfinite(segment[1]) &&
         std::isfinite(segment[2]) && std::isfinite(segment[3]) &&
         (std::fabs(segment[0] - segment[2]) +
          std::fabs(segment[1] - segment[3])) > 1.0e-3F;
}

bool GetVanishingPoint(const float left[4], const float right[4],
                       float* x, float* y) {
  const float left_dx = left[2] - left[0];
  const float left_dy = left[3] - left[1];
  const float right_dx = right[2] - right[0];
  const float right_dy = right[3] - right[1];
  const float denominator = left_dx * right_dy - left_dy * right_dx;
  if (x == nullptr || y == nullptr || std::fabs(denominator) <= 1.0e-5F) {
    return false;
  }
  const float offset_x = right[0] - left[0];
  const float offset_y = right[1] - left[1];
  const float scale =
      (offset_x * right_dy - offset_y * right_dx) / denominator;
  *x = left[0] + scale * left_dx;
  *y = left[1] + scale * left_dy;
  return std::isfinite(*x) && std::isfinite(*y);
}

}  // namespace

bool StaticLaneCalibrator::Init(const StaticCameraCalibration& calibration) {
  if (!std::isfinite(calibration.focal_x) ||
      !std::isfinite(calibration.focal_y) ||
      !std::isfinite(calibration.principal_x) ||
      !std::isfinite(calibration.principal_y) ||
      !std::isfinite(calibration.camera_height_meters) ||
      !std::isfinite(calibration.pitch_radians) ||
      !std::all_of(calibration.radial_distortion.begin(),
                   calibration.radial_distortion.end(),
                   [](float value) { return std::isfinite(value); }) ||
      calibration.focal_x <= 0.0F || calibration.focal_y <= 0.0F ||
      calibration.camera_height_meters <= 0.0F) {
    return false;
  }
  calibration_ = calibration;
  initialized_ = true;
  return true;
}

bool StaticLaneCalibrator::ProjectToGround(const ImagePoint& image_point,
                                           float pitch_radians,
                                           CameraPoint* camera_point) const {
  if (!initialized_ || camera_point == nullptr) {
    return false;
  }
  ImagePoint undistorted_point;
  if (!UndistortImagePoint(image_point, calibration_, &undistorted_point)) {
    return false;
  }
  const float ray_x =
      (undistorted_point.x - calibration_.principal_x) / calibration_.focal_x;
  const float ray_y =
      (undistorted_point.y - calibration_.principal_y) / calibration_.focal_y;
  const float ground_direction =
      std::cos(pitch_radians) * ray_y + std::sin(pitch_radians);
  if (!std::isfinite(ground_direction) ||
      std::fabs(ground_direction) <= 1.0e-6F) {
    return false;
  }
  const float scale = calibration_.camera_height_meters / ground_direction;
  if (!std::isfinite(scale) || scale <= 0.0F) {
    return false;
  }
  camera_point->x = scale * ray_x;
  camera_point->y = scale * ray_y;
  camera_point->z = scale;
  return std::isfinite(camera_point->x) && std::isfinite(camera_point->y) &&
         std::isfinite(camera_point->z);
}

bool StaticLaneCalibrator::EstimatePitchFromLanePoints(
    const LaneDetectionResult& result, float* pitch_radians) const {
  if (pitch_radians == nullptr) {
    return false;
  }
  const LaneLineResult* left_lane = nullptr;
  const LaneLineResult* right_lane = nullptr;
  for (const LaneLineResult& lane : result.lanes) {
    if (lane.position == LanePosition::kEgoLeft) {
      left_lane = &lane;
    } else if (lane.position == LanePosition::kEgoRight) {
      right_lane = &lane;
    }
  }
  if (left_lane == nullptr || right_lane == nullptr) {
    return false;
  }
  float left_segment[4] = {};
  float right_segment[4] = {};
  if (!GetPitchLineSegment(*left_lane, calibration_, left_segment) ||
      !GetPitchLineSegment(*right_lane, calibration_, right_segment)) {
    return false;
  }
  float vanishing_x = 0.0F;
  float vanishing_y = 0.0F;
  if (!GetVanishingPoint(left_segment, right_segment, &vanishing_x,
                         &vanishing_y)) {
    return false;
  }
  const float yaw =
      std::atan2(vanishing_x - calibration_.principal_x, calibration_.focal_x);
  const float pitch =
      std::atan2(calibration_.principal_y - vanishing_y, calibration_.focal_y);
  if (std::fabs(yaw) > kMaxLaneYawRadians ||
      std::fabs(pitch) > kMaxLanePitchRadians) {
    return false;
  }
  *pitch_radians = pitch;
  return true;
}

bool StaticLaneCalibrator::FitCameraCurve(LaneLineResult* lane) {
  if (lane == nullptr || lane->camera_points.size() < 4U) {
    return false;
  }
  long double normal[4][5] = {};
  float longitudinal_min = lane->camera_points.front().z;
  float longitudinal_max = longitudinal_min;
  for (const CameraPoint& point : lane->camera_points) {
    const long double z = point.z;
    long double powers[7] = {1.0L};
    for (size_t power = 1; power < 7U; ++power) {
      powers[power] = powers[power - 1U] * z;
    }
    for (size_t row = 0; row < 4U; ++row) {
      for (size_t column = 0; column < 4U; ++column) {
        normal[row][column] += powers[row + column];
      }
      normal[row][4] += powers[row] * point.x;
    }
    longitudinal_min = std::min(longitudinal_min, point.z);
    longitudinal_max = std::max(longitudinal_max, point.z);
  }
  for (size_t pivot = 0; pivot < 4U; ++pivot) {
    size_t largest_row = pivot;
    for (size_t row = pivot + 1U; row < 4U; ++row) {
      if (std::fabs(normal[row][pivot]) >
          std::fabs(normal[largest_row][pivot])) {
        largest_row = row;
      }
    }
    if (std::fabs(normal[largest_row][pivot]) <= 1.0e-12L) {
      return false;
    }
    if (largest_row != pivot) {
      for (size_t column = pivot; column < 5U; ++column) {
        std::swap(normal[pivot][column], normal[largest_row][column]);
      }
    }
    const long double divisor = normal[pivot][pivot];
    for (size_t column = pivot; column < 5U; ++column) {
      normal[pivot][column] /= divisor;
    }
    for (size_t row = 0; row < 4U; ++row) {
      if (row == pivot) {
        continue;
      }
      const long double factor = normal[row][pivot];
      for (size_t column = pivot; column < 5U; ++column) {
        normal[row][column] -= factor * normal[pivot][column];
      }
    }
  }
  for (size_t index = 0; index < 4U; ++index) {
    if (!std::isfinite(static_cast<double>(normal[index][4]))) {
      return false;
    }
  }
  lane->camera_curve.valid = true;
  lane->camera_curve.longitude_min = longitudinal_min;
  lane->camera_curve.longitude_max = longitudinal_max;
  lane->camera_curve.a = static_cast<float>(normal[3][4]);
  lane->camera_curve.b = static_cast<float>(normal[2][4]);
  lane->camera_curve.c = static_cast<float>(normal[1][4]);
  lane->camera_curve.d = static_cast<float>(normal[0][4]);
  return true;
}

bool StaticLaneCalibrator::PopulateCameraCoordinates(
    LaneDetectionResult* result) const {
  if (!initialized_ || result == nullptr) {
    return false;
  }
  float pitch_radians = calibration_.pitch_radians;
  EstimatePitchFromLanePoints(*result, &pitch_radians);
  result->calibration_pitch_radians = pitch_radians;
  result->camera_height_meters = calibration_.camera_height_meters;
  for (LaneLineResult& lane : result->lanes) {
    lane.camera_points.clear();
    lane.camera_curve = CameraCurve();
    lane.camera_points.reserve(lane.image_points.size());
    for (const ImagePoint& image_point : lane.image_points) {
      CameraPoint camera_point;
      if (ProjectToGround(image_point, pitch_radians, &camera_point)) {
        lane.camera_points.push_back(camera_point);
      }
    }
    FitCameraCurve(&lane);
  }
  return true;
}

}  // namespace lane
}  // namespace apollo
