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

bool StaticLaneCalibrator::Init(const StaticCameraCalibration& calibration) {
  if (!std::isfinite(calibration.focal_x) ||
      !std::isfinite(calibration.focal_y) ||
      !std::isfinite(calibration.principal_x) ||
      !std::isfinite(calibration.principal_y) ||
      !std::isfinite(calibration.camera_height_meters) ||
      !std::isfinite(calibration.pitch_radians) ||
      calibration.focal_x <= 0.0F || calibration.focal_y <= 0.0F ||
      calibration.camera_height_meters <= 0.0F) {
    return false;
  }
  calibration_ = calibration;
  initialized_ = true;
  return true;
}

bool StaticLaneCalibrator::ProjectToGround(const ImagePoint& image_point,
                                           CameraPoint* camera_point) const {
  if (!initialized_ || camera_point == nullptr) {
    return false;
  }
  const float ray_x =
      (image_point.x - calibration_.principal_x) / calibration_.focal_x;
  const float ray_y =
      (image_point.y - calibration_.principal_y) / calibration_.focal_y;
  const float ground_direction = std::cos(calibration_.pitch_radians) * ray_y +
                                 std::sin(calibration_.pitch_radians);
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
  result->calibration_pitch_radians = calibration_.pitch_radians;
  result->camera_height_meters = calibration_.camera_height_meters;
  for (LaneLineResult& lane : result->lanes) {
    lane.camera_points.clear();
    lane.camera_curve = CameraCurve();
    lane.camera_points.reserve(lane.image_points.size());
    for (const ImagePoint& image_point : lane.image_points) {
      CameraPoint camera_point;
      if (ProjectToGround(image_point, &camera_point)) {
        lane.camera_points.push_back(camera_point);
      }
    }
    FitCameraCurve(&lane);
  }
  return true;
}

}  // namespace lane
}  // namespace apollo
