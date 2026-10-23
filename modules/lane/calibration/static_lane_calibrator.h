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

#pragma once

#include <cstdint>

#include <Eigen/Geometry>

#include "modules/lane/types/lane_types.h"

namespace wheel {
namespace vision {
class CameraModel;
}  // namespace vision
}  // namespace wheel

namespace apollo {
namespace lane {

struct StaticCameraCalibration {
  float focal_x = 0.0F;
  float focal_y = 0.0F;
  float principal_x = 0.0F;
  float principal_y = 0.0F;
  float camera_height_meters = 0.0F;
  float pitch_radians = 0.0F;
};

class StaticLaneCalibrator {
 public:
  bool Init(const StaticCameraCalibration& calibration);
  bool PopulateCameraCoordinates(
      LaneDetectionResult* result, uint32_t image_width, uint32_t image_height,
      const Eigen::Isometry3d& T_vehicle_camera) const;

 private:
  bool ProjectToGround(const wheel::vision::CameraModel& camera_model,
                       const ImagePoint& image_point,
                       uint32_t image_width, uint32_t image_height,
                       const Eigen::Isometry3d& T_vehicle_camera,
                       CameraPoint* camera_point) const;
  static bool FitCameraCurve(LaneLineResult* lane);

  StaticCameraCalibration calibration_;
  bool initialized_ = false;
};

}  // namespace lane
}  // namespace apollo
