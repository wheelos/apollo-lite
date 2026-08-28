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
#include <vector>

namespace apollo {
namespace lane {

struct ImagePoint {
  float x = 0.0F;
  float y = 0.0F;
};

struct CameraPoint {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct CameraCurve {
  bool valid = false;
  float longitude_min = 0.0F;
  float longitude_max = 0.0F;
  float a = 0.0F;
  float b = 0.0F;
  float c = 0.0F;
  float d = 0.0F;
};

enum class LanePosition {
  kUnknown,
  kEgoLeft,
  kEgoRight,
  kAdjacentLeft,
  kAdjacentRight,
  kThirdLeft,
  kThirdRight,
};

struct LaneLineResult {
  uint32_t candidate_id = 0;
  LanePosition position = LanePosition::kUnknown;
  float confidence = 0.0F;
  std::vector<ImagePoint> image_points;
  std::vector<CameraPoint> camera_points;
  CameraCurve camera_curve;
};

struct LaneDetectionResult {
  double timestamp_sec = 0.0;
  float calibration_pitch_radians = 0.0F;
  float camera_height_meters = 0.0F;
  std::vector<LaneLineResult> lanes;
};

}  // namespace lane
}  // namespace apollo
