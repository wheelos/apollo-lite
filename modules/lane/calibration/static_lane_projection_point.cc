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

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "modules/lane/calibration/static_lane_calibrator.h"

int main() {
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) {
      continue;
    }
    std::istringstream input(line);
    std::string point_id;
    apollo::lane::ImagePoint image_point;
    apollo::lane::StaticCameraCalibration calibration;
    if (!(input >> point_id >> image_point.x >> image_point.y >>
          calibration.focal_x >> calibration.focal_y >>
          calibration.principal_x >> calibration.principal_y >>
          calibration.camera_height_meters >> calibration.pitch_radians >>
          calibration.radial_distortion[0] >>
          calibration.radial_distortion[1] >>
          calibration.radial_distortion[2])) {
      std::cerr << "invalid point input: " << line << '\n';
      return 2;
    }
    std::string trailing;
    if (input >> trailing) {
      std::cerr << "unexpected trailing input for point " << point_id << '\n';
      return 2;
    }
    apollo::lane::StaticLaneCalibrator calibrator;
    if (!calibrator.Init(calibration)) {
      std::cerr << "invalid calibration for point " << point_id << '\n';
      return 2;
    }
    apollo::lane::LaneDetectionResult result;
    apollo::lane::LaneLineResult lane;
    lane.image_points.push_back(image_point);
    result.lanes.push_back(lane);
    if (!calibrator.PopulateCameraCoordinates(&result)) {
      std::cerr << "projection call failed for point " << point_id << '\n';
      return 2;
    }
    std::cout << point_id << '\t';
    if (result.lanes.front().camera_points.size() != 1U) {
      std::cout << "rejected\t0\t0\t0\n";
      continue;
    }
    const apollo::lane::CameraPoint& point =
        result.lanes.front().camera_points.front();
    std::cout << "valid\t" << std::setprecision(9) << point.x << '\t'
              << point.y << '\t' << point.z << '\n';
  }
  return 0;
}
