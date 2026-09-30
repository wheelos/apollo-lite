// Copyright 2026 WheelOS All Rights Reserved.
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

#pragma once

#include <vector>

#include "modules/world_model/local_map/temporal_lane_map.h"

namespace apollo {
namespace world_model {

struct MeasuredCubicBoundary {
  double c0 = 0.0;
  double c1 = 0.0;
  double c2 = 0.0;
  double c3 = 0.0;
  double start_x = 0.0;
  double end_x = 0.0;
};

struct MeasuredBoundarySample {
  LanePoint point;
  double station = 0.0;
  double heading = 0.0;
  double curvature = 0.0;
};

struct MeasuredCubicLane {
  std::string session;
  uint64_t generation = 0;
  uint64_t sequence = 0;
  double measurement_time = 0.0;
  double position_error = 0.0;
  bool forward_confirmed = false;
  double sample_spacing = 0.0;
  MeasuredCubicBoundary left;
  MeasuredCubicBoundary right;
};

common::Status SampleMeasuredCubicBoundary(
    const MeasuredCubicBoundary& boundary, double start_x, double end_x,
    double spacing, std::vector<MeasuredBoundarySample>* samples);

common::Status BuildMeasuredLaneObservation(const MeasuredCubicLane& input,
                                            LaneObservation* observation);

}  // namespace world_model
}  // namespace apollo
