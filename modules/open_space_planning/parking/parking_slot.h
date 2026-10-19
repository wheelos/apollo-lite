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

#pragma once

#include <string>
#include <vector>

#include "modules/common/math/vec2d.h"
#include "modules/open_space_planning/common/status.h"

namespace apollo {
namespace open_space_planning {

enum class ParkingSlotType {
  kUnknown = 0,
  kParallel = 1,
  kAngled = 2,
  kPerpendicular = 3,
};

struct ParkingSlotInput {
  std::string id;
  std::vector<common::math::Vec2d> corners;
  double heading = 0.0;
  double lane_heading = 0.0;
  bool has_lane_heading = false;
  int opening_edge_index = -1;
};

struct ParkingSlot {
  std::string id;
  ParkingSlotType type = ParkingSlotType::kUnknown;
  double heading = 0.0;
  double lane_heading = 0.0;
  double width = 0.0;
  double depth = 0.0;
  common::math::Vec2d center;
  common::math::Vec2d opening_center;
  common::math::Vec2d rear_center;
  std::vector<common::math::Vec2d> polygon;
  std::vector<common::math::Vec2d> opening_edge;
};

Status BuildParkingSlot(const ParkingSlotInput& input, ParkingSlot* slot);

}  // namespace open_space_planning
}  // namespace apollo
