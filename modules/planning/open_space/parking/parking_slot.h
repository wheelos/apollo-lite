/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <array>
#include <string>
#include <vector>

#include "modules/common/math/vec2d.h"

namespace apollo {
namespace planning {
namespace parking {

enum class ParkingSlotType {
  kUnknown = 0,
  kParallel = 1,
  kPerpendicular = 2,
  kAngled = 3,
};

enum class ParkingApproach {
  kUnknown = 0,
  kHeadIn = 1,
  kTailIn = 2,
};

struct ParkingSlotCorners {
  common::math::Vec2d left_top;
  common::math::Vec2d left_down;
  common::math::Vec2d right_down;
  common::math::Vec2d right_top;

  std::array<common::math::Vec2d, 4> AsArray() const;
};

struct ParkingSlot {
  std::string id;
  ParkingSlotType type = ParkingSlotType::kUnknown;
  double raw_heading = 0.0;
  double heading = 0.0;
  double lane_heading = 0.0;
  double lane_s = 0.0;
  double lane_l = 0.0;
  bool on_left_lane_side = false;
  double width = 0.0;
  double depth = 0.0;
  common::math::Vec2d center;
  common::math::Vec2d opening_center;
  common::math::Vec2d rear_center;
  ParkingSlotCorners corners;
  std::vector<common::math::Vec2d> polygon;

  std::array<common::math::Vec2d, 4> Vertices() const;
};

ParkingSlotType InferParkingSlotType(double slot_heading, double lane_heading);

const char* ParkingSlotTypeName(ParkingSlotType type);

bool NormalizeParkingSlot(const std::vector<common::math::Vec2d>& polygon_points,
                          const std::string& slot_id, double slot_heading,
                          double lane_heading, double lane_l,
                          ParkingSlot* parking_slot, std::string* error);

ParkingSlot TransformParkingSlot(const ParkingSlot& parking_slot,
                                 const common::math::Vec2d& origin_point,
                                 double origin_heading);

}  // namespace parking
}  // namespace planning
}  // namespace apollo
