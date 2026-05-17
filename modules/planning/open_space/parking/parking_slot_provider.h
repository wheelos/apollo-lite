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

#include <string>
#include <vector>

#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/open_space/parking/parking_slot.h"

namespace apollo {
namespace planning {
namespace parking {

struct PerceptionParkingSlotInput {
  std::string id;
  ParkingSlotCorners corners;
  std::vector<common::math::Vec2d> polygon;
  bool has_slot_heading = false;
  double slot_heading = 0.0;
  bool has_lane_context = false;
  double lane_heading = 0.0;
  double lane_s = 0.0;
  double lane_l = 0.0;
};

class ParkingSlotProvider {
 public:
  bool BuildFromMap(const hdmap::ParkingSpaceInfoConstPtr& parking_space_info,
                    const hdmap::Path& nearby_path,
                    const common::math::Vec2d& reference_point,
                    ParkingSlot* parking_slot,
                    std::string* error = nullptr) const;

  bool BuildFromPerception(const PerceptionParkingSlotInput& input,
                           ParkingSlot* parking_slot,
                           std::string* error = nullptr) const;
};

}  // namespace parking
}  // namespace planning
}  // namespace apollo
