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

#include "wheelos_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/open_space/parking/parking_roi_geometry.h"
#include "modules/planning/open_space/parking/parking_slot.h"
#include "modules/planning/proto/open_space_task_config.pb.h"

namespace apollo {
namespace planning {
namespace parking {

struct ParkingPoseCandidate {
  ParkingApproach approach = ParkingApproach::kUnknown;
  std::vector<double> end_pose;
  bool was_probed = false;
  bool feasible = false;
  double score = 0.0;
  double path_length = 0.0;
  double reverse_distance = 0.0;
  double min_clearance = 0.0;
  double aisle_width = 0.0;
  int gear_switch_count = 0;
  int collision_path_index = -1;
  int collision_boundary_index = -1;
  std::string rejection_reason;
  std::vector<double> collision_pose;
  common::math::Vec2d collision_boundary_start;
  common::math::Vec2d collision_boundary_end;
};

struct ParkingPoseSelection {
  std::vector<ParkingPoseCandidate> candidates;
  int selected_index = -1;

  bool has_feasible_candidate() const { return selected_index >= 0; }
  const ParkingPoseCandidate& selected() const {
    return candidates[static_cast<std::size_t>(selected_index)];
  }
};

ParkingApproach ResolveParkingApproachPreference(
    const OpenSpaceRoiDeciderConfig& config);

const char* ParkingApproachName(ParkingApproach approach);

class ParkingPoseSelector {
 public:
  explicit ParkingPoseSelector(const OpenSpaceRoiDeciderConfig& config);

  ParkingPoseSelection Select(
      const ParkingSlot& normalized_slot,
      const ParkingRoiGeometry& roi_geometry,
      const apollo::common::VehicleParam& vehicle_param,
      const common::math::Vec2d& vehicle_position,
      double vehicle_heading) const;

 private:
  OpenSpaceRoiDeciderConfig config_;
};

}  // namespace parking
}  // namespace planning
}  // namespace apollo
