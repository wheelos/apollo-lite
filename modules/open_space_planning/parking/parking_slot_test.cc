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

#include "modules/open_space_planning/parking/parking_slot.h"

#include "gtest/gtest.h"

namespace apollo {
namespace open_space_planning {
namespace {

TEST(ParkingSlotTest, BuildsPerpendicularSlotFromExplicitOpeningEdge) {
  ParkingSlotInput input;
  input.id = "slot-a";
  input.corners = {{0.0, 0.0}, {2.5, 0.0}, {2.5, 5.0}, {0.0, 5.0}};
  input.heading = 1.57079632679;
  input.lane_heading = 0.0;
  input.has_lane_heading = true;
  input.opening_edge_index = 0;

  ParkingSlot slot;
  const Status status = BuildParkingSlot(input, &slot);

  ASSERT_TRUE(status.ok()) << status.message();
  EXPECT_EQ(slot.type, ParkingSlotType::kPerpendicular);
  EXPECT_NEAR(slot.width, 2.5, 1e-8);
  EXPECT_NEAR(slot.depth, 5.0, 1e-8);
  EXPECT_NEAR(slot.opening_center.x(), 1.25, 1e-8);
  EXPECT_NEAR(slot.opening_center.y(), 0.0, 1e-8);
}

TEST(ParkingSlotTest, RejectsMissingOpeningEdgeInsteadOfGuessing) {
  ParkingSlotInput input;
  input.corners = {{0.0, 0.0}, {2.5, 0.0}, {2.5, 5.0}, {0.0, 5.0}};
  input.has_lane_heading = true;

  ParkingSlot slot;
  const Status status = BuildParkingSlot(input, &slot);

  EXPECT_FALSE(status.ok());
}

}  // namespace
}  // namespace open_space_planning
}  // namespace apollo
