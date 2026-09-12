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

#include "modules/common/vehicle_state/reference_point_resolver.h"

#include "gtest/gtest.h"

namespace apollo {
namespace common {

TEST(ReferencePointResolverTest, ResolvesFrontAxleFromRearAxleState) {
  VehicleConfig config;
  config.mutable_vehicle_param()->set_wheel_base(2.8);
  ReferencePointResolver resolver{VehicleDescription(config)};

  VehicleState state;
  state.set_x(1.0);
  state.set_y(2.0);
  state.set_heading(0.0);
  state.set_reference_point(REAR_AXLE_CENTER);

  ReferenceState front_axle;
  ASSERT_TRUE(resolver.Resolve(state, FRONT_AXLE_CENTER, &front_axle).ok());
  EXPECT_DOUBLE_EQ(front_axle.x(), 3.8);
  EXPECT_DOUBLE_EQ(front_axle.y(), 2.0);
  EXPECT_EQ(front_axle.reference_point(), FRONT_AXLE_CENTER);
}

TEST(ReferencePointResolverTest, ResolvesUsingSourceReferencePoint) {
  VehicleConfig config;
  config.mutable_vehicle_param()->set_wheel_base(2.8);
  ReferencePointResolver resolver{VehicleDescription(config)};

  VehicleState state;
  state.set_x(3.8);
  state.set_y(2.0);
  state.set_heading(0.0);
  state.set_reference_point(FRONT_AXLE_CENTER);

  ReferenceState rear_axle;
  ASSERT_TRUE(resolver.Resolve(state, REAR_AXLE_CENTER, &rear_axle).ok());
  EXPECT_DOUBLE_EQ(rear_axle.x(), 1.0);
  EXPECT_DOUBLE_EQ(rear_axle.y(), 2.0);
}

TEST(ReferencePointResolverTest, RejectsNullOutput) {
  VehicleConfig config;
  ReferencePointResolver resolver{VehicleDescription(config)};
  VehicleState state;
  EXPECT_FALSE(resolver.Resolve(state, REAR_AXLE_CENTER, nullptr).ok());
}

}  // namespace common
}  // namespace apollo
