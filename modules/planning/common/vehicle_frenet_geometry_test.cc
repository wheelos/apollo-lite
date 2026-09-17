// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "modules/planning/common/vehicle_frenet_geometry.h"

#include <memory>

#include "gtest/gtest.h"
#include "modules/common/vehicle_state/vehicle_description.h"
#include "wheelos_msgs/config_msgs/vehicle_config.pb.h"

namespace apollo {
namespace planning {

class VehicleFrenetGeometryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_.mutable_vehicle_param()->set_wheel_base(2.8);
    config_.mutable_vehicle_param()->set_length(4.8);
    config_.mutable_vehicle_param()->set_width(2.0);
    config_.mutable_vehicle_param()->set_front_edge_to_center(3.8);
    config_.mutable_vehicle_param()->set_back_edge_to_center(1.0);
    config_.mutable_vehicle_param()->set_left_edge_to_center(1.0);
    config_.mutable_vehicle_param()->set_right_edge_to_center(1.0);
    geometry_ = common::VehicleGeometryModel(
        common::VehicleDescription(config_));
    frenet_geometry_ =
        std::make_unique<VehicleFrenetGeometry>(geometry_);
  }

  common::VehicleConfig config_;
  common::VehicleGeometryModel geometry_;
  std::unique_ptr<VehicleFrenetGeometry> frenet_geometry_;
};

TEST_F(VehicleFrenetGeometryTest, ComputesForwardAndReverseStopReference) {
  EXPECT_NEAR(
      frenet_geometry_->ComputeStopReferenceS(
          100.0, 1.0,
          common::TravelDirection::TRAVEL_DIRECTION_FORWARD,
          common::ReferencePoint::REAR_AXLE_CENTER),
      95.2, 1e-6);
  EXPECT_NEAR(
      frenet_geometry_->ComputeStopReferenceS(
          20.0, 0.5,
          common::TravelDirection::TRAVEL_DIRECTION_REVERSE,
          common::ReferencePoint::REAR_AXLE_CENTER),
      21.5, 1e-6);
}

TEST_F(VehicleFrenetGeometryTest, ComputesOccupancyRanges) {
  const auto s_range = frenet_geometry_->GetOccupancySRange(
      50.0, common::ReferencePoint::REAR_AXLE_CENTER);
  EXPECT_NEAR(s_range.first, 49.0, 1e-6);
  EXPECT_NEAR(s_range.second, 53.8, 1e-6);

  const auto l_range = frenet_geometry_->GetOccupancyLRange(2.0);
  EXPECT_NEAR(l_range.first, 1.0, 1e-6);
  EXPECT_NEAR(l_range.second, 3.0, 1e-6);
}

}  // namespace planning
}  // namespace apollo
