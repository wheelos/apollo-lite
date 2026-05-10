/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/planning/open_space/parking/parking_roi_validator.h"

#include "gtest/gtest.h"

#include "modules/planning/open_space/parking/parking_pose_selector.h"

namespace apollo {
namespace planning {
namespace parking {

using apollo::common::math::Vec2d;

namespace {

ParkingRoiGeometry MakeRectangularGeometry() {
  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(0.0, 0.0), Vec2d(10.0, 0.0), Vec2d(10.0, 6.0),
                            Vec2d(0.0, 6.0)};
  geometry.xy_boundary = {0.0, 10.0, 0.0, 6.0};
  geometry.aisle_width = 5.0;
  for (size_t index = 0; index < geometry.union_polygon.size(); ++index) {
    geometry.boundary_segments.push_back(
        {geometry.union_polygon[index],
         geometry.union_polygon[(index + 1U) % geometry.union_polygon.size()]});
  }
  return geometry;
}

apollo::common::VehicleParam MakeVehicleParam() {
  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(4.933);
  vehicle_param.set_width(2.11);
  vehicle_param.set_back_edge_to_center(1.043);
  vehicle_param.set_front_edge_to_center(3.89);
  vehicle_param.set_wheel_base(2.8448);
  vehicle_param.set_max_steer_angle(8.20304748437);
  vehicle_param.set_steer_ratio(16.0);
  return vehicle_param;
}

apollo::common::VehicleParam Make4p0VehicleParam() {
  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(4.0);
  vehicle_param.set_width(2.11);
  vehicle_param.set_back_edge_to_center(0.8451246705858504);
  vehicle_param.set_front_edge_to_center(3.1548753294141495);
  vehicle_param.set_wheel_base(2.8448);
  vehicle_param.set_max_steer_angle(8.20304748437);
  vehicle_param.set_steer_ratio(16.0);
  return vehicle_param;
}

}  // namespace

TEST(ParkingRoiValidatorTest, ValidateGoalInsideRoi) {
  OpenSpaceRoiDeciderConfig config;
  ParkingRoiValidator validator(config);

  const ParkingRoiGeometry geometry = MakeRectangularGeometry();
  const auto result = validator.Validate(geometry, Vec2d(2.0, 2.0),
                                         {5.8, 3.0, 0.0, 0.0},
                                         MakeVehicleParam());
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.vehicle_inside);
  EXPECT_TRUE(result.goal_inside);
  EXPECT_GT(result.area, 0.0);
}

TEST(ParkingRoiValidatorTest, RejectVehicleOutsideGeometryOnly) {
  OpenSpaceRoiDeciderConfig config;
  ParkingRoiValidator validator(config);

  const ParkingRoiGeometry geometry = MakeRectangularGeometry();
  const auto result = validator.ValidateGeometryOnly(geometry, Vec2d(-1.0, 2.0));
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.vehicle_inside);
  EXPECT_EQ(result.reason, "vehicle outside parking roi");
}

TEST(ParkingRoiValidatorTest, RejectInvalidAisleWidth) {
  OpenSpaceRoiDeciderConfig config;
  ParkingRoiValidator validator(config);

  ParkingRoiGeometry geometry = MakeRectangularGeometry();
  geometry.aisle_width = 0.0;

  const auto result = validator.ValidateGeometryOnly(geometry, Vec2d(2.0, 2.0));
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.vehicle_inside);
  EXPECT_EQ(result.reason, "parking roi aisle width is invalid");
}

TEST(ParkingRoiValidatorTest, RejectGoalOutsideRoi) {
  OpenSpaceRoiDeciderConfig config;
  ParkingRoiValidator validator(config);

  const ParkingRoiGeometry geometry = MakeRectangularGeometry();
  const auto result = validator.Validate(geometry, Vec2d(2.0, 2.0),
                                         {12.0, 3.0, 0.0, 0.0},
                                         MakeVehicleParam());
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.goal_inside);
  EXPECT_EQ(result.reason, "goal vehicle box outside parking envelope");
}

TEST(ParkingRoiValidatorTest, ReportGoalClearanceWithoutRejectingBoundaryNearMiss) {
  OpenSpaceRoiDeciderConfig config;
  config.set_roi_min_goal_clearance(1.0);
  ParkingRoiValidator validator(config);

  const ParkingRoiGeometry geometry = MakeRectangularGeometry();
  const auto result = validator.Validate(geometry, Vec2d(2.0, 2.0),
                                         {5.5, 3.0, 0.0, 0.0},
                                         MakeVehicleParam());
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.goal_inside);
  EXPECT_TRUE(result.reason.empty());
  EXPECT_LT(result.goal_clearance, 1.0);
}

TEST(ParkingRoiValidatorTest, AcceptGoalPointOnConnectorBoundaryWhenBoxFits) {
  OpenSpaceRoiDeciderConfig config;
  config.set_roi_min_goal_clearance(0.2);
  ParkingRoiValidator validator(config);

  ParkingRoiGeometry geometry;
  geometry.union_polygon = {
      Vec2d(-1.9455532117823164, 11.494300328455775),
      Vec2d(-1.9455519393435736, 0.01309520316924695),
      Vec2d(-1.4256269508531647, 0.010907625326331605),
      Vec2d(-3.222852066095146, -4.80492748058604),
      Vec2d(-0.46000651238241863, -4.737384509839769),
      Vec2d(1.4243994951443355, -0.014196710533125101),
      Vec2d(11.993944218812786, -0.04555505140982152),
      Vec2d(12.999098758015514, -0.045313661697779395),
      Vec2d(14.998909534727797, -0.05389314266311995),
      Vec2d(14.957037085426444, 11.874875231575073),
  };
  geometry.aisle_width = 11.535182396872083;

  const auto result = validator.Validate(geometry, Vec2d(0.0, 0.0),
                                         {0.0, 0.0, 1.202460086413434, 0.0},
                                         MakeVehicleParam());
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.goal_inside);
  EXPECT_GT(result.goal_clearance, 0.2);
}

TEST(ParkingRoiValidatorTest, RejectGoalOutsideSlotEnvelopeInsideWideRoi) {
  OpenSpaceRoiDeciderConfig config;
  ParkingRoiValidator validator(config);

  ParkingRoiGeometry geometry = MakeRectangularGeometry();
  geometry.slot_polygon = {Vec2d(3.0, 1.0), Vec2d(7.0, 1.0), Vec2d(7.0, 5.0),
                           Vec2d(3.0, 5.0)};

  const auto result = validator.Validate(geometry, Vec2d(2.0, 2.0),
                                         {8.0, 3.0, 0.0, 0.0},
                                         MakeVehicleParam());
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.goal_inside);
  EXPECT_EQ(result.reason, "goal vehicle box outside parking envelope");
}

TEST(ParkingRoiValidatorTest,
     AcceptGoalWithOpeningOverhangWhenBoxRemainsInsideWideRoi) {
  OpenSpaceRoiDeciderConfig config;
  ParkingRoiValidator validator(config);
  ParkingPoseSelector selector(config);

  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-40.0, -20.0), Vec2d(40.0, -20.0),
                            Vec2d(40.0, 20.0), Vec2d(-40.0, 20.0)};
  geometry.xy_boundary = {-40.0, 40.0, -20.0, 20.0};
  geometry.slot_polygon = {
      Vec2d(-1.4243994947911567, 0.014196710229636977),
      Vec2d(-3.222852066095146, -4.80492748058604),
      Vec2d(-0.46000651238241863, -4.737384509839769),
      Vec2d(1.4243994951443355, -0.014196710533125101),
  };
  geometry.aisle_width = 8.0;
  for (size_t index = 0; index < geometry.union_polygon.size(); ++index) {
    geometry.boundary_segments.push_back(
        {geometry.union_polygon[index],
         geometry.union_polygon[(index + 1U) % geometry.union_polygon.size()]});
  }
  ParkingSlot slot;
  slot.type = ParkingSlotType::kPerpendicular;
  slot.corners.left_top = geometry.slot_polygon[0];
  slot.corners.left_down = geometry.slot_polygon[1];
  slot.corners.right_down = geometry.slot_polygon[2];
  slot.corners.right_top = geometry.slot_polygon[3];
  slot.opening_center = (slot.corners.left_top + slot.corners.right_top) * 0.5;
  slot.rear_center = (slot.corners.left_down + slot.corners.right_down) * 0.5;
  slot.center = (slot.opening_center + slot.rear_center) * 0.5;
  slot.width =
      0.5 * ((slot.corners.right_top - slot.corners.left_top).Length() +
             (slot.corners.right_down - slot.corners.left_down).Length());
  slot.depth =
      0.5 * ((slot.corners.left_down - slot.corners.left_top).Length() +
             (slot.corners.right_down - slot.corners.right_top).Length());
  slot.heading = (slot.rear_center - slot.opening_center).Angle();

  const auto selection =
      selector.Select(slot, geometry, Make4p0VehicleParam(), Vec2d(0.0, 8.0), 0.0);
  ASSERT_TRUE(selection.has_feasible_candidate());

  const auto result = validator.Validate(
      geometry, Vec2d(0.0, 0.0), selection.selected().end_pose,
      Make4p0VehicleParam());
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.goal_inside);
  EXPECT_GT(result.goal_clearance, 0.0);
}

}  // namespace parking
}  // namespace planning
}  // namespace apollo
