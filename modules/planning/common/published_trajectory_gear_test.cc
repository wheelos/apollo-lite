#include "modules/planning/common/published_trajectory_gear.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {
namespace {

TEST(PublishedTrajectoryGearTest, DefaultsLaneModesToDrive) {
  PublishedGearInput input;
  input.mode = MODE_LANE_GRAPH;

  EXPECT_EQ(ResolvePublishedGear(input), canbus::Chassis::GEAR_DRIVE);
}

TEST(PublishedTrajectoryGearTest, UsesExplicitSegmentGearWhenAvailable) {
  PublishedGearInput input;
  input.mode = MODE_OPEN_SPACE;
  input.segment_gear = canbus::Chassis::GEAR_REVERSE;

  EXPECT_EQ(ResolvePublishedGear(input), canbus::Chassis::GEAR_REVERSE);
}

TEST(PublishedTrajectoryGearTest, PreservesLastOpenSpaceGear) {
  PublishedGearInput input;
  input.mode = MODE_OPEN_SPACE;
  input.last_published_gear = canbus::Chassis::GEAR_REVERSE;
  input.prefer_last_published_gear = true;

  EXPECT_EQ(ResolvePublishedGear(input), canbus::Chassis::GEAR_REVERSE);
}

TEST(PublishedTrajectoryGearTest, SafetyHoldCanKeepChassisGear) {
  canbus::Chassis chassis;
  chassis.set_gear_location(canbus::Chassis::GEAR_REVERSE);

  PublishedGearInput input;
  input.mode = MODE_SAFETY_HOLD;
  input.chassis = &chassis;
  input.allow_keep_chassis_gear = true;

  EXPECT_EQ(ResolvePublishedGear(input), canbus::Chassis::GEAR_REVERSE);
}

TEST(PublishedTrajectoryGearTest, LaneModesDoNotInheritReverseChassisGear) {
  canbus::Chassis chassis;
  chassis.set_gear_location(canbus::Chassis::GEAR_REVERSE);

  PublishedGearInput input;
  input.mode = MODE_LANE_GRAPH;
  input.chassis = &chassis;

  EXPECT_EQ(ResolvePublishedGear(input), canbus::Chassis::GEAR_DRIVE);
}

}  // namespace
}  // namespace planning
}  // namespace apollo
