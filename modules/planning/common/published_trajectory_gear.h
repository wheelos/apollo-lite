#pragma once

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"

namespace apollo {
namespace planning {

struct PublishedGearInput {
  PlanningMode mode = MODE_UNKNOWN;
  const canbus::Chassis* chassis = nullptr;
  canbus::Chassis::GearPosition segment_gear = canbus::Chassis::GEAR_NONE;
  canbus::Chassis::GearPosition last_published_gear =
      canbus::Chassis::GEAR_NONE;
  bool prefer_last_published_gear = false;
  bool allow_keep_chassis_gear = false;
};

bool HasDirectionalGear(canbus::Chassis::GearPosition gear);

canbus::Chassis::GearPosition ResolvePublishedGear(
    const PublishedGearInput& input);

}  // namespace planning
}  // namespace apollo
