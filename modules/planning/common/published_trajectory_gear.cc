#include "modules/planning/common/published_trajectory_gear.h"

namespace apollo {
namespace planning {

bool HasDirectionalGear(const canbus::Chassis::GearPosition gear) {
  return gear == canbus::Chassis::GEAR_DRIVE ||
         gear == canbus::Chassis::GEAR_REVERSE;
}

canbus::Chassis::GearPosition ResolvePublishedGear(
    const PublishedGearInput& input) {
  if (HasDirectionalGear(input.segment_gear)) {
    return input.segment_gear;
  }

  if (input.mode == MODE_OPEN_SPACE && input.prefer_last_published_gear &&
      HasDirectionalGear(input.last_published_gear)) {
    return input.last_published_gear;
  }

  if ((input.mode == MODE_SAFETY_HOLD || input.allow_keep_chassis_gear) &&
      input.chassis != nullptr &&
      HasDirectionalGear(input.chassis->gear_location())) {
    return input.chassis->gear_location();
  }

  return canbus::Chassis::GEAR_DRIVE;
}

}  // namespace planning
}  // namespace apollo
