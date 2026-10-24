#pragma once

#include <string>

#include "wheelos_msgs/planning_msgs/motion_execution.pb.h"

namespace apollo {
namespace planning {

class Frame;

// Uses the chosen path's admissible boundary or the open-space ROI, never
// trajectory extrema. Collision validation remains the owning planner's duty.
bool BuildMotionEnvelope(const Frame& frame, MotionSpatialEnvelope* envelope,
                         std::string* reason);

}  // namespace planning
}  // namespace apollo
