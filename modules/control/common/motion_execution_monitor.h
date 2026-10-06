#pragma once

#include <string>

#include "modules/control/common/motion_primitive_executor.h"

namespace apollo {
namespace control {

// Measured completion evidence for trajectory and authorized standstill hold.
// Admission/schema validation remains the manager's responsibility.
class MotionExecutionMonitor {
 public:
  MotionExecutionMonitor(double footprint_radius_m, double spatial_step_m,
                         double max_state_age_sec);
  MotionPrimitiveResult Update(
      const planning::MotionExecutionCommand& command,
      const common::VehicleState& state, const std::string& frame_id,
      double now_sec);
  void Reset();

 private:
  double radius_;
  double step_;
  double max_age_;
  std::string identity_;
  double previous_x_ = 0.0;
  double previous_y_ = 0.0;
  double previous_stamp_ = 0.0;
  double previous_time_ = 0.0;
  double settled_since_ = -1.0;
  double admitted_at_ = 0.0;
  bool holding_ = false;
};

}  // namespace control
}  // namespace apollo
