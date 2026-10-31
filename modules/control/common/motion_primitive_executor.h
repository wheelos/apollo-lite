#pragma once

#include <string>
#include <vector>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "wheelos_msgs/planning_msgs/motion_execution.pb.h"
#include "wheelos_msgs/planning_msgs/planning.pb.h"

namespace apollo {
namespace control {

bool IsMotionSweptFootprintInside(
    double from_x, double from_y, double to_x, double to_y,
    const planning::MotionSpatialEnvelope& envelope, double radius,
    double spatial_step);

// All limits are supplied by the platform, not inferred from a mission.
// The footprint disk is centered at the canonical VehicleState reference point.
struct MotionPrimitiveModel {
  double max_speed_mps = 0.0;
  double max_acceleration_mps2 = 0.0;
  double max_deceleration_mps2 = 0.0;
  double max_jerk_mps3 = 0.0;
  double max_curvature_per_m = 0.0;
  double max_curvature_derivative_per_m2 = 0.0;
  double max_lateral_acceleration_mps2 = 0.0;
  double footprint_radius_m = 0.0;
  double sample_period_sec = 0.0;
  double spatial_step_m = 0.0;
  double max_state_age_sec = 0.0;
  double max_tracking_heading_error_rad = 0.0;
};

struct MotionPrimitiveResult {
  bool accepted = false;
  bool completed = false;
  bool hold_confirmed = false;
  planning::MotionCommandRejectReason reject_reason =
      planning::MOTION_REJECT_NONE;
  std::string reason;
  double position_error_m = 0.0;
  double heading_error_rad = 0.0;
  double observed_at_sec = 0.0;
  double settled_duration_sec = 0.0;
  std::string reference_frame_id;
  planning::ADCTrajectory reference;
};

// A spatial-reference executor, not a route/parking planner. Curved requests
// require planning-owned reference_path guidance. Bare poses and legacy
// centerlines are supported only when straight, aligned and unidirectional.
// A rejected result has no usable reference: the caller must apply its safety
// fallback. Completion is measured from fresh VehicleState, never the reference.
class MotionPrimitiveExecutor {
 public:
  explicit MotionPrimitiveExecutor(const MotionPrimitiveModel& model);
  bool IsAvailable() const;
  MotionPrimitiveResult Update(
      const planning::MotionExecutionCommand& command,
      const common::VehicleState& state, const std::string& state_frame_id,
      double now_sec);
  void Reset();

 private:
  bool BuildPath(const planning::MotionExecutionCommand& command,
                 std::string* reason);
  MotionPrimitiveModel model_;
  std::string command_bytes_;
  std::string command_identity_;
  std::vector<common::PathPoint> path_;
  double direction_ = 1.0;
  double started_at_ = 0.0;
  double last_time_ = 0.0;
  double last_state_time_ = 0.0;
  double settled_since_ = -1.0;
  double last_progress_ = 0.0;
  double reference_duration_ = 0.0;
  double traveled_distance_ = 0.0;
  double previous_x_ = 0.0;
  double previous_y_ = 0.0;
  bool failed_ = false;
  unsigned int sequence_ = 0;
};

}  // namespace control
}  // namespace apollo
