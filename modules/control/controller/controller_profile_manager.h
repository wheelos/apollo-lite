#pragma once

#include <array>
#include <memory>
#include <optional>

#include "wheelos_msgs/control_msgs/control_runtime_status.pb.h"

#include "modules/control/controller/controller_agent.h"

namespace apollo {
namespace control {

class ControllerProfileManager {
 public:
  common::Status Init(std::shared_ptr<DependencyInjector> injector,
                      const ControlConf& conf, double stopped_speed_mps,
                      double max_steer_rate_pct_per_sec,
                      double max_acceleration_mps2,
                      double max_deceleration_mps2);
  common::Status Bind(ControllerParameterProfile profile,
                      const planning::MotionExecutionCommand& command,
                      canbus::Chassis::GearPosition gear, double speed_mps);
  common::Status ComputeControlCommand(
      const localization::LocalizationEstimate* localization,
      const canbus::Chassis* chassis, const planning::ADCTrajectory* trajectory,
      ControlCommand* command);
  common::Status Reset();
  bool SupportsSpatialPrimitives() const;
  const ControllerSelectionStatus& status() const { return status_; }

  static common::Status BuildConfigurations(
      const ControlConf& base, double stopped_speed_mps,
      std::array<std::optional<ControlConf>, 3>* configurations);

 private:
  struct Entry {
    ControlConf conf;
    std::unique_ptr<ControllerAgent> agent;
    std::optional<double> max_entry_speed_mps;
  };

  common::Status SelectionError(const std::string& reason);
  std::array<std::unique_ptr<Entry>, 3> entries_;
  size_t selected_ = 0;
  bool enabled_ = false;
  bool bound_ = false;
  double stopped_speed_mps_ = 0.0;
  double max_switch_speed_mps_ = 0.0;
  double max_steer_rate_pct_per_sec_ = 0.0;
  double max_acceleration_mps2_ = 0.0;
  double max_deceleration_mps2_ = 0.0;
  planning::MotionConstraints motion_constraints_;
  bool have_previous_steer_ = false;
  double previous_steer_ = 0.0;
  canbus::Chassis::GearPosition gear_ = canbus::Chassis::GEAR_NONE;
  ControllerSelectionStatus status_;
};

const char* ControllerProfileName(ControllerParameterProfile profile);

}  // namespace control
}  // namespace apollo
