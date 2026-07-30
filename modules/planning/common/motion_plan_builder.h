#pragma once

#include <string>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/motion_execution.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/common/planning_semantics.h"
#include "modules/planning/planning_runtime_context.h"

namespace apollo {
namespace planning {

struct MotionPlanBuildResult {
  bool has_directive = false;
  MotionDirective directive;
  std::string reason;
};

class MotionPlanBuilder {
 public:
  explicit MotionPlanBuilder(std::string producer_epoch);

  MotionPlanBuildResult Build(
      const PlanningCoordinatorState& state,
      const PlanningSemanticSummary& semantics,
      const canbus::Chassis& chassis,
      const localization::LocalizationEstimate& localization,
      const ADCTrajectory& trajectory, double now_sec);

  void ObserveControlStatus(const MotionExecutionStatus& status,
                            MotionDirectiveScope scope);
  void SetProducerEpoch(std::string producer_epoch);

 private:
  bool BuildCommand(const PlanningCoordinatorState& state,
                    const PlanningSemanticSummary& semantics,
                    const canbus::Chassis& chassis,
                    const localization::LocalizationEstimate& localization,
                    const ADCTrajectory& trajectory, double now_sec,
                    MotionExecutionCommand* command,
                    std::string* reason);
  bool BuildIdleHold(
      const canbus::Chassis& chassis,
      const localization::LocalizationEstimate& localization,
      double now_sec, MotionExecutionCommand* command,
      std::string* reason);
  bool BuildStoppingCommand(
      const canbus::Chassis& chassis,
      const localization::LocalizationEstimate& localization,
      double now_sec, MotionExecutionCommand* command,
      std::string* reason);
  void PopulateCommonCommand(
      const canbus::Chassis& chassis,
      const localization::LocalizationEstimate& localization,
      double now_sec, MotionExecutionCommand* command) const;

  std::string producer_epoch_;
  uint64_t next_revision_ = 1;
  MotionCommandIdentity active_identity_;
  MotionCommandIdentity pending_identity_;
  MissionCommandIdentity active_parent_;
  MissionCommandIdentity fenced_parent_;
  MotionDirectiveScope active_scope_ = MOTION_SCOPE_UNKNOWN;
  bool cancellation_fenced_ = false;
  bool pending_cancel_ = false;
  bool stopping_requested_ = false;
};

}  // namespace planning
}  // namespace apollo
