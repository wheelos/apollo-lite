#pragma once

#include <string>
#include <unordered_map>

#include "modules/common_msgs/planning_msgs/motion_execution.pb.h"
#include "modules/control/common/motion_execution_validator.h"

namespace apollo {
namespace control {

struct MotionExecutionVehicleState {
  common::PointENU position;
  double heading = 0.0;
  canbus::Chassis::GearPosition gear = canbus::Chassis::GEAR_NONE;
  double speed_mps = 0.0;
  double timestamp_sec = 0.0;
  std::string reference_frame_id;
};

class MotionExecutionManager {
 public:
  explicit MotionExecutionManager(MotionExecutionValidator validator);

  planning::MotionExecutionStatus Apply(
      const planning::MotionDirective& directive, double now_sec);
  planning::MotionExecutionStatus Submit(
      const planning::MotionExecutionCommand& command, double now_sec);
  planning::MotionExecutionStatus Arm(double now_sec);
  planning::MotionExecutionStatus Start(
      const MotionExecutionVehicleState& vehicle_state, double now_sec);
  planning::MotionExecutionStatus ConfirmExecutorRevoked(
      double now_sec, const std::string& reason);
  planning::MotionExecutionStatus Succeed(double now_sec,
                                          const std::string& reason);
  planning::MotionExecutionStatus Fail(double now_sec,
                                       const std::string& reason);
  planning::MotionExecutionStatus EnterHolding(double now_sec,
                                               const std::string& reason);
  planning::MotionExecutionStatus Tick(double now_sec);

  const planning::MotionExecutionCommand* active_command() const;
  const planning::MotionExecutionStatus& status() const { return status_; }
  const planning::MotionExecutionStatus& last_terminal_status() const {
    return last_terminal_status_;
  }

 private:
  bool HasActiveCommand() const;
  bool IsExecuting() const;
  bool IsTerminal(planning::MotionExecutionState state) const;
  planning::MotionExecutionStatus Reject(
      const planning::MotionExecutionCommand& command, double now_sec,
      planning::MotionCommandRejectReason reject_reason,
      const std::string& reason) const;
  planning::MotionExecutionStatus InvalidTransition(
      double now_sec, const std::string& reason) const;
  void SetState(planning::MotionExecutionState state, double now_sec,
                const std::string& reason);
  void InitializeStatus(const planning::MotionExecutionCommand& command,
                        double now_sec,
                        planning::MotionExecutionType execution_type,
                        const planning::MissionCommandIdentity* parent);
  planning::MotionExecutionStatus SubmitCommand(
      const planning::MotionExecutionCommand& command,
      const planning::MissionCommandIdentity* parent, double now_sec);
  std::string CommandKey(
      const planning::MotionExecutionCommand& command) const;
  std::string ParentKey(
      const planning::MissionCommandIdentity& identity) const;
  bool IsExactIdentity(const planning::MotionCommandIdentity& lhs,
                       const planning::MotionCommandIdentity& rhs) const;
  bool IsExactParentIdentity(
      const planning::MissionCommandIdentity& lhs,
      const planning::MissionCommandIdentity& rhs) const;
  bool IsValidParentIdentity(
      const planning::MissionCommandIdentity& identity) const;
  bool IsPlanningIdleHold(
      const planning::MotionExecutionCommand& command) const;
  bool IsParentFenced(
      const planning::MissionCommandIdentity& identity) const;
  bool StartConditionMatches(
      const planning::MotionExecutionCommand& command,
      const MotionExecutionVehicleState& vehicle_state,
      double now_sec, std::string* reason) const;
  void RecordRevision(const planning::MotionExecutionCommand& command);
  void RemoveExpiredRevisionRecords(double now_sec);
  bool AdvanceDeadlines(double now_sec);

  struct RevisionRecord {
    uint64_t revision = 0;
    double expiry_time_sec = 0.0;
  };

  MotionExecutionValidator validator_;
  planning::MotionExecutionCommand active_command_;
  planning::MissionCommandIdentity active_parent_identity_;
  planning::MotionExecutionCommand pending_replacement_;
  planning::MissionCommandIdentity pending_parent_identity_;
  planning::MotionExecutionType pending_execution_type_ =
      planning::MOTION_EXECUTION_TYPE_UNKNOWN;
  planning::MotionExecutionStatus status_;
  planning::MotionExecutionStatus last_terminal_status_;
  std::unordered_map<std::string, RevisionRecord>
      highest_revision_by_command_;
  std::unordered_map<std::string, uint64_t>
      highest_fenced_revision_by_parent_;
  double last_event_time_sec_ = -1.0;
};

}  // namespace control
}  // namespace apollo
