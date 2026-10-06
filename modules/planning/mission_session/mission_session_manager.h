#pragma once

#include <string>
#include <unordered_map>

#include "wheelos_msgs/localization_msgs/localization.pb.h"
#include "wheelos_msgs/planning_msgs/mission_directive.pb.h"
#include "wheelos_msgs/planning_msgs/motion_execution.pb.h"

namespace apollo {
namespace planning {

enum class MissionAdmissionCode {
  kAccepted = 0,
  kDuplicate,
  kInvalidDirective,
  kInvalidIdentity,
  kInvalidPlan,
  kInvalidStart,
  kReplay,
  kBusy,
  kCasMismatch,
  kInvalidTransition,
  kUnsupportedOperation,
};

struct MissionAdmissionResult {
  bool accepted = false;
  MissionAdmissionCode code = MissionAdmissionCode::kInvalidDirective;
  std::string reason;
};

struct MissionGuidance {
  MissionCommandIdentity identity;
  MissionPlan plan;
  MissionStartSnapshot accepted_start;
  MissionSessionPhase phase = MISSION_PHASE_UNKNOWN;
  MissionRouteContext route;
  MissionSessionState state = MISSION_SESSION_UNKNOWN;
  bool cancellation_fenced = false;
};

class MissionSessionManager {
 public:
  MissionAdmissionResult Apply(
      const MissionDirective& directive,
      const localization::LocalizationEstimate& localization, double now_sec);

  MissionAdmissionResult MarkExecuting();
  MissionAdmissionResult UpdateRoute(
      const MissionCommandIdentity& expected_identity,
      const MissionRouteContext& route);
  MissionAdmissionResult AdvancePhase(
      const MissionCommandIdentity& expected_identity,
      MissionSessionPhase next_phase);
  MissionAdmissionResult Suspend(const std::string& reason);
  MissionAdmissionResult Resume();
  MissionAdmissionResult BeginCompleting();
  MissionAdmissionResult Complete(
      const MotionTerminalEvidence& terminal_evidence);
  MissionAdmissionResult ConfirmCancellation(
      const MotionTerminalEvidence& terminal_evidence);
  MissionAdmissionResult Fail(const std::string& reason);

  bool HasActiveSession() const;
  PlanningCommand BuildPlanningCommand() const;
  const MissionGuidance& guidance() const { return guidance_; }
  const std::string& reason() const { return reason_; }
  const MissionCommandIdentity& last_accepted_directive_identity() const {
    return last_accepted_directive_identity_;
  }

 private:
  MissionAdmissionResult ApplyActivate(
      const MissionDirective& directive,
      const localization::LocalizationEstimate& localization, double now_sec);
  MissionAdmissionResult ApplyReplace(
      const MissionDirective& directive,
      const localization::LocalizationEstimate& localization, double now_sec);
  MissionAdmissionResult ApplyCancel(const MissionDirective& directive);
  MissionAdmissionResult AcceptPlan(
      const MissionCommandIdentity& identity, const MissionPlan& plan,
      const localization::LocalizationEstimate& localization, double now_sec);
  MissionAdmissionResult BuildStartSnapshot(
      const MissionStart& start,
      const localization::LocalizationEstimate& localization, double now_sec,
      MissionStartSnapshot* snapshot) const;
  MissionAdmissionResult ValidateDirectiveIdentity(
      const MissionCommandIdentity& identity) const;
  MissionAdmissionResult ValidatePlan(const MissionPlan& plan) const;
  MissionAdmissionResult Transition(MissionSessionState state,
                                    const std::string& reason);
  bool ValidateHoldEvidence(const MotionTerminalEvidence& evidence,
                            std::string* reason) const;
  bool IsExactIdentity(const MissionCommandIdentity& lhs,
                       const MissionCommandIdentity& rhs) const;
  std::string CommandKey(const MissionCommandIdentity& identity) const;
  bool IsTerminal(MissionSessionState state) const;
  bool IsPhaseTransitionAllowed(MissionSessionPhase from,
                                MissionSessionPhase to) const;

  MissionGuidance guidance_;
  std::string reason_;
  std::unordered_map<std::string, uint64_t> highest_revision_by_command_;
  MissionCommandIdentity last_accepted_directive_identity_;
  MissionDirective last_accepted_directive_;
};

}  // namespace planning
}  // namespace apollo
