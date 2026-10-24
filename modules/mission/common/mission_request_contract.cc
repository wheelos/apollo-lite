#include "modules/mission/common/mission_request_contract.h"

namespace apollo {
namespace mission {

MissionContractCapabilities CurrentMissionContractCapabilities() {
  MissionContractCapabilities capabilities;
  capabilities.set_request_contract_version(kMissionRequestContractVersion);
  capabilities.set_complete_inventory(true);
  for (int operation = MISSION_OPERATION_SUBMIT_TASK;
       operation <= MissionRequestOperation_MAX; ++operation) {
    if (MissionRequestOperation_IsValid(operation)) {
      capabilities.add_declared_operation(
          static_cast<MissionRequestOperation>(operation));
    }
  }
  for (int task = planning::MISSION_TASK_A_TO_B;
       task <= planning::MissionTaskType_MAX; ++task) {
    if (planning::MissionTaskType_IsValid(task)) {
      capabilities.add_declared_task_type(
          static_cast<planning::MissionTaskType>(task));
    }
  }
  capabilities.add_implemented_operation(MISSION_OPERATION_SUBMIT_TASK);
  capabilities.add_implemented_operation(MISSION_OPERATION_REPLACE_TASK);
  capabilities.add_implemented_operation(MISSION_OPERATION_CANCEL_TASK);
  capabilities.add_implemented_operation(MISSION_OPERATION_QUERY);
  capabilities.add_implemented_operation(MISSION_OPERATION_SAFETY_STOP);
  capabilities.add_implemented_operation(MISSION_OPERATION_SAFETY_RESET);
  capabilities.set_typed_dispatcher_implemented(true);
  capabilities.set_durable_request_ledger_implemented(true);
  capabilities.set_independent_safety_ingress_implemented(true);
  capabilities.set_restart_reauthorization_implemented(false);
  capabilities.set_angular_motion_evidence_implemented(false);
  return capabilities;
}

std::optional<MissionRequestResult> RejectUnavailableTypedRequest(
    const MissionRequest& request) {
  if (request.operation_case() == MissionRequest::OPERATION_NOT_SET &&
      !request.has_identity() && !request.has_contract_version()) {
    return std::nullopt;
  }
  MissionRequestResult result;
  if (request.has_identity()) {
    result.mutable_identity()->CopyFrom(request.identity());
  }
  if (request.has_mission_id()) {
    result.set_requested_mission_id(request.mission_id());
  }
  result.set_outcome(MISSION_REQUEST_REJECTED);
  result.set_retry_disposition(MISSION_REQUEST_DO_NOT_RETRY);
  result.set_durably_committed(false);
  const planning::MissionPlan* plan =
      request.has_submit_task() ? &request.submit_task().plan()
      : request.has_replace_task() ? &request.replace_task().plan()
                                  : nullptr;
  const control::SafetyOperationIdentity* safety_identity =
      request.has_safety_stop() ? &request.safety_stop().identity()
      : request.has_safety_reset() ? &request.safety_reset().identity()
                                   : nullptr;
  const bool internal_plan_fields =
      plan != nullptr &&
      ((plan->has_preferred_mode() &&
        (!planning::PlanningMode_IsValid(plan->preferred_mode()) ||
         plan->preferred_mode() == planning::MODE_UNKNOWN ||
         plan->preferred_mode() == planning::MODE_SAFETY_HOLD)) ||
       plan->has_domain_policy() || plan->has_open_space() ||
       plan->has_budget_authorization_id());
  const bool mismatched_safety_identity =
      safety_identity != nullptr &&
      (safety_identity->requester_epoch() != request.identity().client_epoch() ||
       safety_identity->request_id() != request.identity().request_id());
  if (!request.has_identity() || request.identity().client_epoch().empty() ||
      request.identity().request_id().empty() ||
      request.identity().client_epoch().size() > 256 ||
      request.identity().request_id().size() > 256 ||
      !request.has_contract_version() ||
      request.operation_case() == MissionRequest::OPERATION_NOT_SET ||
      request.has_task_name() || request.has_enable_loop() ||
      request.parameters_size() != 0 || request.waypoints_size() != 0 ||
      internal_plan_fields || mismatched_safety_identity) {
    result.set_code(MISSION_REQUEST_INVALID_INPUT);
    result.set_reason("typed request requires bounded client identity, version "
                      "and one operation without legacy/internal fields; "
                      "safety identities must match");
  } else if (request.contract_version() != kMissionRequestContractVersion) {
    result.set_code(MISSION_REQUEST_UNSUPPORTED_VERSION);
    result.set_reason("typed request contract version is unsupported");
  } else {
    result.set_code(MISSION_REQUEST_UNSUPPORTED_OPERATION);
    result.set_reason("typed request dispatcher is unavailable; request was "
                      "not executed, safety stop was not enforced");
  }
  return result;
}

planning::MissionTaskType MissionTaskForScene(
    planning::PlanningSceneType scene) {
  switch (scene) {
    case planning::SCENE_LANE_CRUISE:
      return planning::MISSION_TASK_A_TO_B;
    case planning::SCENE_PARK_IN:
      return planning::MISSION_TASK_PARK_IN;
    case planning::SCENE_PULL_OUT:
      return planning::MISSION_TASK_PARK_OUT;
    case planning::SCENE_PULL_OVER:
      return planning::MISSION_TASK_PULL_OVER;
    case planning::SCENE_DOCK:
      return planning::MISSION_TASK_DOCK;
    case planning::SCENE_SUMMON:
      return planning::MISSION_TASK_SUMMON;
    default:
      return planning::MISSION_TASK_UNKNOWN;
  }
}

}  // namespace mission
}  // namespace apollo
