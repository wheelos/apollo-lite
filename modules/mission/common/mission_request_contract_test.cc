#include "modules/mission/common/mission_request_contract.h"

#include <vector>

#include "gtest/gtest.h"
#include "google/protobuf/descriptor.h"

namespace apollo {
namespace mission {
namespace {

MissionRequest TypedRequest() {
  MissionRequest request;
  request.mutable_identity()->set_client_epoch("client-epoch");
  request.mutable_identity()->set_request_id("request-1");
  request.set_contract_version(1);
  return request;
}

TEST(MissionRequestContractTest, LegacyRequestsRemainSeparate) {
  MissionRequest request;
  request.set_mission_id("mission-1");
  request.set_task_name("PatrolTask");
  EXPECT_FALSE(RejectUnavailableTypedRequest(request).has_value());
}

TEST(MissionRequestContractTest, EveryTypedOperationRejectsWithoutExecution) {
  std::vector<MissionRequest> requests(11, TypedRequest());
  requests[0].mutable_submit_task();
  requests[1].mutable_replace_task();
  requests[2].mutable_cancel_task();
  requests[3].mutable_suspend_task();
  requests[4].mutable_resume_task();
  requests[5].mutable_query();
  requests[6].mutable_safety_stop();
  requests[7].mutable_safety_reset();
  requests[6].mutable_safety_stop()->mutable_identity()->set_requester_epoch(
      "client-epoch");
  requests[6].mutable_safety_stop()->mutable_identity()->set_request_id(
      "request-1");
  requests[7].mutable_safety_reset()->mutable_identity()->set_requester_epoch(
      "client-epoch");
  requests[7].mutable_safety_reset()->mutable_identity()->set_request_id(
      "request-1");
  requests[8].mutable_acknowledge_recovery();
  requests[9].mutable_retry_task();
  requests[10].mutable_abort_task();
  const auto capabilities = CurrentMissionContractCapabilities();
  const auto* operation = MissionRequest::descriptor()->FindOneofByName(
      "operation");
  ASSERT_NE(operation, nullptr);
  ASSERT_EQ(requests.size(), operation->field_count());
  ASSERT_EQ(capabilities.declared_operation_size(), operation->field_count());
  for (int i = 0; i < operation->field_count(); ++i) {
    EXPECT_EQ(capabilities.declared_operation(i), operation->field(i)->number());
    EXPECT_EQ(requests[i].operation_case(), operation->field(i)->number());
  }
  for (const auto& request : requests) {
    MissionRequest decoded;
    ASSERT_TRUE(decoded.ParseFromString(request.SerializeAsString()));
    EXPECT_EQ(decoded.operation_case(), request.operation_case());
    const auto result = RejectUnavailableTypedRequest(decoded);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->identity().SerializeAsString(),
              request.identity().SerializeAsString());
    EXPECT_EQ(result->outcome(), MISSION_REQUEST_REJECTED);
    EXPECT_EQ(result->code(), MISSION_REQUEST_UNSUPPORTED_OPERATION);
    EXPECT_EQ(result->retry_disposition(), MISSION_REQUEST_DO_NOT_RETRY);
    EXPECT_FALSE(result->durably_committed());
    EXPECT_FALSE(result->has_commit_sequence());
    EXPECT_FALSE(result->has_current_task_identity());
  }
}

TEST(MissionRequestContractTest, InventoryMatchesImplementedIngress) {
  MissionContractCapabilities capabilities;
  EXPECT_FALSE(capabilities.has_complete_inventory());
  EXPECT_FALSE(capabilities.has_independent_safety_ingress_implemented());
  ASSERT_TRUE(capabilities.ParseFromString(
      CurrentMissionContractCapabilities().SerializeAsString()));
  EXPECT_EQ(capabilities.request_contract_version(),
            kMissionRequestContractVersion);
  ASSERT_TRUE(capabilities.complete_inventory());
  EXPECT_EQ(capabilities.implemented_operation_size(), 6);
  EXPECT_EQ(capabilities.implemented_task_type_size(), 0);
  ASSERT_TRUE(capabilities.has_typed_dispatcher_implemented());
  EXPECT_TRUE(capabilities.typed_dispatcher_implemented());
  ASSERT_TRUE(capabilities.has_durable_request_ledger_implemented());
  EXPECT_TRUE(capabilities.durable_request_ledger_implemented());
  ASSERT_TRUE(capabilities.has_independent_safety_ingress_implemented());
  EXPECT_TRUE(capabilities.independent_safety_ingress_implemented());
  ASSERT_TRUE(capabilities.has_restart_reauthorization_implemented());
  EXPECT_FALSE(capabilities.restart_reauthorization_implemented());
  ASSERT_TRUE(capabilities.has_angular_motion_evidence_implemented());
  EXPECT_FALSE(capabilities.angular_motion_evidence_implemented());
  const auto* tasks = planning::MissionTaskType_descriptor();
  ASSERT_EQ(capabilities.declared_task_type_size(), tasks->value_count() - 1);
  for (int i = 1; i < tasks->value_count(); ++i) {
    EXPECT_EQ(capabilities.declared_task_type(i - 1), tasks->value(i)->number());
  }
}

TEST(MissionRequestContractTest, TypedMarkersCannotFallBackToLegacyBt) {
  auto request = TypedRequest();
  request.set_task_name("PatrolTask");
  auto result = RejectUnavailableTypedRequest(request);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->code(), MISSION_REQUEST_INVALID_INPUT);
  request.mutable_submit_task();
  result = RejectUnavailableTypedRequest(request);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->code(), MISSION_REQUEST_INVALID_INPUT);
}

TEST(MissionRequestContractTest, InvalidIdentityAndVersionRejectExplicitly) {
  auto request = TypedRequest();
  request.mutable_query();
  request.mutable_identity()->clear_request_id();
  EXPECT_EQ(RejectUnavailableTypedRequest(request)->code(),
            MISSION_REQUEST_INVALID_INPUT);
  request.mutable_identity()->set_request_id("request-1");
  request.set_contract_version(2);
  EXPECT_EQ(RejectUnavailableTypedRequest(request)->code(),
            MISSION_REQUEST_UNSUPPORTED_VERSION);
}

TEST(MissionRequestContractTest, EmergencyIsNotAnOrdinaryMissionScene) {
  EXPECT_EQ(MissionTaskForScene(planning::SCENE_EMERGENCY_STOP),
            planning::MISSION_TASK_UNKNOWN);
  EXPECT_EQ(MissionTaskForScene(planning::SCENE_UNKNOWN),
            planning::MISSION_TASK_UNKNOWN);
  EXPECT_EQ(MissionTaskForScene(planning::SCENE_HOLD),
            planning::MISSION_TASK_UNKNOWN);
  EXPECT_EQ(MissionTaskForScene(planning::SCENE_LANE_CRUISE),
            planning::MISSION_TASK_A_TO_B);
  EXPECT_EQ(MissionTaskForScene(planning::SCENE_PULL_OUT),
            planning::MISSION_TASK_PARK_OUT);
  EXPECT_EQ(MissionTaskForScene(planning::SCENE_PARK_IN),
            planning::MISSION_TASK_PARK_IN);
}

TEST(MissionRequestContractTest, PublicPlanMayRequestModeButNotInternalPolicy) {
  auto request = TypedRequest();
  auto* identity = request.mutable_safety_stop()->mutable_identity();
  identity->set_requester_epoch("client-epoch");
  identity->set_request_id("other-request");
  EXPECT_EQ(RejectUnavailableTypedRequest(request)->code(),
            MISSION_REQUEST_INVALID_INPUT);
  request.clear_safety_stop();
  request.mutable_submit_task()->mutable_plan()->set_preferred_mode(
      planning::MODE_CORRIDOR);
  EXPECT_EQ(RejectUnavailableTypedRequest(request)->code(),
            MISSION_REQUEST_UNSUPPORTED_OPERATION);
  request.mutable_submit_task()->mutable_plan()->mutable_domain_policy();
  EXPECT_EQ(RejectUnavailableTypedRequest(request)->code(),
            MISSION_REQUEST_INVALID_INPUT);

  request = TypedRequest();
  request.mutable_submit_task()->mutable_plan()->set_preferred_mode(
      planning::MODE_UNKNOWN);
  EXPECT_EQ(RejectUnavailableTypedRequest(request)->code(),
            MISSION_REQUEST_INVALID_INPUT);
  request.mutable_submit_task()->mutable_plan()->set_preferred_mode(
      planning::MODE_SAFETY_HOLD);
  EXPECT_EQ(RejectUnavailableTypedRequest(request)->code(),
            MISSION_REQUEST_INVALID_INPUT);
}

}  // namespace
}  // namespace mission
}  // namespace apollo
