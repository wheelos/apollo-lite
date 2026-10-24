#pragma once

#include <cstdint>
#include <optional>

#include "wheelos_msgs/mission_msgs/mission_request.pb.h"
#include "wheelos_msgs/mission_msgs/mission_request_result.pb.h"
#include "wheelos_msgs/mission_msgs/mission_runtime_status.pb.h"
#include "wheelos_msgs/planning_msgs/mission_directive.pb.h"

namespace apollo {
namespace mission {

constexpr uint32_t kMissionRequestContractVersion = 1;

MissionContractCapabilities CurrentMissionContractCapabilities();

// No result means a legacy BT request, not admission of a typed operation.
std::optional<MissionRequestResult> RejectUnavailableTypedRequest(
    const MissionRequest& request);

planning::MissionTaskType MissionTaskForScene(
    planning::PlanningSceneType scene);

}  // namespace mission
}  // namespace apollo
