#include "modules/planning/mode/mode_resolution.h"

#include <algorithm>

namespace apollo {
namespace planning {

namespace {

void AddUniqueBlocker(const std::string& blocker,
                      std::vector<std::string>* blockers) {
  if (blocker.empty() || blockers == nullptr) {
    return;
  }
  if (std::find(blockers->begin(), blockers->end(), blocker) ==
      blockers->end()) {
    blockers->emplace_back(blocker);
  }
}

void AddUniqueMode(std::vector<PlanningMode>* modes, PlanningMode mode) {
  if (modes == nullptr || mode == MODE_UNKNOWN) {
    return;
  }
  if (std::find(modes->begin(), modes->end(), mode) == modes->end()) {
    modes->emplace_back(mode);
  }
}

std::string PlanningModeName(PlanningMode mode) {
  switch (mode) {
    case MODE_LANE_GRAPH:
      return "lane_graph";
    case MODE_CORRIDOR:
      return "corridor";
    case MODE_OPEN_SPACE:
      return "open_space";
    case MODE_FREE_SPACE:
      return "free_space";
    case MODE_SAFETY_HOLD:
      return "safety_hold";
    case MODE_UNKNOWN:
    default:
      return "unknown";
  }
}

bool HasCapabilityForMode(PlanningMode mode, const CapabilitySet* capability) {
  if (capability == nullptr) {
    return false;
  }
  switch (mode) {
    case MODE_LANE_GRAPH:
      return capability->has_lane_graph;
    case MODE_CORRIDOR:
      return capability->has_local_corridor;
    case MODE_OPEN_SPACE:
      return capability->has_parking_roi && capability->has_goal_pose;
    case MODE_FREE_SPACE:
      return capability->has_drivable_area && capability->has_goal_pose;
    case MODE_SAFETY_HOLD:
      return capability->has_stop_target;
    case MODE_UNKNOWN:
    default:
      return false;
  }
}

bool IsModeAvailable(PlanningMode mode,
                     const ModeShellAvailability& availability) {
  switch (mode) {
    case MODE_LANE_GRAPH:
      return availability.lane_graph_available;
    case MODE_CORRIDOR:
      return availability.corridor_available;
    case MODE_OPEN_SPACE:
      return availability.open_space_available;
    case MODE_FREE_SPACE:
      return availability.free_space_available;
    case MODE_SAFETY_HOLD:
      return availability.safety_hold_available;
    case MODE_UNKNOWN:
    default:
      return false;
  }
}

std::string ModeCapabilityUnavailableReason(PlanningMode mode) {
  switch (mode) {
    case MODE_LANE_GRAPH:
      return "lane-graph capability unavailable";
    case MODE_CORRIDOR:
      return "corridor capability unavailable";
    case MODE_OPEN_SPACE:
      return "open-space capability unavailable";
    case MODE_FREE_SPACE:
      return "free-space capability unavailable";
    case MODE_SAFETY_HOLD:
      return "safety-hold capability unavailable";
    case MODE_UNKNOWN:
    default:
      return "planning capability unavailable";
  }
}

std::string ModeShellUnavailableReason(PlanningMode mode) {
  switch (mode) {
    case MODE_LANE_GRAPH:
      return "lane-graph shell unavailable";
    case MODE_CORRIDOR:
      return "corridor shell unavailable";
    case MODE_OPEN_SPACE:
      return "open-space shell unavailable";
    case MODE_FREE_SPACE:
      return "free-space shell unavailable";
    case MODE_SAFETY_HOLD:
      return "safety-hold shell unavailable";
    case MODE_UNKNOWN:
    default:
      return "planning mode unknown";
  }
}

std::vector<PlanningMode> BuildFallbackModes(const PlanningCommand* command,
                                             PlanningMode requested_mode,
                                             PlanningMode legacy_mode) {
  std::vector<PlanningMode> fallback_modes;
  if (command != nullptr && command->has_fallback() &&
      command->fallback().allowed_degraded_modes_size() > 0) {
    for (const auto mode : command->fallback().allowed_degraded_modes()) {
      const auto degraded_mode = static_cast<PlanningMode>(mode);
      if (degraded_mode != requested_mode) {
        AddUniqueMode(&fallback_modes, degraded_mode);
      }
    }
    return fallback_modes;
  }

  if (legacy_mode != requested_mode) {
    AddUniqueMode(&fallback_modes, legacy_mode);
  }
  if (requested_mode != MODE_LANE_GRAPH) {
    AddUniqueMode(&fallback_modes, MODE_LANE_GRAPH);
  }
  if (requested_mode != MODE_CORRIDOR) {
    AddUniqueMode(&fallback_modes, MODE_CORRIDOR);
  }
  if (requested_mode != MODE_SAFETY_HOLD) {
    AddUniqueMode(&fallback_modes, MODE_SAFETY_HOLD);
  }
  return fallback_modes;
}

}  // namespace

PlanningMode ModeResolution::InferRequestedMode(const PlanningCommand* command,
                                                PlanningMode legacy_mode) {
  if (command != nullptr) {
    if (command->has_preferred_mode()) {
      return command->preferred_mode();
    }
    if (command->has_requested_scene()) {
      switch (command->requested_scene()) {
        case SCENE_PARK_IN:
        case SCENE_PULL_OUT:
          return MODE_OPEN_SPACE;
        case SCENE_DOCK:
        case SCENE_SUMMON:
          return MODE_FREE_SPACE;
        case SCENE_EMERGENCY_STOP:
        case SCENE_HOLD:
          return MODE_SAFETY_HOLD;
        case SCENE_UNKNOWN:
        case SCENE_LANE_CRUISE:
        case SCENE_PULL_OVER:
        default:
          break;
      }
    }
  }
  return legacy_mode;
}

ModeResolutionResult ModeResolution::Resolve(
    const PlanningCommand* command, const CapabilitySet* capability,
    const ModeShellAvailability& availability, PlanningMode legacy_mode) {
  ModeResolutionResult result;
  result.requested_mode = InferRequestedMode(command, legacy_mode);

  std::string requested_mode_reason;
  if (IsModeAvailable(result.requested_mode, availability) &&
      HasCapabilityForMode(result.requested_mode, capability)) {
    result.resolved_mode = result.requested_mode;
    return result;
  }

  if (!IsModeAvailable(result.requested_mode, availability)) {
    requested_mode_reason = ModeShellUnavailableReason(result.requested_mode);
  } else {
    requested_mode_reason = ModeCapabilityUnavailableReason(result.requested_mode);
  }
  AddUniqueBlocker(requested_mode_reason, &result.blockers);

  for (const auto fallback_mode :
       BuildFallbackModes(command, result.requested_mode, legacy_mode)) {
    if (!IsModeAvailable(fallback_mode, availability) ||
        !HasCapabilityForMode(fallback_mode, capability)) {
      continue;
    }
    result.resolved_mode = fallback_mode;
    result.reason = "requested " + PlanningModeName(result.requested_mode) +
                    " mode degraded to " + PlanningModeName(fallback_mode) +
                    " because " + requested_mode_reason;
    return result;
  }

  result.reason = requested_mode_reason;
  AddUniqueBlocker("no executable planning mode available", &result.blockers);
  return result;
}

}  // namespace planning
}  // namespace apollo
