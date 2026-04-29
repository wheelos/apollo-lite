#pragma once

#include <algorithm>
#include <string>

#include "modules/perception/traffic_light/interface/stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class HeuristicStage : public BaseStage {
 public:
  explicit HeuristicStage(const HeuristicOptions& options)
      : BaseStage(true), options_(options) {}

  std::string Name() const override { return "HeuristicStage"; }

  bool Process(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }

    std::lock_guard<std::mutex> lock(context->rw_mutex);
    context->metrics.heuristic = StageRuntimeMetrics();
    HeuristicState state;
    state.inferred_color = LightColor::UNKNOWN;
    state.probability = 0.0f;
    state.advisory_only = true;

    if (context->nav_topology.is_in_intersection ||
        context->nav_topology.distance_to_intersection_m >
            options_.max_distance_to_intersection_m) {
      context->heuristic_state = state;
      context->metrics.heuristic.note = "outside heuristic window";
      return true;
    }

    const bool has_topology_support =
        !context->map_signals.empty() ||
        (context->runtime_state != nullptr &&
         !context->runtime_state->tracked_memory.empty());
    if (!has_topology_support) {
      context->heuristic_state = state;
      context->metrics.heuristic.note = "no topology support";
      return true;
    }

    int starting_agents = 0;
    for (const auto& agent : context->surrounding_agents) {
      const bool intent_match =
          context->nav_topology.ego_lane_intent == LaneIntent::UNKNOWN ||
          agent.intent == context->nav_topology.ego_lane_intent;
      if (!intent_match) {
        continue;
      }
      if (agent.is_starting || agent.velocity > 1.0) {
        ++starting_agents;
      }
    }

    if (starting_agents >= options_.min_starting_agents) {
      state.inferred_color = LightColor::GREEN;
      state.probability = std::min(
          0.99f, options_.green_probability +
                     0.03f * static_cast<float>(starting_agents - 1));
      state.supporting_agents = starting_agents;
      state.advisory_only = true;
      state.inference_reason = "same-intent vehicles started moving";
    } else if (context->ego_state.velocity < 0.3 &&
               context->nav_topology.distance_to_intersection_m < 30.0) {
      state.inferred_color = LightColor::RED;
      state.probability = options_.red_hold_probability;
      state.advisory_only = false;
      state.inference_reason = "approaching stopline without leading movement";
    }

    context->heuristic_state = state;
    context->metrics.heuristic.output_count =
        state.inferred_color == LightColor::UNKNOWN ? 0 : 1;
    context->metrics.heuristic.max_confidence = state.probability;
    context->metrics.heuristic.avg_confidence = state.probability;
    context->metrics.heuristic.note = state.inference_reason;
    return true;
  }

 private:
  HeuristicOptions options_;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
