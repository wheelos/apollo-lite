#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "modules/perception/traffic_light/interface/stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class FusionStage : public BaseStage {
 public:
  explicit FusionStage(const FusionOptions& options)
      : BaseStage(false), options_(options) {}

  std::string Name() const override { return "FusionStage"; }

  bool Process(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }

    std::lock_guard<std::mutex> lock(context->rw_mutex);
    context->metrics.fusion = StageRuntimeMetrics();
    std::vector<TrafficLightResult> results;
    const double frame_ts_sec = static_cast<double>(context->timestamp) * 1e-9;
    for (const auto& track : context->tracked_lights) {
      TrafficLightResult result;
      result.color = track.stabilized_color;
      result.shape = track.current_state.visual_light.shape;
      result.existence_confidence =
          track.current_state.visual_light.existence_confidence;
      result.state_confidence = track.stabilized_confidence;
      result.topology_confidence = track.current_state.topology_confidence;
      result.confidence =
          std::min(1.0f, 0.30f * result.existence_confidence +
                             0.45f * result.state_confidence +
                             0.25f * result.topology_confidence);
      result.blink = track.blink;
      result.signal_id = track.current_state.visual_light.signal_id;
      result.lane_id = track.current_state.lane_id;
      result.signal_group_id = track.current_state.signal_group_id;
      result.camera_name = track.current_state.visual_light.camera_name;
      result.bound_intent = track.current_state.bound_intent;
      result.signal_movement_mask = track.current_state.signal_movement_mask;
      result.controlling_ego_lane = track.current_state.controlling_ego_lane;
      result.stopline_distance_m = track.current_state.stopline_distance_m;
      result.freshness_sec = frame_ts_sec - track.last_visible_timestamp_sec;
      result.is_degraded = track.lost_frames > 0 || result.freshness_sec > 0.6;
      if (result.is_degraded) {
        result.degrade_reason = "stale_or_occluded_track";
      }
      result.source = EvidenceSource::VISION;
      result.decision_reason =
          "vision track " + std::to_string(track.track_id) + " stabilized";
      if (result.is_degraded) {
        result.confidence *= 0.75f;
      }

      const V2XLightEvidence* v2x = SelectBestV2XEvidence(*context, result);
      if (v2x != nullptr) {
        const bool no_visual = result.color == LightColor::UNKNOWN;
        const bool weak_visual = result.confidence < options_.weak_vision_threshold;
        const bool strong_v2x = v2x->confidence >= options_.strong_v2x_threshold;
        const bool conflict = !no_visual && v2x->color != LightColor::UNKNOWN &&
                              v2x->color != result.color;
        const bool matching_signal =
            result.signal_id.empty() || v2x->signal_id.empty() ||
            result.signal_id == v2x->signal_id;
        const bool matching_intent =
            !options_.prefer_ego_intent ||
            MovementMasksCompatible(
                NormalizeMovementMask(result.signal_movement_mask,
                                      result.bound_intent, result.shape),
                NormalizeMovementMask(v2x->movement_mask, v2x->movement,
                                      LightShape::UNKNOWN));
        if (strong_v2x && matching_signal && matching_intent &&
            (no_visual || weak_visual ||
             (conflict && result.confidence + options_.min_conflict_override_margin <
                              v2x->confidence &&
              result.topology_confidence < 0.5f))) {
          result.color = v2x->color;
          result.blink = v2x->blink;
          result.confidence = std::max(result.confidence, v2x->confidence);
          result.state_confidence =
              std::max(result.state_confidence, v2x->confidence);
          if (result.signal_id.empty()) {
            result.signal_id = v2x->signal_id;
          }
          result.source = EvidenceSource::V2X;
          result.decision_reason = "v2x override";
        } else if (!conflict && strong_v2x && matching_intent &&
                   result.source == EvidenceSource::VISION) {
          result.confidence =
              std::min(1.0f, result.confidence + 0.1f * v2x->confidence);
          result.decision_reason += ", corroborated by v2x";
        } else if (conflict) {
          result.decision_reason += ", kept vision over conflicting v2x";
        }
      }
      results.push_back(result);
    }

    if (results.empty()) {
      const V2XLightEvidence* best_v2x = SelectStandaloneV2XEvidence(*context);
      if (best_v2x != nullptr &&
          best_v2x->confidence >= options_.strong_v2x_threshold) {
        TrafficLightResult result;
        result.color = best_v2x->color;
        result.confidence = best_v2x->confidence;
        result.existence_confidence = best_v2x->confidence;
        result.state_confidence = best_v2x->confidence;
        result.signal_id = best_v2x->signal_id;
        result.bound_intent = best_v2x->movement;
        result.signal_movement_mask =
            NormalizeMovementMask(best_v2x->movement_mask, best_v2x->movement,
                                  LightShape::UNKNOWN);
        result.source = EvidenceSource::V2X;
        result.decision_reason = "standalone strong v2x";
        results.push_back(result);
      }
    }

    int primary_index = SelectPrimaryIndex(*context, results);
    if ((results.empty() || results[primary_index].confidence <
                                options_.heuristic_trigger_threshold) &&
        context->heuristic_state.probability >=
            options_.heuristic_accept_threshold) {
      const bool can_use_green_heuristic =
          context->heuristic_state.inferred_color != LightColor::GREEN ||
          (results.empty() && !context->map_signals.empty());
      TrafficLightResult heuristic_result;
      heuristic_result.color = context->heuristic_state.inferred_color;
      heuristic_result.shape = LightShape::CIRCLE;
      heuristic_result.confidence = context->heuristic_state.probability;
      heuristic_result.existence_confidence =
          context->heuristic_state.probability * 0.8f;
      heuristic_result.state_confidence = context->heuristic_state.probability;
      heuristic_result.is_heuristic_override = true;
      heuristic_result.source = EvidenceSource::HEURISTIC;
      heuristic_result.bound_intent = context->nav_topology.ego_lane_intent;
      heuristic_result.signal_movement_mask =
          MovementMaskFromLaneIntent(context->nav_topology.ego_lane_intent);
      heuristic_result.controlling_ego_lane = true;
      heuristic_result.is_degraded = context->heuristic_state.advisory_only;
      heuristic_result.degrade_reason =
          context->heuristic_state.advisory_only ? "heuristic_only" : "";
      heuristic_result.decision_reason = context->heuristic_state.inference_reason;
      if (!context->map_signals.empty()) {
        heuristic_result.signal_id = context->map_signals.front().signal_id;
        heuristic_result.lane_id = context->map_signals.front().lane_id;
        heuristic_result.signal_group_id =
            context->map_signals.front().signal_group_id;
        heuristic_result.stopline_distance_m =
            context->map_signals.front().stopline_distance_m;
      } else {
        heuristic_result.signal_id = "ego_primary";
      }
      if (heuristic_result.color != LightColor::UNKNOWN && can_use_green_heuristic &&
          results.empty()) {
        results.push_back(heuristic_result);
        primary_index = 0;
      } else if (heuristic_result.color == LightColor::RED &&
                 !context->heuristic_state.advisory_only) {
        results[primary_index] = heuristic_result;
      }
    }

    if (results.empty()) {
      context->final_lights.clear();
      context->primary_decision = TrafficLightResult();
      return true;
    }

    if (primary_index > 0) {
      std::swap(results.front(), results[primary_index]);
    }
    if (results.size() > 1) {
      std::sort(results.begin() + 1, results.end(),
                [](const TrafficLightResult& lhs,
                   const TrafficLightResult& rhs) {
                  return lhs.confidence > rhs.confidence;
                });
    }
    context->primary_decision = results.front();
    context->final_lights = results;
    context->primary_decision.degrade_reason = context->status.degrade_reason;
    context->primary_decision.is_degraded =
        context->primary_decision.is_degraded ||
        !context->status.degrade_reason.empty();
    context->metrics.fusion.input_count =
        static_cast<int>(context->tracked_lights.size());
    context->metrics.fusion.output_count =
        static_cast<int>(context->final_lights.size());
    context->metrics.fusion.max_confidence = context->primary_decision.confidence;
    context->metrics.fusion.avg_confidence = context->primary_decision.confidence;
    context->metrics.fusion.note = context->primary_decision.decision_reason;
    return true;
  }

 private:
  const V2XLightEvidence* SelectBestV2XEvidence(
      const PipelineContext& context, const TrafficLightResult& result) const {
    const double frame_ts_sec = static_cast<double>(context.timestamp) * 1e-9;
    const V2XLightEvidence* best = nullptr;
    float best_score = -1.0f;
    auto consider = [&](const V2XLightEvidence& evidence) {
      const double dt = std::fabs(frame_ts_sec - evidence.timestamp_sec);
      if (dt > options_.v2x_sync_window_sec) {
        return;
      }
      if (!result.signal_id.empty() && !evidence.signal_id.empty() &&
          result.signal_id != evidence.signal_id) {
        return;
      }
      if (options_.prefer_ego_intent &&
          !MovementMasksCompatible(
              NormalizeMovementMask(result.signal_movement_mask,
                                    result.bound_intent, result.shape),
              NormalizeMovementMask(evidence.movement_mask, evidence.movement,
                                    LightShape::UNKNOWN))) {
        return;
      }
      const float recency =
          static_cast<float>(std::max(0.0, 1.0 - dt / options_.v2x_sync_window_sec));
      const float score = 0.7f * evidence.confidence + 0.3f * recency;
      if (score > best_score) {
        best_score = score;
        best = &evidence;
      }
    };
    for (const auto& evidence : context.v2x_lights) {
      consider(evidence);
    }
    if (context.runtime_state != nullptr) {
      for (const auto& evidence : context.runtime_state->v2x_buffer) {
        consider(evidence);
      }
    }
    return best;
  }

  int SelectPrimaryIndex(const PipelineContext& context,
                         const std::vector<TrafficLightResult>& results) const {
    if (results.empty()) {
      return 0;
    }
    int best_index = 0;
    float best_score = ScorePrimary(context, results.front());
    for (size_t i = 1; i < results.size(); ++i) {
      const float score = ScorePrimary(context, results[i]);
      if (score > best_score) {
        best_score = score;
        best_index = static_cast<int>(i);
      }
    }
    return best_index;
  }

  float ScorePrimary(const PipelineContext& context,
                     const TrafficLightResult& result) const {
    float score = result.confidence;
    if (options_.prefer_ego_intent &&
        result.controlling_ego_lane &&
        MovementMasksCompatible(
            NormalizeMovementMask(result.signal_movement_mask,
                                  result.bound_intent, result.shape),
            MovementMaskFromLaneIntent(context.nav_topology.ego_lane_intent))) {
      score += 0.2f;
    }
    score += 0.15f * result.topology_confidence;
    if (result.stopline_distance_m >= 0.0) {
      const float distance_bonus = std::max(
          0.0f, 1.0f - static_cast<float>(result.stopline_distance_m / 80.0));
      score += 0.2f * distance_bonus;
    }
    if (result.source == EvidenceSource::VISION) {
      score += 0.05f;
    }
    return score;
  }

  const V2XLightEvidence* SelectStandaloneV2XEvidence(
      const PipelineContext& context) const {
    TrafficLightResult placeholder;
    placeholder.bound_intent = context.nav_topology.ego_lane_intent;
    return SelectBestV2XEvidence(context, placeholder);
  }

  FusionOptions options_;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
