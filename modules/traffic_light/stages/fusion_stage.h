// Copyright 2026 The Apollo Authors. All Rights Reserved.
#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "modules/traffic_light/interface/stage.h"

namespace apollo {
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
      if (!track.current_state.binding_valid ||
          track.current_state.binding_ambiguous) {
        continue;
      }
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
      result.freshness_sec = frame_ts_sec - track.last_state_observed_sec;
      result.is_degraded =
          track.lost_frames > 0 || result.color == LightColor::UNKNOWN;
      if (result.is_degraded) {
        result.degrade_reason = "stale_or_occluded_track";
      }
      result.source = EvidenceSource::VISION;
      result.decision_reason =
          "vision track " + std::to_string(track.track_id) + " stabilized";
      if (result.is_degraded) {
        result.confidence *= 0.75f;
      }
      results.push_back(result);
    }

    const auto conflicts = ConsolidateSignals(context, &results);
    for (auto& result : results) {
      if (conflicts.count(result.signal_id) != 0) {
        continue;
      }
      const V2XLightEvidence* v2x = SelectBestV2XEvidence(*context, result);
      if (v2x != nullptr) {
        const bool no_visual = result.color == LightColor::UNKNOWN;
        const bool weak_visual =
            result.confidence < options_.weak_vision_threshold;
        const bool strong_v2x =
            v2x->confidence >= options_.strong_v2x_threshold;
        const bool conflict = !no_visual && v2x->color != LightColor::UNKNOWN &&
                              v2x->color != result.color;
        const bool matching_signal =
            !result.signal_id.empty() && result.signal_id == v2x->signal_id;
        const bool matching_intent =
            !options_.prefer_ego_intent ||
            MovementMasksCompatible(
                NormalizeMovementMask(result.signal_movement_mask,
                                      result.bound_intent, result.shape),
                NormalizeMovementMask(v2x->movement_mask, v2x->movement,
                                      LightShape::UNKNOWN));
        if (strong_v2x && matching_signal && matching_intent &&
            (no_visual || weak_visual ||
             (conflict &&
              result.confidence + options_.min_conflict_override_margin <
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
        result.signal_movement_mask = NormalizeMovementMask(
            best_v2x->movement_mask, best_v2x->movement, LightShape::UNKNOWN);
        result.source = EvidenceSource::V2X;
        result.decision_reason = "standalone strong v2x";
        results.push_back(result);
      }
    }

    if (results.empty()) {
      for (const auto& signal : context->map_signals) {
        TrafficLightResult result;
        result.color = LightColor::UNKNOWN;
        result.shape = signal.shape_hint;
        result.confidence = 0.0f;
        result.existence_confidence = 0.0f;
        result.state_confidence = 0.0f;
        result.topology_confidence = signal.topology_confidence;
        result.signal_id = signal.signal_id;
        result.lane_id = signal.lane_id;
        result.signal_group_id = signal.signal_group_id;
        result.camera_name = signal.camera_name;
        result.bound_intent = signal.intended_movement;
        result.signal_movement_mask = NormalizeMovementMask(
            signal.movement_mask, signal.intended_movement, signal.shape_hint);
        result.controlling_ego_lane = false;
        result.stopline_distance_m = signal.stopline_distance_m;
        result.is_degraded = true;
        result.source = EvidenceSource::UNKNOWN;
        result.degrade_reason = context->status.degrade_reason;
        result.decision_reason =
            "map signal present but no visual/v2x state estimate";
        results.push_back(result);
      }
      ConsolidateSignals(context, &results);
    }

    if (results.empty()) {
      context->final_lights.clear();
      context->primary_decision = TrafficLightResult();
      context->primary_decision.is_degraded = true;
      context->primary_decision.degrade_reason = context->status.degrade_reason;
      context->primary_decision.decision_reason =
          "no unambiguous bound visual state or strong v2x";
      return true;
    }

    const int primary_index = SelectPrimaryIndex(*context, results);
    if (primary_index > 0) {
      std::swap(results.front(), results[primary_index]);
    }
    if (results.size() > 1) {
      std::sort(
          results.begin() + 1, results.end(),
          [](const TrafficLightResult& lhs, const TrafficLightResult& rhs) {
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
    context->metrics.fusion.max_confidence =
        context->primary_decision.confidence;
    context->metrics.fusion.avg_confidence =
        context->primary_decision.confidence;
    context->metrics.fusion.note = context->primary_decision.decision_reason;
    return true;
  }

 private:
  static std::set<std::string> ConsolidateSignals(
      PipelineContext* context, std::vector<TrafficLightResult>* results) {
    std::map<std::string, size_t> by_id;
    std::set<std::string> conflicts;
    std::vector<TrafficLightResult> unique;
    for (const auto& result : *results) {
      if (result.signal_id.empty()) {
        context->AppendDegradeReason("fusion missing external signal identity");
        continue;
      }
      const auto inserted = by_id.emplace(result.signal_id, unique.size());
      if (inserted.second) {
        unique.push_back(result);
        continue;
      }
      auto& previous = unique[inserted.first->second];
      if (previous.color != result.color || previous.blink != result.blink) {
        conflicts.insert(result.signal_id);
      }
      if (result.confidence > previous.confidence) {
        previous = result;
      }
    }
    for (const auto& id : conflicts) {
      auto& result = unique[by_id.at(id)];
      result.color = LightColor::UNKNOWN;
      result.blink = false;
      result.confidence = 0.0f;
      result.state_confidence = 0.0f;
      result.is_degraded = true;
      result.source = EvidenceSource::UNKNOWN;
      result.degrade_reason = "conflicting physical heads for signal " + id;
      result.decision_reason = result.degrade_reason;
      context->AppendDegradeReason(result.degrade_reason);
    }
    *results = std::move(unique);
    return conflicts;
  }

  const V2XLightEvidence* SelectBestV2XEvidence(
      PipelineContext& context, const TrafficLightResult& result) const {
    const double frame_ts_sec = static_cast<double>(context.timestamp) * 1e-9;
    if (!std::isfinite(options_.v2x_sync_window_sec) ||
        options_.v2x_sync_window_sec <= 0.0) {
      return nullptr;
    }
    std::map<std::string, const V2XLightEvidence*> latest;
    std::set<std::string> conflicts;
    auto consider = [&](const V2XLightEvidence& evidence) {
      if (evidence.signal_id.empty() ||
          !std::isfinite(evidence.timestamp_sec) ||
          !std::isfinite(evidence.confidence) || evidence.confidence < 0.0f ||
          evidence.confidence > 1.0f) {
        return;
      }
      const double dt = std::fabs(frame_ts_sec - evidence.timestamp_sec);
      if (dt > options_.v2x_sync_window_sec) {
        return;
      }
      auto& previous = latest[evidence.signal_id];
      if (previous == nullptr ||
          evidence.timestamp_sec > previous->timestamp_sec) {
        previous = &evidence;
        conflicts.erase(evidence.signal_id);
      } else if (evidence.timestamp_sec == previous->timestamp_sec) {
        if (evidence.color != previous->color ||
            evidence.blink != previous->blink) {
          conflicts.insert(evidence.signal_id);
        }
        if (evidence.confidence > previous->confidence) {
          previous = &evidence;
        }
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
    const V2XLightEvidence* best = nullptr;
    float best_score = -1.0f;
    for (const auto& item : latest) {
      const auto& evidence = *item.second;
      if (conflicts.count(item.first) != 0) {
        context.AppendDegradeReason("conflicting v2x state for signal " +
                                    item.first);
        continue;
      }
      if (evidence.color == LightColor::UNKNOWN) {
        continue;
      }
      if (!result.signal_id.empty() && !evidence.signal_id.empty() &&
          result.signal_id != evidence.signal_id) {
        continue;
      }
      if (options_.prefer_ego_intent &&
          !MovementMasksCompatible(
              NormalizeMovementMask(result.signal_movement_mask,
                                    result.bound_intent, result.shape),
              NormalizeMovementMask(evidence.movement_mask, evidence.movement,
                                    LightShape::UNKNOWN))) {
        continue;
      }
      const double dt = std::fabs(frame_ts_sec - evidence.timestamp_sec);
      const float recency = static_cast<float>(
          std::max(0.0, 1.0 - dt / options_.v2x_sync_window_sec));
      const float score = 0.7f * evidence.confidence + 0.3f * recency;
      if (score > best_score) {
        best_score = score;
        best = &evidence;
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
    if (options_.prefer_ego_intent && result.controlling_ego_lane &&
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
      PipelineContext& context) const {
    TrafficLightResult placeholder;
    placeholder.bound_intent = context.nav_topology.ego_lane_intent;
    return SelectBestV2XEvidence(context, placeholder);
  }

  FusionOptions options_;
};

}  // namespace traffic_light
}  // namespace apollo
