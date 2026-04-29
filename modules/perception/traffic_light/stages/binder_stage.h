#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "modules/perception/traffic_light/interface/stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class BinderStage : public BaseStage {
 public:
  explicit BinderStage(const BinderOptions& options)
      : BaseStage(false), options_(options) {}

  std::string Name() const override { return "BinderStage"; }

  bool Process(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }

    std::lock_guard<std::mutex> lock(context->rw_mutex);
    context->metrics.binder = StageRuntimeMetrics();
    std::vector<BoundLight> bound_lights;
    int rejected_candidates = 0;
    for (const auto& visual_light : context->visual_lights) {
      BoundLight bound_light;
      bound_light.visual_light = visual_light;
      const SignalCandidate* best_signal =
          MatchSignal(*context, visual_light, &bound_light.bind_score,
                      &bound_light.topology_confidence,
                      &bound_light.movement_match_score);
      if (best_signal != nullptr) {
        bound_light.signal_movement_mask =
            NormalizeMovementMask(best_signal->movement_mask,
                                  best_signal->intended_movement,
                                  best_signal->shape_hint);
        bound_light.bound_intent =
            PrimaryLaneIntentFromMask(bound_light.signal_movement_mask);
        if (bound_light.bound_intent == LaneIntent::UNKNOWN) {
          bound_light.bound_intent = best_signal->intended_movement;
        }
        bound_light.stopline_distance_m = best_signal->stopline_distance_m;
        bound_light.visual_light.signal_id = best_signal->signal_id;
        bound_light.lane_id = best_signal->lane_id;
        bound_light.signal_group_id = best_signal->signal_group_id;
        const uint32_t ego_movement_mask =
            MovementMaskFromLaneIntent(context->nav_topology.ego_lane_intent);
        bound_light.controlling_ego_lane =
            MovementMasksCompatible(bound_light.signal_movement_mask,
                                    ego_movement_mask);
      } else {
        bound_light.signal_movement_mask = NormalizeMovementMask(
            visual_light.movement_mask, visual_light.intended_movement,
            visual_light.shape);
        bound_light.bound_intent =
            PrimaryLaneIntentFromMask(bound_light.signal_movement_mask);
        if (bound_light.bound_intent == LaneIntent::UNKNOWN) {
          bound_light.bound_intent = visual_light.intended_movement;
        }
        bound_light.stopline_distance_m = visual_light.stopline_distance_m;
        bound_light.bind_score =
            (visual_light.prompt_source == PromptSource::HISTORY
                 ? 0.30f
                 : 0.10f) *
            visual_light.existence_confidence;
        bound_light.topology_confidence = 0.0f;
        const uint32_t ego_movement_mask =
            MovementMaskFromLaneIntent(context->nav_topology.ego_lane_intent);
        bound_light.movement_match_score =
            MovementMasksCompatible(bound_light.signal_movement_mask,
                                    ego_movement_mask)
                ? 1.0f
                : 0.0f;
        bound_light.controlling_ego_lane = MovementMasksCompatible(
            bound_light.signal_movement_mask, ego_movement_mask);
      }

      if (bound_light.bind_score < options_.min_bind_score ||
          (best_signal != nullptr &&
           bound_light.topology_confidence < options_.min_topology_confidence)) {
        ++rejected_candidates;
        continue;
      }
      if (options_.require_intent_match &&
          !MovementMasksCompatible(
              bound_light.signal_movement_mask,
              MovementMaskFromLaneIntent(context->nav_topology.ego_lane_intent))) {
        ++rejected_candidates;
        continue;
      }
      bound_lights.push_back(bound_light);
    }

    std::sort(bound_lights.begin(), bound_lights.end(),
              [](const BoundLight& lhs, const BoundLight& rhs) {
                if (lhs.bind_score != rhs.bind_score) {
                  return lhs.bind_score > rhs.bind_score;
                }
                if (lhs.topology_confidence != rhs.topology_confidence) {
                  return lhs.topology_confidence > rhs.topology_confidence;
                }
                if (lhs.stopline_distance_m < 0.0) {
                  return false;
                }
                if (rhs.stopline_distance_m < 0.0) {
                  return true;
                }
                return lhs.stopline_distance_m < rhs.stopline_distance_m;
              });
    context->bound_lights = std::move(bound_lights);
    context->metrics.binder.input_count =
        static_cast<int>(context->visual_lights.size());
    context->metrics.binder.output_count =
        static_cast<int>(context->bound_lights.size());
    context->metrics.binder.rejected_count = rejected_candidates;
    for (const auto& light : context->bound_lights) {
      context->metrics.binder.max_confidence =
          std::max(context->metrics.binder.max_confidence, light.bind_score);
      context->metrics.binder.avg_confidence += light.bind_score;
    }
    if (!context->bound_lights.empty()) {
      context->metrics.binder.avg_confidence /=
          static_cast<float>(context->bound_lights.size());
    }
    return true;
  }

 private:
  const SignalCandidate* MatchSignal(const PipelineContext& context,
                                     const VisualLight& visual_light,
                                     float* bind_score,
                                     float* topology_confidence,
                                     float* movement_match_score) const {
    const SignalCandidate* best_signal = nullptr;
    float best_score = -std::numeric_limits<float>::max();
    float best_topology = 0.0f;
    float best_movement_match = 0.0f;
    const uint32_t visual_movement_mask =
        NormalizeMovementMask(visual_light.movement_mask,
                              visual_light.intended_movement,
                              visual_light.shape);
    const uint32_t ego_movement_mask =
        MovementMaskFromLaneIntent(context.nav_topology.ego_lane_intent);
    for (const auto& signal : context.map_signals) {
      const uint32_t signal_movement_mask =
          NormalizeMovementMask(signal.movement_mask, signal.intended_movement,
                                signal.shape_hint);
      if (options_.require_intent_match &&
          !MovementMasksCompatible(visual_movement_mask, signal_movement_mask)) {
        continue;
      }
      if (options_.require_intent_match &&
          !MovementMasksCompatible(signal_movement_mask, ego_movement_mask)) {
        continue;
      }
      const float overlap = ComputeIou(visual_light.bbox, signal.projection_roi);
      const float center_distance_ratio =
          ComputeCenterDistanceRatio(visual_light.bbox, signal.projection_roi);
      if (overlap < options_.min_signal_overlap &&
          center_distance_ratio > options_.max_center_distance_ratio) {
        continue;
      }
      const float geometry_consistency = std::max(
          overlap, 1.0f - std::min(1.0f, center_distance_ratio /
                                            std::max(0.1f,
                                                     options_.max_center_distance_ratio)));
      const float id_bonus =
          (!visual_light.signal_id.empty() && visual_light.signal_id == signal.signal_id)
              ? 0.20f
              : 0.0f;
      const float movement_match =
          MovementMasksCompatible(visual_movement_mask, signal_movement_mask)
              ? 1.0f
              : 0.0f;
      const float ego_match =
          MovementMasksCompatible(signal_movement_mask, ego_movement_mask)
              ? 1.0f
              : 0.0f;
      const float topology =
          0.60f * geometry_consistency + 0.40f * signal.topology_confidence;
      const float score = 0.30f * visual_light.existence_confidence +
                          0.20f * visual_light.state_confidence +
                          0.30f * topology + 0.20f * signal.confidence +
                          0.10f * movement_match + 0.08f * ego_match + id_bonus;
      if (score > best_score) {
        best_score = score;
        best_topology = topology;
        best_movement_match = 0.5f * movement_match + 0.5f * ego_match;
        best_signal = &signal;
      }
    }
    if (bind_score != nullptr) {
      *bind_score = std::max(0.0f, best_score);
    }
    if (topology_confidence != nullptr) {
      *topology_confidence = best_topology;
    }
    if (movement_match_score != nullptr) {
      *movement_match_score = best_movement_match;
    }
    return best_signal;
  }

  static float ComputeIou(const Rect2f& lhs, const Rect2f& rhs) {
    const float x1 = std::max(lhs.x, rhs.x);
    const float y1 = std::max(lhs.y, rhs.y);
    const float x2 = std::min(lhs.x + lhs.width, rhs.x + rhs.width);
    const float y2 = std::min(lhs.y + lhs.height, rhs.y + rhs.height);
    const float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    const float uni = lhs.Area() + rhs.Area() - inter;
    return uni > 1e-6f ? inter / uni : 0.0f;
  }

  static float ComputeCenterDistanceRatio(const Rect2f& lhs, const Rect2f& rhs) {
    if (!lhs.IsValid() || !rhs.IsValid()) {
      return std::numeric_limits<float>::max();
    }
    const float lhs_cx = lhs.x + lhs.width * 0.5f;
    const float lhs_cy = lhs.y + lhs.height * 0.5f;
    const float rhs_cx = rhs.x + rhs.width * 0.5f;
    const float rhs_cy = rhs.y + rhs.height * 0.5f;
    const float dx = lhs_cx - rhs_cx;
    const float dy = lhs_cy - rhs_cy;
    const float distance = std::sqrt(dx * dx + dy * dy);
    const float scale = std::max(1.0f, std::max(rhs.width, rhs.height));
    return distance / scale;
  }

  BinderOptions options_;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
