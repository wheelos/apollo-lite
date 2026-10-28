// Copyright 2026 The Apollo Authors. All Rights Reserved.
#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "modules/traffic_light/interface/stage.h"

namespace apollo {
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
    context->bound_lights.clear();
    const double now = static_cast<double>(context->timestamp) * 1e-9;
    if (now < last_frame_sec_ || epoch_ != context->lane_context.epoch) {
      history_.clear();
    }
    last_frame_sec_ = now;
    epoch_ = context->lane_context.epoch;
    for (auto it = history_.begin(); it != history_.end();) {
      if (now - it->second.observed_sec > options_.binding_history_sec) {
        it = history_.erase(it);
      } else {
        ++it;
      }
    }
    std::vector<VisualLight> observations;
    if (!context->tracked_lights.empty()) {
      for (const auto& track : context->tracked_lights) {
        observations.push_back(track.current_state.visual_light);
      }
    } else {
      observations = context->visual_lights;
    }
    bool lane_ambiguous = false;
    const LaneCandidate* lane = SelectLane(*context, &lane_ambiguous);
    std::vector<bool> current_observations(observations.size(), true);
    for (size_t i = 0; i < observations.size(); ++i) {
      if (!context->tracked_lights.empty()) {
        const auto& track = context->tracked_lights[i];
        double observed_at = static_cast<double>(context->timestamp) * 1e-9;
        for (const auto& frame : context->camera_frames) {
          if (frame.camera_name == observations[i].camera_name &&
              frame.timestamp_ns != 0) {
            observed_at = static_cast<double>(frame.timestamp_ns) * 1e-9;
            break;
          }
        }
        current_observations[i] =
            track.lost_frames == 0 &&
            std::isfinite(track.last_visible_timestamp_sec) &&
            track.last_visible_timestamp_sec > 0.0 &&
            std::abs(track.last_visible_timestamp_sec - observed_at) <= 1e-6;
      }
    }
    for (size_t i = 0; i < observations.size(); ++i) {
      BoundLight bound;
      if (current_observations[i]) {
        bound = Bind(*context, observations[i], lane, lane_ambiguous,
                     ObservedGroupSize(observations, current_observations, i));
      } else {
        bound.visual_light = observations[i];
        bound.visual_light.signal_id.clear();
        bound.binding_reason = "visual target not observed in current frame";
      }
      if (!context->tracked_lights.empty()) {
        context->tracked_lights[i].current_state = bound;
      }
      if (!bound.binding_valid) {
        context->AppendDegradeReason("binder: " + bound.binding_reason);
        ++context->metrics.binder.rejected_count;
        continue;
      }
      context->metrics.binder.max_confidence =
          std::max(context->metrics.binder.max_confidence, bound.bind_score);
      context->metrics.binder.avg_confidence += bound.bind_score;
      context->bound_lights.push_back(std::move(bound));
    }
    std::sort(context->bound_lights.begin(), context->bound_lights.end(),
              [](const BoundLight& lhs, const BoundLight& rhs) {
                return lhs.bind_score > rhs.bind_score;
              });
    context->metrics.binder.input_count = static_cast<int>(observations.size());
    context->metrics.binder.output_count =
        static_cast<int>(context->bound_lights.size());
    if (!context->bound_lights.empty()) {
      context->metrics.binder.avg_confidence /=
          static_cast<float>(context->bound_lights.size());
    }
    return true;
  }

 private:
  struct BindingPrior {
    std::string signal_id;
    std::string lane_id;
    std::string approach_id;
    double observed_sec = 0.0;
  };

  // These are ranking margins, not calibrated posterior probabilities.
  static constexpr float kSignalAmbiguityMargin = 0.08f;
  static constexpr float kLaneAmbiguityMargin = 0.10f;
  static constexpr float kLaneWeight = 0.40f;
  static constexpr float kTopologyWeight = 0.15f;
  static constexpr float kSignalConfidenceWeight = 0.05f;
  static constexpr float kArrowWeight = 0.25f;
  static constexpr float kGroupWeight = 0.20f;
  static constexpr float kProjectionWeight = 0.02f;

  static float UnitScore(float score) {
    return std::isfinite(score) ? std::max(0.0f, std::min(1.0f, score)) : 0.0f;
  }

  static uint32_t ArrowMask(LightShape shape) {
    return shape == LightShape::CIRCLE ? kMovementMaskNone
                                       : MovementMaskFromShape(shape);
  }

  static uint32_t SignalMask(const SignalCandidate& signal) {
    if (signal.movement_mask != kMovementMaskNone) {
      return signal.movement_mask;
    }
    const uint32_t intent =
        MovementMaskFromLaneIntent(signal.intended_movement);
    return intent != kMovementMaskNone ? intent : ArrowMask(signal.shape_hint);
  }

  static uint32_t VisualMask(const VisualLight& visual) {
    // Circular heads and navigation/prompt intent do not identify a movement.
    if (visual.shape == LightShape::CIRCLE) {
      return kMovementMaskNone;
    }
    const uint32_t arrow = ArrowMask(visual.shape);
    return arrow != kMovementMaskNone ? arrow : visual.movement_mask;
  }

  static bool Conflicts(uint32_t lhs, uint32_t rhs) {
    return lhs != kMovementMaskNone && rhs != kMovementMaskNone &&
           (lhs & rhs) == 0u;
  }

  static float MovementSimilarity(uint32_t lhs, uint32_t rhs) {
    uint32_t intersection = lhs & rhs;
    uint32_t combined = lhs | rhs;
    int common_count = 0;
    int total_count = 0;
    while (combined != 0u) {
      common_count += static_cast<int>(intersection & 1u);
      total_count += static_cast<int>(combined & 1u);
      intersection >>= 1;
      combined >>= 1;
    }
    return total_count > 0 ? static_cast<float>(common_count) / total_count
                           : 0.0f;
  }

  const LaneCandidate* SelectLane(const PipelineContext& context,
                                  bool* ambiguous) const {
    const auto& lanes = context.lane_context;
    const double now = static_cast<double>(context.timestamp) * 1e-9;
    if (!std::isfinite(lanes.measurement_time_sec) ||
        !std::isfinite(lanes.valid_until_sec) ||
        lanes.valid_until_sec <= lanes.measurement_time_sec ||
        now < lanes.measurement_time_sec || now > lanes.valid_until_sec) {
      return nullptr;
    }
    const LaneCandidate* best = nullptr;
    for (const auto& lane : lanes.candidates) {
      if (!lane.lane_id.empty() && lane.movement_mask != kMovementMaskNone &&
          UnitScore(lane.confidence) > 0.0f &&
          UnitScore(lane.confidence) >= options_.min_topology_confidence &&
          (best == nullptr ||
           UnitScore(lane.confidence) > UnitScore(best->confidence))) {
        best = &lane;
      }
    }
    if (best == nullptr) {
      return nullptr;
    }
    for (const auto& lane : lanes.candidates) {
      if (&lane == best || lane.lane_id.empty() ||
          lane.movement_mask == kMovementMaskNone ||
          UnitScore(lane.confidence) <= 0.0f ||
          UnitScore(lane.confidence) < options_.min_topology_confidence) {
        continue;
      }
      if ((lane.lane_id != best->lane_id ||
           lane.approach_id != best->approach_id ||
           lane.movement_mask != best->movement_mask) &&
          UnitScore(best->confidence) - UnitScore(lane.confidence) <=
              kLaneAmbiguityMargin) {
        *ambiguous = true;
        return nullptr;
      }
    }
    return best;
  }

  BoundLight Bind(const PipelineContext& context, const VisualLight& visual,
                  const LaneCandidate* lane, bool lane_ambiguous,
                  int observed_group_size) {
    BoundLight bound;
    bound.visual_light = visual;
    bound.visual_light.signal_id.clear();
    bound.visual_light.observed_group_size = observed_group_size;
    bound.binding_reason = "no independently supported signal relation";
    if (lane_ambiguous) {
      bound.binding_ambiguous = true;
      bound.binding_reason = "ambiguous lane context";
      return bound;
    }
    const uint32_t visual_mask = VisualMask(visual);
    if (lane != nullptr && Conflicts(visual_mask, lane->movement_mask)) {
      bound.binding_reason =
          "observed arrow conflicts with legal lane movement";
      return bound;
    }
    const SignalCandidate* best = nullptr;
    const SignalCandidate* previous = nullptr;
    float best_score = -1.0f;
    float previous_score = -1.0f;
    float runner_up = -1.0f;
    const auto key = std::make_pair(visual.camera_name, visual.track_id);
    auto prior = history_.find(key);
    if (lane == nullptr || visual.track_id <= 0 ||
        (prior != history_.end() &&
         (prior->second.lane_id != lane->lane_id ||
          prior->second.approach_id != lane->approach_id))) {
      history_.erase(key);
      prior = history_.end();
    }
    for (const auto& signal : context.map_signals) {
      if ((!signal.camera_name.empty() &&
           signal.camera_name != visual.camera_name) ||
          UnitScore(signal.topology_confidence) <
              options_.min_topology_confidence) {
        continue;
      }
      if (signal.signal_id.empty()) {
        bound.binding_reason = "signal candidate missing external identity";
        continue;
      }
      const uint32_t signal_mask = SignalMask(signal);
      const uint32_t hint_mask = ArrowMask(signal.shape_hint);
      if (Conflicts(visual_mask, signal_mask) ||
          Conflicts(visual_mask, hint_mask)) {
        continue;
      }
      const bool lane_match =
          lane != nullptr && signal.lane_id == lane->lane_id &&
          (lane->approach_id.empty() || signal.approach_id.empty() ||
           lane->approach_id == signal.approach_id) &&
          signal_mask != kMovementMaskNone &&
          (signal_mask & lane->movement_mask) != 0u;
      if (lane != nullptr && !lane_match) {
        continue;
      }
      const float geometry =
          signal.projection_reliable && signal.camera_name == visual.camera_name
              ? ProjectionSupport(visual.bbox, signal.projection_roi)
              : 0.0f;
      if (!lane_match && geometry <= 0.0f) {
        continue;
      }
      const float arrow_support =
          visual_mask != kMovementMaskNone && signal_mask != kMovementMaskNone
              ? MovementSimilarity(visual_mask, signal_mask)
              : 0.0f;
      float group_support = 0.0f;
      if (observed_group_size > 0 && signal.expected_group_size > 0) {
        group_support = static_cast<float>(std::min(
                            observed_group_size, signal.expected_group_size)) /
                        static_cast<float>(std::max(
                            observed_group_size, signal.expected_group_size));
      }
      // Arrow specificity and observed cardinality can distinguish clear cases;
      // small cardinality differences still remain inside the ambiguity margin.
      // Counts remain soft support, never a hard visibility/cardinality veto.
      // Neither color nor a prompt-inherited signal ID is identity evidence.
      const float score =
          kLaneWeight * (lane_match ? UnitScore(lane->confidence) : geometry) +
          kTopologyWeight * UnitScore(signal.topology_confidence) +
          kSignalConfidenceWeight * UnitScore(signal.confidence) +
          kArrowWeight * arrow_support + kProjectionWeight * geometry +
          kGroupWeight * group_support;
      if (score < options_.min_bind_score) {
        continue;
      }
      if (prior != history_.end() &&
          prior->second.signal_id == signal.signal_id) {
        previous = &signal;
        previous_score = score;
      }
      if (score > best_score) {
        runner_up = best_score;
        best_score = score;
        best = &signal;
      } else {
        runner_up = std::max(runner_up, score);
      }
    }
    if (best == nullptr) {
      return bound;
    }
    const bool ambiguous =
        runner_up >= 0.0f && best_score - runner_up <= kSignalAmbiguityMargin;
    if (ambiguous) {
      if (previous == nullptr ||
          best_score - previous_score > kSignalAmbiguityMargin) {
        bound.binding_ambiguous = true;
        bound.binding_reason = "near-equal signal candidates";
        return bound;
      }
      best = previous;
      best_score = previous_score;
    }
    bound.binding_valid = true;
    bound.binding_reason =
        ambiguous ? "bounded historical relation with current lane support"
                  : (lane != nullptr ? "fresh lane topology"
                                     : "reliable map projection");
    // A prior resolves short ambiguities but cannot renew its own lifetime.
    if (!ambiguous && lane != nullptr && visual.track_id > 0) {
      history_[key] = {best->signal_id, lane->lane_id, lane->approach_id,
                       static_cast<double>(context.timestamp) * 1e-9};
    }
    bound.bind_score = best_score;
    bound.topology_confidence = UnitScore(best->topology_confidence);
    bound.visual_light.signal_id = best->signal_id;
    bound.signal_group_id = best->signal_group_id;
    bound.lane_id = best->lane_id;
    bound.signal_movement_mask = SignalMask(*best);
    bound.bound_intent = PrimaryLaneIntentFromMask(bound.signal_movement_mask);
    bound.movement_match_score =
        visual_mask != kMovementMaskNone &&
                bound.signal_movement_mask != kMovementMaskNone
            ? 1.0f
            : 0.0f;
    bound.stopline_distance_m = best->stopline_distance_m;
    bound.controlling_ego_lane = lane != nullptr;
    return bound;
  }

  static int ObservedGroupSize(const std::vector<VisualLight>& lights,
                               const std::vector<bool>& current_observations,
                               size_t index) {
    const auto& visual = lights[index];
    if (visual.observed_group_size > 0) {
      return visual.observed_group_size;
    }
    int count = 1;
    for (size_t i = 0; i < lights.size(); ++i) {
      const auto& other = lights[i];
      if (i == index || !current_observations[i] ||
          visual.camera_name != other.camera_name) {
        continue;
      }
      if (!visual.observed_group_id.empty()) {
        count += visual.observed_group_id == other.observed_group_id ? 1 : 0;
        continue;
      }
      if (!other.observed_group_id.empty() || !visual.bbox.IsValid() ||
          !other.bbox.IsValid()) {
        continue;
      }
      const auto& a = visual.bbox;
      const auto& b = other.bbox;
      const float scale = std::max(a.width, a.height);
      const float other_scale = std::max(b.width, b.height);
      if (other_scale < 0.5f * scale || other_scale > 2.0f * scale) {
        continue;
      }
      const float dx =
          std::abs((a.x + 0.5f * a.width) - (b.x + 0.5f * b.width));
      const float dy =
          std::abs((a.y + 0.5f * a.height) - (b.y + 0.5f * b.height));
      const bool row = dy <= 0.5f * std::max(a.height, b.height) &&
                       dx <= 3.0f * std::max(a.width, b.width);
      const bool column = dx <= 0.5f * std::max(a.width, b.width) &&
                          dy <= 3.0f * std::max(a.height, b.height);
      count += row || column ? 1 : 0;
    }
    return count;
  }

  float ProjectionSupport(const Rect2f& lhs, const Rect2f& rhs) const {
    if (!lhs.IsValid() || !rhs.IsValid()) {
      return 0.0f;
    }
    const float x1 = std::max(lhs.x, rhs.x);
    const float y1 = std::max(lhs.y, rhs.y);
    const float x2 = std::min(lhs.x + lhs.width, rhs.x + rhs.width);
    const float y2 = std::min(lhs.y + lhs.height, rhs.y + rhs.height);
    const float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    const float total = lhs.Area() + rhs.Area() - inter;
    const float overlap = total > 1e-6f ? inter / total : 0.0f;
    const float dx = lhs.x + lhs.width * 0.5f - rhs.x - rhs.width * 0.5f;
    const float dy = lhs.y + lhs.height * 0.5f - rhs.y - rhs.height * 0.5f;
    const float distance = std::sqrt(dx * dx + dy * dy) /
                           std::max(1.0f, std::max(rhs.width, rhs.height));
    if (!std::isfinite(distance) ||
        (overlap < options_.min_signal_overlap &&
         distance > options_.max_center_distance_ratio)) {
      return 0.0f;
    }
    const float scale = std::max(0.1f, options_.max_center_distance_ratio);
    return std::max(overlap, 1.0f - std::min(1.0f, distance / scale));
  }

  BinderOptions options_;
  uint64_t epoch_ = 0;
  double last_frame_sec_ = -1.0;
  std::map<std::pair<std::string, int>, BindingPrior> history_;
};

}  // namespace traffic_light
}  // namespace apollo
