#pragma once

#include <algorithm>
#include <array>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "modules/perception/traffic_light/interface/stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class TrackerStage : public BaseStage {
 public:
  explicit TrackerStage(const TrackerOptions& options)
      : BaseStage(false), options_(options) {}

  std::string Name() const override { return "TrackerStage"; }

  bool Process(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }

    std::lock_guard<std::mutex> lock(context->rw_mutex);
    context->metrics.tracker = StageRuntimeMetrics();
    BootstrapFromRuntimeState(context);
    const double frame_ts_sec = static_cast<double>(context->timestamp) * 1e-9;

    std::set<int> matched_track_ids;
    for (const auto& bound_light : context->bound_lights) {
      const int matched_id = MatchWithHistory(bound_light);
      TrackedLight tracked =
          matched_id >= 0 ? tracking_pool_[matched_id] : MakeNewTrack(bound_light);
      tracked.current_state = bound_light;
      tracked.age += 1;
      tracked.visible_count += 1;
      tracked.lost_frames = 0;
      tracked.confirmed =
          tracked.visible_count >= options_.min_confirmed_visible_count ||
          !bound_light.visual_light.signal_id.empty();
      tracked.last_visible_timestamp_sec = frame_ts_sec;
      UpdateBelief(bound_light.visual_light.color, bound_light.visual_light.confidence,
                   &tracked);
      tracking_pool_[tracked.track_id] = tracked;
      matched_track_ids.insert(tracked.track_id);
    }

    std::vector<TrackedLight> tracked_lights;
    int dropped_tracks = 0;
    for (auto it = tracking_pool_.begin(); it != tracking_pool_.end();) {
      if (matched_track_ids.find(it->first) == matched_track_ids.end()) {
        it->second.age += 1;
        it->second.lost_frames += 1;
        it->second.stabilized_confidence *= options_.belief_decay;
        if (it->second.lost_frames > options_.max_lost_frames) {
          ++dropped_tracks;
          it = tracking_pool_.erase(it);
          continue;
        }
      }
      if (it->second.confirmed) {
        tracked_lights.push_back(it->second);
      }
      ++it;
    }

    std::sort(tracked_lights.begin(), tracked_lights.end(),
              [](const TrackedLight& lhs, const TrackedLight& rhs) {
                if (lhs.stabilized_confidence != rhs.stabilized_confidence) {
                  return lhs.stabilized_confidence > rhs.stabilized_confidence;
                }
                return lhs.current_state.bind_score > rhs.current_state.bind_score;
              });
    context->tracked_lights = tracked_lights;
    context->metrics.tracker.input_count =
        static_cast<int>(context->bound_lights.size());
    context->metrics.tracker.output_count =
        static_cast<int>(context->tracked_lights.size());
    context->metrics.tracker.rejected_count = dropped_tracks;
    for (const auto& track : context->tracked_lights) {
      context->metrics.tracker.max_confidence = std::max(
          context->metrics.tracker.max_confidence, track.stabilized_confidence);
      context->metrics.tracker.avg_confidence += track.stabilized_confidence;
    }
    if (!context->tracked_lights.empty()) {
      context->metrics.tracker.avg_confidence /=
          static_cast<float>(context->tracked_lights.size());
    }

    if (context->runtime_state != nullptr) {
      context->runtime_state->tracked_memory = tracked_lights;
      context->runtime_state->last_frame_id = context->frame_id;
      context->runtime_state->last_processed_ts_sec =
          static_cast<double>(context->timestamp) * 1e-9;
    }
    return true;
  }

 private:
  void BootstrapFromRuntimeState(const PipelineContext* context) {
    if (context == nullptr || context->runtime_state == nullptr ||
        !tracking_pool_.empty()) {
      return;
    }
    for (const auto& track : context->runtime_state->tracked_memory) {
      tracking_pool_[track.track_id] = track;
      next_track_id_ = std::max(next_track_id_, track.track_id + 1);
    }
  }

  TrackedLight MakeNewTrack(const BoundLight& bound_light) {
    TrackedLight tracked;
    tracked.track_id = next_track_id_++;
    tracked.current_state = bound_light;
    tracked.age = 0;
    tracked.visible_count = 0;
    tracked.lost_frames = 0;
    tracked.confirmed = false;
    tracked.color_belief = {{1.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
    return tracked;
  }

  int MatchWithHistory(const BoundLight& bound_light) const {
    for (const auto& item : tracking_pool_) {
      const TrackedLight& history = item.second;
      if (!bound_light.visual_light.signal_id.empty() &&
          bound_light.visual_light.signal_id ==
              history.current_state.visual_light.signal_id) {
        return item.first;
      }
    }

    float best_iou = options_.min_iou_match;
    int best_id = -1;
    for (const auto& item : tracking_pool_) {
      const float iou =
          ComputeIou(bound_light.visual_light.bbox,
                     item.second.current_state.visual_light.bbox);
      if (iou > best_iou) {
        best_iou = iou;
        best_id = item.first;
      }
    }
    return best_id;
  }

  void UpdateBelief(LightColor observed_color, float observed_confidence,
                    TrackedLight* tracked) const {
    if (tracked == nullptr) {
      return;
    }
    for (float& belief : tracked->color_belief) {
      belief *= options_.belief_decay;
    }
    const size_t color_index = static_cast<size_t>(observed_color);
    const float gain = std::max(0.1f, observed_confidence) * options_.measurement_gain;
    if (color_index < tracked->color_belief.size()) {
      tracked->color_belief[color_index] += gain;
    } else {
      tracked->color_belief[0] += gain;
    }

    float sum = 0.0f;
    for (float belief : tracked->color_belief) {
      sum += belief;
    }
    if (sum <= 1e-6f) {
      tracked->stabilized_color = LightColor::UNKNOWN;
      tracked->stabilized_confidence = 0.0f;
      return;
    }

    size_t best_index = 0;
    for (size_t i = 0; i < tracked->color_belief.size(); ++i) {
      tracked->color_belief[i] /= sum;
      if (tracked->color_belief[i] > tracked->color_belief[best_index]) {
        best_index = i;
      }
    }
    tracked->stabilized_color = static_cast<LightColor>(best_index);
    tracked->stabilized_confidence = tracked->color_belief[best_index];
    tracked->blink = tracked->current_state.visual_light.blink;
    tracked->current_state.visual_light.color = tracked->stabilized_color;
    tracked->current_state.visual_light.confidence = std::max(
        tracked->current_state.visual_light.confidence,
        tracked->stabilized_confidence);
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

  TrackerOptions options_;
  int next_track_id_ = 1;
  std::map<int, TrackedLight> tracking_pool_;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
