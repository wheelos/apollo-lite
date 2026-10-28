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

class PrompterStage : public BaseStage {
 public:
  explicit PrompterStage(const PrompterOptions& options)
      : BaseStage(false), options_(options) {}

  std::string Name() const override { return "PrompterStage"; }

  bool Process(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }

    std::lock_guard<std::mutex> lock(context->rw_mutex);
    const int frame_width = GetFrameWidth(*context);
    const int frame_height = GetFrameHeight(*context);
    std::vector<PromptRegion> prompts;
    context->metrics.prompter = StageRuntimeMetrics();
    context->status.using_full_frame_fallback = false;

    const double now = static_cast<double>(context->timestamp) * 1e-9;
    for (const auto& prediction : context->track_predictions) {
      const double age = now - prediction.last_observed_sec;
      if (prediction.camera_name != context->primary_camera_name ||
          !std::isfinite(prediction.position_stddev_px) ||
          prediction.position_stddev_px < 0.0f ||
          std::fabs(now - prediction.timestamp_sec) > 1e-6 || age < 0.0 ||
          age > options_.prediction_max_age_sec ||
          !FiniteBox(prediction.bbox)) {
        continue;
      }
      PromptRegion prompt;
      prompt.roi_box = prediction.bbox;
      const float margin = 2.0f * prediction.position_stddev_px;
      prompt.roi_box.x -= margin;
      prompt.roi_box.y -= margin;
      prompt.roi_box.width += 2.0f * margin;
      prompt.roi_box.height += 2.0f * margin;
      ExpandBox(&prompt.roi_box, options_.history_expand_ratio, frame_width,
                frame_height);
      prompt.weight = options_.history_weight;
      prompt.source = PromptSource::HISTORY;
      prompt.camera_name = prediction.camera_name;
      if (prompt.roi_box.IsValid()) {
        prompts.push_back(prompt);
      }
    }

    const std::vector<SignalCandidate>& candidates =
        SelectSignalCandidates(*context);
    for (const auto& signal : candidates) {
      if (!signal.camera_name.empty() &&
          signal.camera_name != context->primary_camera_name) {
        continue;
      }
      if (!signal.projection_reliable || !FiniteBox(signal.projection_roi)) {
        continue;
      }
      PromptRegion prompt;
      prompt.roi_box = signal.projection_roi;
      ExpandBox(&prompt.roi_box, options_.map_expand_ratio, frame_width,
                frame_height);
      prompt.weight = std::max(options_.map_weight_floor, signal.confidence);
      prompt.source = PromptSource::MAP;
      prompt.signal_id = signal.signal_id;
      prompt.camera_name = signal.camera_name;
      prompt.intended_movement = signal.intended_movement;
      prompt.movement_mask = signal.movement_mask;
      prompts.push_back(prompt);
    }

    if (std::isfinite(context->lane_context.measurement_time_sec) &&
        std::isfinite(context->lane_context.valid_until_sec) &&
        context->lane_context.measurement_time_sec <= now &&
        now <= context->lane_context.valid_until_sec) {
      for (const auto& lane : context->lane_context.candidates) {
        if (lane.camera_name != context->primary_camera_name ||
            !FiniteBox(lane.search_roi) || !std::isfinite(lane.confidence) ||
            lane.confidence <= 0.0f) {
          continue;
        }
        PromptRegion prompt;
        prompt.roi_box = lane.search_roi;
        ExpandBox(&prompt.roi_box, 1.0f, frame_width, frame_height);
        prompt.weight = std::min(1.0f, lane.confidence);
        prompt.source = PromptSource::NAVIGATION;
        prompt.camera_name = lane.camera_name;
        if (prompt.roi_box.IsValid()) {
          prompts.push_back(prompt);
        }
      }
    }

    if (prompts.empty() &&
        context->nav_topology.distance_to_intersection_m > 0.0 &&
        context->nav_topology.distance_to_intersection_m <=
            options_.cold_start_distance_to_intersection_m) {
      PromptRegion prompt;
      const float width = static_cast<float>(frame_width);
      const float height = static_cast<float>(frame_height);
      prompt.roi_box =
          Rect2f{0.2f * width, 0.05f * height, 0.6f * width, 0.45f * height};
      prompt.weight = 0.5f;
      prompt.source = PromptSource::NAVIGATION;
      prompt.camera_name = context->primary_camera_name;
      prompt.intended_movement = context->nav_topology.ego_lane_intent;
      prompt.movement_mask =
          MovementMaskFromLaneIntent(context->nav_topology.ego_lane_intent);
      prompts.push_back(prompt);
    }

    const auto last = last_exploration_sec_.find(context->primary_camera_name);
    const bool exploration_due =
        last == last_exploration_sec_.end() || now < last->second ||
        now - last->second >= options_.full_frame_interval_sec;
    if (prompts.empty() || options_.always_add_full_frame_fallback ||
        exploration_due) {
      PromptRegion fallback;
      fallback.roi_box = Rect2f{
          0.0f,
          0.0f,
          static_cast<float>(frame_width),
          static_cast<float>(frame_height),
      };
      fallback.weight = options_.full_frame_weight;
      fallback.source = PromptSource::FULL_FRAME;
      fallback.camera_name = context->primary_camera_name;
      prompts.push_back(fallback);
      last_exploration_sec_[context->primary_camera_name] = now;
      context->status.using_full_frame_fallback = true;
      context->metrics.prompter.fallback_used = true;
    }

    std::sort(prompts.begin(), prompts.end(),
              [](const PromptRegion& lhs, const PromptRegion& rhs) {
                return lhs.weight > rhs.weight;
              });
    context->prompts = Deduplicate(prompts);
    context->metrics.prompter.input_count = static_cast<int>(candidates.size());
    context->metrics.prompter.output_count =
        static_cast<int>(context->prompts.size());
    context->metrics.prompter.note = context->status.using_full_frame_fallback
                                         ? "full-frame fallback active"
                                         : "map/history guided";
    return !context->prompts.empty();
  }

 private:
  static int GetFrameWidth(const PipelineContext& context) {
    for (const auto& frame : context.camera_frames) {
      if (frame.camera_name == context.primary_camera_name &&
          frame.image.cols > 0) {
        return frame.image.cols;
      }
    }
    if (!context.camera_frames.empty() &&
        context.camera_frames.front().image.cols > 0) {
      return context.camera_frames.front().image.cols;
    }
    if (context.image_tele.cols > 0) {
      return context.image_tele.cols;
    }
    return 1920;
  }

  static int GetFrameHeight(const PipelineContext& context) {
    for (const auto& frame : context.camera_frames) {
      if (frame.camera_name == context.primary_camera_name &&
          frame.image.rows > 0) {
        return frame.image.rows;
      }
    }
    if (!context.camera_frames.empty() &&
        context.camera_frames.front().image.rows > 0) {
      return context.camera_frames.front().image.rows;
    }
    if (context.image_tele.rows > 0) {
      return context.image_tele.rows;
    }
    return 1080;
  }

  const std::vector<SignalCandidate>& SelectSignalCandidates(
      const PipelineContext& context) const {
    if (!context.map_signals.empty()) {
      return context.map_signals;
    }
    return context.map_signals;
  }

  static bool FiniteBox(const Rect2f& box) {
    return box.IsValid() && std::isfinite(box.x) && std::isfinite(box.y) &&
           std::isfinite(box.width) && std::isfinite(box.height);
  }

  static void ExpandBox(Rect2f* box, float ratio, int frame_width,
                        int frame_height) {
    if (box == nullptr || !box->IsValid()) {
      return;
    }
    const float center_x = box->x + box->width * 0.5f;
    const float center_y = box->y + box->height * 0.5f;
    box->width *= ratio;
    box->height *= ratio;
    const float x2 =
        std::min(static_cast<float>(frame_width), center_x + box->width * 0.5f);
    const float y2 = std::min(static_cast<float>(frame_height),
                              center_y + box->height * 0.5f);
    box->x = std::max(0.0f, center_x - box->width * 0.5f);
    box->y = std::max(0.0f, center_y - box->height * 0.5f);
    box->width = std::max(0.0f, x2 - box->x);
    box->height = std::max(0.0f, y2 - box->y);
  }

  static float ComputeIou(const Rect2f& lhs, const Rect2f& rhs) {
    const float x1 = std::max(lhs.x, rhs.x);
    const float y1 = std::max(lhs.y, rhs.y);
    const float x2 = std::min(lhs.x + lhs.width, rhs.x + rhs.width);
    const float y2 = std::min(lhs.y + lhs.height, rhs.y + rhs.height);
    const float width = std::max(0.0f, x2 - x1);
    const float height = std::max(0.0f, y2 - y1);
    const float inter = width * height;
    const float uni = lhs.Area() + rhs.Area() - inter;
    return uni > 1e-6f ? inter / uni : 0.0f;
  }

  static std::vector<PromptRegion> Deduplicate(
      const std::vector<PromptRegion>& prompts) {
    std::vector<PromptRegion> deduped;
    for (const auto& prompt : prompts) {
      bool merged = false;
      for (auto& existing : deduped) {
        if (prompt.source == PromptSource::FULL_FRAME ||
            existing.source == PromptSource::FULL_FRAME) {
          continue;
        }
        const bool same_box =
            ComputeIou(prompt.roi_box, existing.roi_box) > 0.9f;
        if (!same_box) {
          continue;
        }
        if (prompt.weight > existing.weight) {
          existing = prompt;
        }
        merged = true;
        break;
      }
      if (!merged) {
        deduped.push_back(prompt);
      }
    }
    return deduped;
  }

  PrompterOptions options_;
  std::map<std::string, double> last_exploration_sec_;
};

}  // namespace traffic_light
}  // namespace apollo
