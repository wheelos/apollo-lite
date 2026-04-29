#pragma once

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/traffic_light/interface/stage.h"

namespace apollo {
namespace perception {
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

    if (context->runtime_state != nullptr) {
      for (const auto& track : context->runtime_state->tracked_memory) {
        if (!track.confirmed ||
            (track.current_state.visual_light.camera_name !=
                 context->primary_camera_name &&
             !track.current_state.visual_light.camera_name.empty())) {
          continue;
        }
        PromptRegion prompt;
        prompt.roi_box = track.current_state.visual_light.bbox;
        ExpandBox(&prompt.roi_box, options_.history_expand_ratio, frame_width,
                  frame_height);
        prompt.weight = options_.history_weight;
        prompt.source = PromptSource::HISTORY;
        prompt.signal_id = track.current_state.visual_light.signal_id;
        prompt.camera_name = track.current_state.visual_light.camera_name;
        prompt.intended_movement = track.current_state.bound_intent;
        prompt.movement_mask = track.current_state.signal_movement_mask;
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
      if (!signal.projection_roi.IsValid()) {
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

    if (prompts.empty() || options_.always_add_full_frame_fallback) {
      PromptRegion fallback;
      fallback.roi_box = Rect2f{
          0.0f, 0.0f, static_cast<float>(frame_width),
          static_cast<float>(frame_height),
      };
      fallback.weight = options_.full_frame_weight;
      fallback.source = PromptSource::FULL_FRAME;
      fallback.camera_name = context->primary_camera_name;
      fallback.intended_movement = context->nav_topology.ego_lane_intent;
      fallback.movement_mask =
          MovementMaskFromLaneIntent(context->nav_topology.ego_lane_intent);
      prompts.push_back(fallback);
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
    context->metrics.prompter.note =
        context->status.using_full_frame_fallback ? "full-frame fallback active"
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
    if (!context.camera_frames.empty() && context.camera_frames.front().image.cols > 0) {
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
    if (!context.camera_frames.empty() && context.camera_frames.front().image.rows > 0) {
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
    if (context.runtime_state != nullptr) {
      return context.runtime_state->cached_signals;
    }
    return context.map_signals;
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
    box->x = std::max(0.0f, center_x - box->width * 0.5f);
    box->y = std::max(0.0f, center_y - box->height * 0.5f);
    box->width = std::max(
        0.0f, std::min(box->width, static_cast<float>(frame_width) - box->x));
    box->height = std::max(
        0.0f, std::min(box->height, static_cast<float>(frame_height) - box->y));
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
        const bool same_signal = !prompt.signal_id.empty() &&
                                 prompt.signal_id == existing.signal_id;
        const bool same_box = ComputeIou(prompt.roi_box, existing.roi_box) > 0.9f;
        if (!same_signal && !same_box) {
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
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
