#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "modules/perception/traffic_light/interface/stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class DetectorStage : public BaseStage {
 public:
  explicit DetectorStage(const DetectorOptions& options)
      : BaseStage(false), options_(options) {}

  std::string Name() const override { return "DetectorStage"; }

  bool Process(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }

    std::lock_guard<std::mutex> lock(context->rw_mutex);
    context->metrics.detector = StageRuntimeMetrics();
    const CameraFrameState* frame = SelectPrimaryFrame(*context);
    const bool has_raw_yolo_candidates = !context->raw_yolo_lights.empty();
    if (frame == nullptr || frame->image.cols <= 0 || frame->image.rows <= 0) {
      context->AppendDegradeReason("detector missing image");
      return false;
    }

    if (!context->status.neural_detector_ran) {
      context->AppendDegradeReason("neural detector unavailable");
      context->metrics.detector.note =
          "neural detector unavailable; publishing degraded empty result";
      context->status.detector_ran = true;
      context->visual_lights.clear();
      context->metrics.detector.input_count =
          static_cast<int>(context->prompts.size());
      context->metrics.detector.output_count = 0;
      return true;
    }
    std::vector<VisualLight> detections = DecodeRawYoloCandidates(*context, *frame);
    context->metrics.detector.note =
        has_raw_yolo_candidates ? "decoded raw yolo detections"
                                : "neural detector ran with zero candidates";

    context->status.detector_ran = true;
    context->visual_lights = Deduplicate(detections);
    context->metrics.detector.input_count =
        static_cast<int>(context->prompts.size());
    context->metrics.detector.output_count =
        static_cast<int>(context->visual_lights.size());
    for (const auto& light : context->visual_lights) {
      context->metrics.detector.max_confidence =
          std::max(context->metrics.detector.max_confidence, light.confidence);
      context->metrics.detector.avg_confidence += light.confidence;
    }
    if (!context->visual_lights.empty()) {
      context->metrics.detector.avg_confidence /=
          static_cast<float>(context->visual_lights.size());
    }
    return true;
  }

 private:
  const CameraFrameState* SelectPrimaryFrame(const PipelineContext& context) const {
    for (const auto& frame : context.camera_frames) {
      if (frame.camera_name == context.primary_camera_name) {
        return &frame;
      }
    }
    if (!context.camera_frames.empty()) {
      return &context.camera_frames.front();
    }
    return nullptr;
  }

  std::vector<VisualLight> DecodeRawYoloCandidates(
      const PipelineContext& context, const CameraFrameState& frame) const {
    std::vector<VisualLight> detections;
    for (const auto& candidate : context.raw_yolo_lights) {
      if (!candidate.camera_name.empty() &&
          candidate.camera_name != frame.camera_name) {
        continue;
      }
      if (candidate.objectness < options_.min_objectness ||
          candidate.semantic_confidence < options_.min_semantic_confidence) {
        continue;
      }
      Rect2f roi = candidate.bbox;
      if (frame.image.cols > 0 && frame.image.rows > 0) {
        roi = ClampBox(candidate.bbox, frame.image.cols, frame.image.rows);
      }
      if (!roi.IsValid() || roi.Area() < options_.min_roi_area) {
        continue;
      }

      const SignalCandidate* signal =
          FindBestSignal(context, candidate.signal_id, candidate.camera_name, roi);
      const float signal_overlap = signal == nullptr
                                       ? 0.0f
                                       : ComputeIou(roi, signal->projection_roi);
      VisualLight light;
      light.bbox = roi;
      light.color = candidate.color;
      light.shape = candidate.shape;
      light.existence_confidence = candidate.objectness;
      light.state_confidence = candidate.semantic_confidence;
      light.confidence = std::min(
          1.0f, 0.50f * candidate.objectness +
                    0.35f * candidate.semantic_confidence +
                    0.15f * signal_overlap);
      light.signal_id = !candidate.signal_id.empty()
                            ? candidate.signal_id
                            : (signal != nullptr ? signal->signal_id : "");
      light.camera_name = frame.camera_name;
      light.blink = candidate.blink;
      light.yolo_class_id = candidate.class_id;
      light.yolo_class_name = candidate.class_name;
      light.prompt_source =
          signal != nullptr ? PromptSource::MAP : PromptSource::FULL_FRAME;
      light.intended_movement =
          signal != nullptr
              ? signal->intended_movement
              : PrimaryLaneIntentFromMask(candidate.movement_mask);
      light.movement_mask =
          signal != nullptr
              ? NormalizeMovementMask(signal->movement_mask,
                                      signal->intended_movement,
                                      signal->shape_hint)
              : NormalizeMovementMask(candidate.movement_mask,
                                      PrimaryLaneIntentFromMask(
                                          candidate.movement_mask),
                                      candidate.shape);
      light.stopline_distance_m =
          signal != nullptr ? signal->stopline_distance_m : -1.0;
      detections.push_back(light);
    }
    return detections;
  }

  const SignalCandidate* FindBestSignal(const PipelineContext& context,
                                        const std::string& signal_id,
                                        const std::string& camera_name,
                                        const Rect2f& roi) const {
    const SignalCandidate* best = nullptr;
    float best_score = -1.0f;
    for (const auto& signal : context.map_signals) {
      if (!signal_id.empty() && signal.signal_id == signal_id) {
        return &signal;
      }
      if (!camera_name.empty() && !signal.camera_name.empty() &&
          signal.camera_name != camera_name) {
        continue;
      }
      const float score = ComputeIou(signal.projection_roi, roi) + signal.confidence;
      if (score > best_score) {
        best_score = score;
        best = &signal;
      }
    }
    return best;
  }

  static Rect2f ClampBox(const Rect2f& box, int width, int height) {
    Rect2f clamped = box;
    clamped.x = std::max(0.0f, box.x);
    clamped.y = std::max(0.0f, box.y);
    clamped.width =
        std::max(0.0f, std::min(box.width, static_cast<float>(width) - clamped.x));
    clamped.height =
        std::max(0.0f, std::min(box.height, static_cast<float>(height) - clamped.y));
    return clamped;
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

  static std::vector<VisualLight> Deduplicate(
      const std::vector<VisualLight>& detections) {
    std::vector<VisualLight> deduped;
    for (const auto& detection : detections) {
      bool merged = false;
      for (auto& existing : deduped) {
        const bool same_signal = !detection.signal_id.empty() &&
                                 detection.signal_id == existing.signal_id;
        const bool same_box = ComputeIou(detection.bbox, existing.bbox) > 0.85f;
        if (!same_signal && !same_box) {
          continue;
        }
        if (detection.confidence > existing.confidence) {
          existing = detection;
        }
        merged = true;
        break;
      }
      if (!merged) {
        deduped.push_back(detection);
      }
    }
    return deduped;
  }

  DetectorOptions options_;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
