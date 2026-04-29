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
    const bool has_raw_yolo_candidates =
        options_.prefer_raw_yolo_candidates && !context->raw_yolo_lights.empty();
    if (frame == nullptr ||
        ((!has_raw_yolo_candidates) &&
         (frame->image.data == nullptr || frame->image.cols <= 0 ||
          frame->image.rows <= 0 || frame->image.channels < 3))) {
      context->AppendDegradeReason("detector missing image");
      return false;
    }

    std::vector<VisualLight> detections;
    int glare_rejections = 0;
    if (has_raw_yolo_candidates &&
        options_.backend == DetectorBackendType::YOLO) {
      detections = DecodeRawYoloCandidates(*context, *frame);
      context->metrics.detector.note = "decoded raw yolo detections";
    } else {
      for (const auto& prompt : context->prompts) {
        if (prompt.camera_name != frame->camera_name &&
            !prompt.camera_name.empty()) {
          continue;
        }
        if (prompt.source != PromptSource::FULL_FRAME &&
            prompt.weight < options_.min_prompt_weight) {
          continue;
        }
        Rect2f roi = ClampBox(prompt.roi_box, frame->image.cols, frame->image.rows);
        if (roi.Area() < options_.min_roi_area) {
          continue;
        }

        const SignalCandidate* signal = FindBestSignal(*context, prompt, roi);
        const float signal_overlap = signal == nullptr
                                         ? 0.0f
                                         : ComputeIou(roi, signal->projection_roi);
        const ColorDecision color = ClassifyColor(frame->image, roi);
        if (color.glare_suspected &&
            signal_overlap < options_.min_signal_overlap_for_glare_override &&
            prompt.source != PromptSource::HISTORY) {
          context->status.glare_detected = true;
          ++glare_rejections;
          continue;
        }
        if (color.color == LightColor::UNKNOWN && !options_.allow_unknown_output &&
            prompt.source == PromptSource::FULL_FRAME && signal == nullptr) {
          continue;
        }

        VisualLight light;
        light.bbox = roi;
        light.color = color.color;
        light.shape = InferShape(prompt, signal);
        light.existence_confidence = color.existence_confidence;
        light.state_confidence = color.state_confidence;
        light.glare_suspected = color.glare_suspected;
        light.confidence = std::min(
            1.0f,
            0.45f * light.existence_confidence +
                0.45f * light.state_confidence + 0.10f * prompt.weight +
                options_.map_overlap_bonus * signal_overlap +
                (prompt.source == PromptSource::HISTORY ? options_.history_bonus
                                                        : 0.0f));
        light.signal_id = signal != nullptr ? signal->signal_id : prompt.signal_id;
        light.camera_name = frame->camera_name;
        light.blink = color.blink;
        light.prompt_source = prompt.source;
        light.intended_movement =
            signal != nullptr ? signal->intended_movement : prompt.intended_movement;
        light.movement_mask =
            signal != nullptr
                ? NormalizeMovementMask(signal->movement_mask,
                                        signal->intended_movement,
                                        signal->shape_hint)
                : NormalizeMovementMask(prompt.movement_mask,
                                        prompt.intended_movement, light.shape);
        light.stopline_distance_m =
            signal != nullptr ? signal->stopline_distance_m : -1.0;
        detections.push_back(light);
      }
    }

    context->status.detector_ran = true;
    context->visual_lights = Deduplicate(detections);
    context->metrics.detector.input_count =
        static_cast<int>(context->prompts.size());
    context->metrics.detector.output_count =
        static_cast<int>(context->visual_lights.size());
    context->metrics.detector.rejected_count = glare_rejections;
    for (const auto& light : context->visual_lights) {
      context->metrics.detector.max_confidence =
          std::max(context->metrics.detector.max_confidence, light.confidence);
      context->metrics.detector.avg_confidence += light.confidence;
    }
    if (!context->visual_lights.empty()) {
      context->metrics.detector.avg_confidence /=
          static_cast<float>(context->visual_lights.size());
    }
    if (glare_rejections > 0) {
      context->metrics.detector.note = "glare-filtered candidates present";
    }
    return true;
  }

 private:
  struct ColorDecision {
    LightColor color = LightColor::UNKNOWN;
    float existence_confidence = 0.0f;
    float state_confidence = 0.0f;
    bool blink = false;
    bool glare_suspected = false;
  };

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

  const SignalCandidate* FindBestSignal(const PipelineContext& context,
                                        const PromptRegion& prompt,
                                        const Rect2f& roi) const {
    const SignalCandidate* best = nullptr;
    float best_score = -1.0f;
    for (const auto& signal : context.map_signals) {
      if (!prompt.signal_id.empty() && !signal.signal_id.empty() &&
          prompt.signal_id == signal.signal_id) {
        return &signal;
      }
      if (!signal.camera_name.empty() && !prompt.camera_name.empty() &&
          signal.camera_name != prompt.camera_name) {
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

  ColorDecision ClassifyColor(const Image& image, const Rect2f& roi) const {
    ColorDecision decision;
    const int x_start = std::max(0, static_cast<int>(roi.x));
    const int y_start = std::max(0, static_cast<int>(roi.y));
    const int x_end =
        std::min(image.cols, static_cast<int>(std::ceil(roi.x + roi.width)));
    const int y_end =
        std::min(image.rows, static_cast<int>(std::ceil(roi.y + roi.height)));
    const int step_x = std::max(1, (x_end - x_start) / options_.sample_grid_divisor);
    const int step_y = std::max(1, (y_end - y_start) / options_.sample_grid_divisor);
    const bool is_bgr = image.encoding.find("bgr") != std::string::npos;

    float red_score = 0.0f;
    float green_score = 0.0f;
    float yellow_score = 0.0f;
    int active_pixels = 0;
    int white_pixels = 0;
    int sampled_pixels = 0;
    for (int y = y_start; y < y_end; y += step_y) {
      for (int x = x_start; x < x_end; x += step_x) {
        const int offset = (y * image.cols + x) * image.channels;
        const uint8_t c0 = image.data[offset];
        const uint8_t c1 = image.data[offset + 1];
        const uint8_t c2 = image.data[offset + 2];
        const float r = static_cast<float>(is_bgr ? c2 : c0);
        const float g = static_cast<float>(c1);
        const float b = static_cast<float>(is_bgr ? c0 : c2);
        const float brightness = std::max(r, std::max(g, b));
        ++sampled_pixels;
        if (brightness < 70.0f) {
          continue;
        }
        ++active_pixels;
        if (brightness > 220.0f && std::fabs(r - g) < 25.0f &&
            std::fabs(r - b) < 25.0f && std::fabs(g - b) < 25.0f) {
          ++white_pixels;
        }
        if (r > 1.25f * g && r > 1.25f * b) {
          red_score += r - std::max(g, b);
        } else if (g > 1.20f * r && g > 1.20f * b) {
          green_score += g - std::max(r, b);
        } else if (r > 90.0f && g > 90.0f && b < 0.8f * std::min(r, g)) {
          yellow_score += (r + g) * 0.5f - b;
        }
      }
    }

    if (sampled_pixels == 0) {
      return decision;
    }
    const float active_ratio =
        static_cast<float>(active_pixels) / static_cast<float>(sampled_pixels);
    const float total_score = red_score + green_score + yellow_score;
    decision.existence_confidence = std::min(
        1.0f, 0.20f + 0.80f * std::min(1.0f, active_ratio / 0.18f));
    if (active_ratio < options_.min_bright_pixel_ratio || total_score <= 1e-3f) {
      return decision;
    }
    const float white_ratio =
        active_pixels > 0 ? static_cast<float>(white_pixels) /
                                static_cast<float>(active_pixels)
                          : 0.0f;

    decision.color = LightColor::RED;
    float best_score = red_score;
    if (green_score > best_score) {
      decision.color = LightColor::GREEN;
      best_score = green_score;
    }
    if (yellow_score > best_score) {
      decision.color = LightColor::YELLOW;
      best_score = yellow_score;
    }
    const float color_ratio = best_score / total_score;
    decision.glare_suspected =
        white_ratio >= options_.glare_white_ratio_threshold &&
        color_ratio < options_.min_color_ratio + 0.10f;
    if (color_ratio < options_.min_color_ratio) {
      decision.color = LightColor::UNKNOWN;
      decision.state_confidence = 0.0f;
      return decision;
    }
    decision.state_confidence = std::min(
        1.0f, 0.25f + 0.5f * color_ratio +
                  0.25f * std::min(1.0f, active_ratio));
    if (decision.glare_suspected) {
      decision.state_confidence =
          std::max(0.0f, decision.state_confidence - options_.glare_penalty);
    }
    return decision;
  }

  static LightShape InferShape(const PromptRegion& prompt,
                               const SignalCandidate* signal) {
    if (signal != nullptr && signal->shape_hint != LightShape::UNKNOWN) {
      return signal->shape_hint;
    }
    const uint32_t movement_mask =
        signal != nullptr
            ? NormalizeMovementMask(signal->movement_mask,
                                    signal->intended_movement,
                                    signal->shape_hint)
            : NormalizeMovementMask(prompt.movement_mask,
                                    prompt.intended_movement, LightShape::UNKNOWN);
    if (movement_mask == kMovementMaskLeft) {
      return LightShape::ARROW_LEFT;
    }
    if (movement_mask == kMovementMaskRight) {
      return LightShape::ARROW_RIGHT;
    }
    if (movement_mask == kMovementMaskStraight) {
      return LightShape::ARROW_STRAIGHT;
    }
    switch (signal != nullptr ? signal->intended_movement
                              : prompt.intended_movement) {
      case LaneIntent::LEFT:
        return LightShape::ARROW_LEFT;
      case LaneIntent::RIGHT:
        return LightShape::ARROW_RIGHT;
      case LaneIntent::STRAIGHT:
        return LightShape::ARROW_STRAIGHT;
      default:
        return LightShape::CIRCLE;
    }
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
