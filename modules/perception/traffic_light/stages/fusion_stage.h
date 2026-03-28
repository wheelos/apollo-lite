#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "modules/perception/traffic_light/interface/stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {

// Step 6: 概率融合与仲裁 (Fusion & Arbitration)
// 职责: 结合时序稳定后的主视觉和启发式旁路的概率，做最终裁决输出。
class FusionStage : public BaseStage {
public:
    FusionStage() = default;
    ~FusionStage() override = default;

    std::string Name() const override { return "FusionStage"; }

    bool Init(const StageConfig& config) override {
        (void)config;
        return true;
    }

    bool Process(PipelineContext* context) override {
        if (!context) return false;
        std::lock_guard<std::mutex> lock(context->rw_mutex);

        TrafficLightResult result;

        const TrackedLight* best_visual = SelectBestVisualTrack(*context);
        if (best_visual != nullptr) {
            result.color = best_visual->current_state.visual_light.color;
            result.shape = best_visual->current_state.visual_light.shape;
            result.confidence = best_visual->current_state.visual_light.confidence;
            result.signal_id = best_visual->current_state.visual_light.signal_id;
            result.blink = best_visual->current_state.visual_light.blink;
            result.source = EvidenceSource::VISION;
            result.decision_reason =
                "Vision track " + std::to_string(best_visual->track_id);
        }

        const V2XLightEvidence* v2x = SelectBestV2XEvidence(*context, result.signal_id);
        if (v2x != nullptr) {
            const bool no_visual = result.source == EvidenceSource::UNKNOWN;
            const bool visual_weak = result.confidence < kWeakVisionThreshold;
            const bool v2x_strong = v2x->confidence >= kStrongV2XThreshold;
            const bool color_conflict =
                !no_visual && result.color != LightColor::UNKNOWN &&
                v2x->color != LightColor::UNKNOWN && result.color != v2x->color;

            if (no_visual || (visual_weak && v2x_strong) || color_conflict) {
                result.color = v2x->color;
                result.blink = v2x->blink;
                result.confidence = std::max(result.confidence, v2x->confidence);
                result.signal_id = v2x->signal_id;
                result.source = EvidenceSource::V2X;
                result.decision_reason =
                    "V2X override (ts=" + std::to_string(v2x->timestamp_sec) + ")";
            } else if (!color_conflict && result.source == EvidenceSource::VISION) {
                result.confidence =
                    std::min(1.0f, result.confidence + 0.1f * v2x->confidence);
                result.decision_reason += ", corroborated by V2X";
            }
        }

        // 启发式作为最后兜底，仅在视觉/V2X都不可靠时启用。
        if (result.confidence < kHeuristicTriggerThreshold &&
            context->heuristic_state.probability > kHeuristicAcceptThreshold) {
            result.color = context->heuristic_state.inferred_color;
            result.shape = LightShape::CIRCLE;
            result.confidence = context->heuristic_state.probability;
            result.is_heuristic_override = true;
            result.source = EvidenceSource::HEURISTIC;
            result.decision_reason =
                "Heuristic fallback: " + context->heuristic_state.inference_reason;
        }

        context->final_decision = result;
        return true;
    }

private:
    const TrackedLight* SelectBestVisualTrack(const PipelineContext& context) const {
        const TrackedLight* best = nullptr;
        float best_score = -1.0f;
        for (const auto& track : context.tracked_lights) {
            const float score =
                track.current_state.visual_light.confidence *
                std::max(0.2f, track.current_state.bind_score);
            if (score > best_score) {
                best = &track;
                best_score = score;
            }
        }
        return best;
    }

    const V2XLightEvidence* SelectBestV2XEvidence(
        PipelineContext& context, const std::string& signal_id) const {
        if (context.runtime_state != nullptr) {
            for (const auto& item : context.v2x_lights) {
                context.runtime_state->v2x_buffer.push_back(item);
            }
            context.runtime_state->TrimV2XBuffer(kMaxV2XBufferSize);
        }

        const double frame_ts_sec = static_cast<double>(context.timestamp) * 1e-9;
        const V2XLightEvidence* best = nullptr;
        float best_score = -1.0f;

        auto score_candidate = [&](const V2XLightEvidence& e) {
            const double dt = std::fabs(frame_ts_sec - e.timestamp_sec);
            if (dt > kV2XSyncWindowSec) {
                return;
            }
            if (!signal_id.empty() && !e.signal_id.empty() && signal_id != e.signal_id) {
                return;
            }
            const float recency =
                static_cast<float>(std::max(0.0, 1.0 - dt / kV2XSyncWindowSec));
            const float score = 0.7f * e.confidence + 0.3f * recency;
            if (score > best_score) {
                best = &e;
                best_score = score;
            }
        };

        for (const auto& e : context.v2x_lights) {
            score_candidate(e);
        }
        if (context.runtime_state != nullptr) {
            for (const auto& e : context.runtime_state->v2x_buffer) {
                score_candidate(e);
            }
        }
        return best;
    }

    static constexpr float kWeakVisionThreshold = 0.60f;
    static constexpr float kStrongV2XThreshold = 0.70f;
    static constexpr float kHeuristicTriggerThreshold = 0.40f;
    static constexpr float kHeuristicAcceptThreshold = 0.80f;
    static constexpr double kV2XSyncWindowSec = 0.30;
    static constexpr size_t kMaxV2XBufferSize = 64;
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo
