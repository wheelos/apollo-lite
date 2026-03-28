#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "modules/perception/traffic_light/interface/stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {

// Step 3: 拓扑绑定 (Topology Binding)
// 职责: 将天空中的 2D 实体，利用空间先验和导航意图进行降噪过滤，仅关联本车道需遵守的灯
class SemanticBinderStage : public BaseStage {
public:
    SemanticBinderStage() = default;
    ~SemanticBinderStage() override = default;

    std::string Name() const override { return "SemanticBinderStage"; }

    bool Init(const StageConfig& config) override {
        (void)config;
        return true;
    }

    bool Process(PipelineContext* context) override {
        if (!context) return false;
        std::lock_guard<std::mutex> lock(context->rw_mutex);

        std::vector<BoundLight> mapped_lights;
        const LaneIntent ego_intent = context->nav_topology.ego_lane_intent;

        for (const auto& v_light : context->visual_lights) {
            BoundLight b_light;
            b_light.visual_light = v_light;

            const LaneIntent shape_intent = InferIntentFromShape(v_light.shape);

            float best_score = -1.0f;
            const SignalCandidate* best_signal = nullptr;
            for (const auto& signal : context->map_signals) {
                if (!IsIntentCompatible(shape_intent, signal.intended_movement,
                                        ego_intent)) {
                    continue;
                }
                const float iou = ComputeIou(v_light.bbox, signal.projection_roi);
                if (iou > best_score) {
                    best_score = iou;
                    best_signal = &signal;
                }
            }

            if (best_signal != nullptr) {
                b_light.bound_intent = best_signal->intended_movement;
                b_light.bind_score = std::max(0.0f, best_score) *
                                     std::max(0.2f, best_signal->confidence);
                b_light.stopline_distance_m = best_signal->stopline_distance_m;
                b_light.visual_light.signal_id = best_signal->signal_id;
            } else {
                b_light.bound_intent = shape_intent;
                b_light.bind_score = std::max(0.05f, 0.3f * v_light.confidence);
            }

            if (b_light.bound_intent != LaneIntent::UNKNOWN &&
                ego_intent != LaneIntent::UNKNOWN &&
                b_light.bound_intent != ego_intent) {
                continue;
            }
            mapped_lights.push_back(b_light);
        }

        std::sort(mapped_lights.begin(), mapped_lights.end(),
                  [](const BoundLight& lhs, const BoundLight& rhs) {
                      return lhs.bind_score > rhs.bind_score;
                  });

        context->bound_lights = std::move(mapped_lights);
        return true;
    }

private:
    LaneIntent InferIntentFromShape(LightShape shape) const {
        switch (shape) {
            case LightShape::ARROW_LEFT:
                return LaneIntent::LEFT;
            case LightShape::ARROW_RIGHT:
                return LaneIntent::RIGHT;
            case LightShape::ARROW_STRAIGHT:
            case LightShape::CIRCLE:
                return LaneIntent::STRAIGHT;
            default:
                return LaneIntent::UNKNOWN;
        }
    }

    bool IsIntentCompatible(LaneIntent shape_intent, LaneIntent signal_intent,
                            LaneIntent ego_intent) const {
        if (ego_intent != LaneIntent::UNKNOWN && signal_intent != LaneIntent::UNKNOWN &&
            signal_intent != ego_intent) {
            return false;
        }
        if (shape_intent != LaneIntent::UNKNOWN && signal_intent != LaneIntent::UNKNOWN &&
            shape_intent != signal_intent) {
            return false;
        }
        return true;
    }

    float ComputeIou(const Rect2f& a, const Rect2f& b) const {
        const float ax2 = a.x + a.width;
        const float ay2 = a.y + a.height;
        const float bx2 = b.x + b.width;
        const float by2 = b.y + b.height;

        const float inter_x1 = std::max(a.x, b.x);
        const float inter_y1 = std::max(a.y, b.y);
        const float inter_x2 = std::min(ax2, bx2);
        const float inter_y2 = std::min(ay2, by2);

        const float inter_w = std::max(0.0f, inter_x2 - inter_x1);
        const float inter_h = std::max(0.0f, inter_y2 - inter_y1);
        const float inter_area = inter_w * inter_h;
        const float union_area = a.width * a.height + b.width * b.height - inter_area;
        if (union_area <= std::numeric_limits<float>::epsilon()) {
            return 0.0f;
        }
        return inter_area / union_area;
    }
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo
