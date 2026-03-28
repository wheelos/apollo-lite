#pragma once

#include <algorithm>
#include <map>
#include <set>

#include "modules/perception/traffic_light/interface/stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {

// Step 4: 时序记忆 (Temporal Tracking)
// 职责: 维持多帧连续性，抗大卡车短暂遮挡 (Occlusion) 和 LED 频闪去抖。
class TrackerStage : public BaseStage {
public:
    TrackerStage() = default;
    ~TrackerStage() override = default;

    std::string Name() const override { return "TrackerStage"; }

    bool Init(const StageConfig& config) override {
        (void)config;
        return true;
    }

    bool Process(PipelineContext* context) override {
        if (!context) return false;
        std::lock_guard<std::mutex> lock(context->rw_mutex);

        BootstrapFromRuntimeState(context);

        std::vector<TrackedLight> current_tracked;
        std::set<int> matched_ids;

        // IOU Tracker 简易实现：遍历 bound_lights 和上一次的 tracking_pool
        for (const auto& bound : context->bound_lights) {
            int matched_id = MatchWithHistory(bound);
            if (matched_id == -1) {
                // New track
                matched_id = next_track_id_++;
            }

            TrackedLight t_light;
            t_light.track_id = matched_id;
            t_light.current_state = bound;
            t_light.lost_frames = 0;
            current_tracked.push_back(t_light);
            matched_ids.insert(matched_id);

            tracking_pool_[matched_id] = t_light;
        }

        // 处理遮挡找不到了的灯
        for (auto it = tracking_pool_.begin(); it != tracking_pool_.end(); ) {
            if (matched_ids.find(it->first) == matched_ids.end()) {
                it->second.lost_frames++;
                // 预测运动(若静止不管)并推入当前观察作为记忆补偿，容忍 5 帧丢失
                if (it->second.lost_frames <= kMaxLostFrames) {
                    current_tracked.push_back(it->second);
                    ++it;
                } else {
                    it = tracking_pool_.erase(it); // 超时彻底遗忘
                }
            } else {
                ++it;
            }
        }

        context->tracked_lights = current_tracked;
        if (context->runtime_state != nullptr) {
            context->runtime_state->tracked_memory = current_tracked;
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

    int MatchWithHistory(const BoundLight& bl) {
        float best_iou = kMinIouMatch;
        int best_id = -1;
        for (const auto& item : tracking_pool_) {
            const TrackedLight& history = item.second;
            if (!bl.visual_light.signal_id.empty() &&
                !history.current_state.visual_light.signal_id.empty() &&
                bl.visual_light.signal_id !=
                    history.current_state.visual_light.signal_id) {
                continue;
            }

            const float iou = ComputeIou(bl.visual_light.bbox,
                                         history.current_state.visual_light.bbox);
            if (iou > best_iou) {
                best_iou = iou;
                best_id = item.first;
            }
        }
        return best_id;
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
        const float union_area =
            a.width * a.height + b.width * b.height - inter_area;
        if (union_area <= 1e-6f) {
            return 0.0f;
        }
        return inter_area / union_area;
    }

    static constexpr int kMaxLostFrames = 5;
    static constexpr float kMinIouMatch = 0.2f;
    int next_track_id_ = 1;
    std::map<int, TrackedLight> tracking_pool_; // 长期记忆库
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo
