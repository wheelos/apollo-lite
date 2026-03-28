#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "modules/perception/traffic_light/interface/stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {

// Step 1: 意图与区域 (Attention ROI)
// 职责: 结合历史Track结果和导航，输出接下来希望检测器重点抠图扫描的 ROI 区域
class PrompterStage : public BaseStage {
public:
    PrompterStage() = default;
    ~PrompterStage() override = default;

    std::string Name() const override { return "PrompterStage"; }

    bool Init(const StageConfig& config) override {
        (void)config;
        return true;
    }

    bool Process(PipelineContext* context) override {
        if (!context) return false;
        std::lock_guard<std::mutex> lock(context->rw_mutex);

        std::vector<PromptRegion> new_prompts;
        const int frame_width = GetFrameWidth(*context);
        const int frame_height = GetFrameHeight(*context);

        // 1) 根据当前帧 Track 生成 Prompt (高优权重)
        for (const auto& track : context->tracked_lights) {
            PromptRegion p;
            p.roi_box = track.current_state.visual_light.bbox;
            ExpandBox(p.roi_box, 1.4f, frame_width, frame_height);
            p.weight = 0.9f;
            p.source = 1; // 1: History
            p.signal_id = track.current_state.visual_light.signal_id;
            new_prompts.push_back(p);
        }

        // 2) 根据跨帧记忆补充 Prompt（对短时遮挡更稳）
        if (context->runtime_state != nullptr && new_prompts.empty()) {
            for (const auto& tracked : context->runtime_state->tracked_memory) {
                PromptRegion p;
                p.roi_box = tracked.current_state.visual_light.bbox;
                ExpandBox(p.roi_box, 1.6f, frame_width, frame_height);
                p.weight = 0.75f;
                p.source = 1;
                p.signal_id = tracked.current_state.visual_light.signal_id;
                new_prompts.push_back(p);
            }
        }

        // 3) 使用 HDMap 信号候选生成几何先验 Prompt
        const std::vector<SignalCandidate>& candidates =
            SelectSignalCandidates(*context);
        for (const auto& signal : candidates) {
            PromptRegion p;
            p.roi_box = signal.projection_roi;
            ExpandBox(p.roi_box, 1.25f, frame_width, frame_height);
            p.weight = 0.7f + 0.2f * std::max(0.0f, std::min(1.0f, signal.confidence));
            p.source = 2; // 2: Nav/Map
            p.signal_id = signal.signal_id;
            new_prompts.push_back(p);
        }

        // 4) 冷启动: 在进入路口前固定区域做弱先验扫描
        if (new_prompts.empty() && context->nav_topology.distance_to_intersection < 120.0) {
            PromptRegion p;
            const float w = static_cast<float>(frame_width);
            const float h = static_cast<float>(frame_height);
            p.roi_box = Rect2f{0.2f * w, 0.05f * h, 0.6f * w, 0.45f * h};
            p.weight = 0.5f;
            p.source = 2; // 2: Nav Topology
            new_prompts.push_back(p);
        }

        // 5) Fallback: 全图兜底，保证不丢检
        PromptRegion fallback;
        fallback.roi_box = Rect2f{0.0f, 0.0f,
                                  static_cast<float>(frame_width),
                                  static_cast<float>(frame_height)};
        fallback.weight = 0.1f;
        fallback.source = 3;
        new_prompts.push_back(fallback);

        context->prompts = std::move(new_prompts);
        return true;
    }

private:
    int GetFrameWidth(const PipelineContext& context) const {
        if (!context.camera_frames.empty()) {
            const int width = context.camera_frames.front().image.cols;
            if (width > 0) {
                return width;
            }
        }
        if (context.image_tele.cols > 0) {
            return context.image_tele.cols;
        }
        if (context.image_wide.cols > 0) {
            return context.image_wide.cols;
        }
        return 1920;
    }

    int GetFrameHeight(const PipelineContext& context) const {
        if (!context.camera_frames.empty()) {
            const int height = context.camera_frames.front().image.rows;
            if (height > 0) {
                return height;
            }
        }
        if (context.image_tele.rows > 0) {
            return context.image_tele.rows;
        }
        if (context.image_wide.rows > 0) {
            return context.image_wide.rows;
        }
        return 1080;
    }

    const std::vector<SignalCandidate>& SelectSignalCandidates(
        PipelineContext& context) const {
        if (!context.map_signals.empty()) {
            if (context.runtime_state != nullptr) {
                context.runtime_state->cached_signals = context.map_signals;
                context.runtime_state->last_signals_ts_sec =
                    static_cast<double>(context.timestamp) * 1e-9;
            }
            return context.map_signals;
        }
        if (context.runtime_state != nullptr) {
            return context.runtime_state->cached_signals;
        }
        return context.map_signals;
    }

    void ExpandBox(Rect2f& box, float ratio, int frame_width,
                   int frame_height) const {
        float cx = box.x + box.width / 2.0f;
        float cy = box.y + box.height / 2.0f;
        box.width *= ratio;
        box.height *= ratio;
        box.x = cx - box.width / 2.0f;
        box.y = cy - box.height / 2.0f;

        box.x = std::max(0.0f, box.x);
        box.y = std::max(0.0f, box.y);
        box.width = std::max(0.0f,
                             std::min(box.width,
                                      static_cast<float>(frame_width) - box.x));
        box.height = std::max(0.0f,
                              std::min(box.height,
                                       static_cast<float>(frame_height) - box.y));
    }
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo
