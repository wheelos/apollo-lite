#pragma once

#include "modules/perception/traffic_light/interface/stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {

// Step 5: 启发式推理 (Heuristic Inference/Side Path)
// 职责: 当完全致盲（被前车大卡车挡住红绿灯）时，利用周围 Agent 的加速起步意图判断当前状态
// 注: 此模块可以在 Pipeline 中与 Vision(Step 2~4) 并行执行
class HeuristicStage : public BaseStage {
public:
    HeuristicStage() = default;
    ~HeuristicStage() override = default;

    std::string Name() const override { return "HeuristicStage"; }

    bool Init(const StageConfig& config) override { return true; }

    bool Process(PipelineContext* context) override {
        if (!context) return false;
        std::lock_guard<std::mutex> lock(context->rw_mutex);

        HeuristicState state;
        state.inferred_color = LightColor::UNKNOWN;
        state.probability = 0.0f;

        // 如果距离路口较近才启动启发式推断
        if (context->nav_topology.distance_to_intersection < 100.0 && !context->nav_topology.is_in_intersection) {

            // 假设我们有前车或右侧车的状态 (Agent is_starting)
            bool leading_vehicle_starting = false;
            for (const auto& agent : context->surrounding_agents) {
                // 如果发现意图相同的前车起步了
                if (agent.intent == context->nav_topology.ego_lane_intent && agent.is_starting) {
                    leading_vehicle_starting = true;
                    break;
                }
            }

            if (leading_vehicle_starting) {
                state.inferred_color = LightColor::GREEN;
                state.probability = 0.85f; // 起步推断为绿灯的置信度很高
                state.inference_reason = "Leading vehicle with same intent has started moving.";
            }
        }

        context->heuristic_state = state;
        return true;
    }
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo
