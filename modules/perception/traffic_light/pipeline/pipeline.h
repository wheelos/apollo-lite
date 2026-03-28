#pragma once

#include <vector>
#include <memory>
#include "modules/perception/traffic_light/interface/stage.h"
#include "modules/perception/traffic_light/common/context.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class PerceptionPipeline {
public:
    PerceptionPipeline() = default;
    ~PerceptionPipeline() = default;

    // 注册节点。目前默认按注册顺序串行执行
    void RegisterStage(std::shared_ptr<BaseStage> stage) {
        if (stage != nullptr) {
            stages_.push_back(stage);
        }
    }

    bool InitAll() {
        for (auto& stage : stages_) {
            StageConfig cfg;
            cfg.name = stage->Name() + "_config";
            if (!stage->Init(cfg)) {
                return false;
            }
        }
        return true;
    }

    // 运行全管道
    bool ProcessFrame(PipelineContext* context) {
        if (!context) return false;

        for (auto& stage : stages_) {
            // 在这一层可以打点 time，记录每个 stage 耗时
            if (!stage->Process(context)) {
                // 如果是核心节点挂了（如 Detector）可以选择中止
                // 如果是旁路节点（如 Heuristic）可以忽略并 continue
            }
        }
        return true;
    }

private:
    std::vector<std::shared_ptr<BaseStage>> stages_;
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo
