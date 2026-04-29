#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/traffic_light/common/context.h"
#include "modules/perception/traffic_light/interface/stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class PerceptionPipeline {
 public:
  void RegisterStage(const std::shared_ptr<BaseStage>& stage) {
    if (stage != nullptr) {
      stages_.push_back(stage);
    }
  }

  bool InitAll() {
    for (const auto& stage : stages_) {
      if (stage == nullptr) {
        continue;
      }
      if (!stage->Init()) {
        return false;
      }
    }
    return true;
  }

  bool ProcessFrame(PipelineContext* context) {
    if (context == nullptr) {
      return false;
    }

    for (const auto& stage : stages_) {
      if (stage == nullptr) {
        continue;
      }
      if (stage->Process(context)) {
        continue;
      }
      context->AppendDegradeReason(stage->Name() + " failed");
      if (!stage->optional()) {
        return false;
      }
    }
    return true;
  }

 private:
  std::vector<std::shared_ptr<BaseStage>> stages_;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
