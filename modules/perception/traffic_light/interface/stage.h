#pragma once

#include <string>

#include "modules/perception/traffic_light/common/context.h"

namespace apollo {
namespace perception {
namespace traffic_light {

struct StageConfig {
  std::string name;
  bool optional = false;
  // std::string model_path;
  // float threshold等可以通过 proto 注入，这里用占位符
};

// 认知步骤抽象基类，所有流水线的算法节点都要继承它
class BaseStage {
 public:
  virtual ~BaseStage() = default;

  virtual std::string Name() const = 0;

  // 可以在此拉起 TensorRT engine，加载 config 等
  virtual bool Init(const StageConfig& config) = 0;

  // 核心处理逻辑，通过 context 黑板取自己需要的数据，并将产出写回 context
  virtual bool Process(PipelineContext* context) = 0;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
