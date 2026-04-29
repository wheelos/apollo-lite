#pragma once

#include <string>

#include "modules/perception/traffic_light/common/context.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class BaseStage {
 public:
  explicit BaseStage(bool optional = false) : optional_(optional) {}
  virtual ~BaseStage() = default;

  virtual std::string Name() const = 0;
  virtual bool Init() { return true; }
  virtual bool Process(PipelineContext* context) = 0;

  bool optional() const { return optional_; }

 protected:
  void set_optional(bool optional) { optional_ = optional; }

 private:
  bool optional_ = false;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
