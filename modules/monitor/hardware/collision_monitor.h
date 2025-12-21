#pragma once

#include "modules/monitor/common/recurrent_runner.h"

namespace apollo {
namespace monitor {

class CollisionMonitor : public RecurrentRunner {
 public:
  CollisionMonitor();
  void RunOnce(const double current_time) override;
};

}  // namespace monitor
}  // namespace apollo
