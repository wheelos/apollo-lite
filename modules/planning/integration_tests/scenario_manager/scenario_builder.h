#pragma once

#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/integration_tests/scenario_manager/scenario_case.h"

namespace apollo {
namespace planning {
namespace scenario {

class ScenarioBuilder {
 public:
  void Build(const EnvInput& input, DependencyInjector* injector,
             LocalView* local_view, common::VehicleState* vehicle_state);
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
