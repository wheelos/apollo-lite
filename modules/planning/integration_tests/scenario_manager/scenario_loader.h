#pragma once

#include "yaml-cpp/yaml.h"

#include "modules/planning/integration_tests/scenario_manager/scenario_case.h"

namespace apollo {
namespace planning {
namespace scenario {

class ScenarioLoader {
 public:
  static std::vector<ScenarioCase> Load(const std::string& file_path);
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
