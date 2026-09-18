#pragma once

#include "nlohmann/json.hpp"

#include "modules/dreamview/backend/simulator/scenario/scenario.h"

namespace apollo {
namespace dreamview {

class ScenarioParser {
 public:
  // Parse JSON into Scenario. Returns true on success.
  static bool FromJson(const nlohmann::json& j, Scenario* out);
};

}  // namespace dreamview
}  // namespace apollo
