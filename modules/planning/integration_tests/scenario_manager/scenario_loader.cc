#include "modules/planning/integration_tests/scenario_manager/scenario_loader.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {
namespace scenario {

namespace {

ScenarioCase ParseScenarioCase(const YAML::Node& node) {
  ScenarioCase tc;
  tc.id = node["id"].as<std::string>();
  tc.description = node["description"].as<std::string>();

  const auto input = node["input"];
  if (const auto vehicle_state = input["vehicle_state"]) {
    tc.input.timestamp = vehicle_state["timestamp"].as<double>(tc.input.timestamp);
    tc.input.speed = vehicle_state["speed"].as<double>(tc.input.speed);
  }
  tc.input.timestamp = input["timestamp"].as<double>(tc.input.timestamp);
  tc.input.speed = input["speed"].as<double>(tc.input.speed);
  tc.input.pad_message = input["pad_message"].as<std::string>(tc.input.pad_message);
  tc.input.internal_emergency =
      input["internal_emergency"].as<bool>(tc.input.internal_emergency);
  tc.input.history_stuck_time =
      input["history_stuck_time"].as<double>(tc.input.history_stuck_time);
  tc.input.has_routing = input["has_routing"].as<bool>(tc.input.has_routing);
  tc.input.current_scenario =
      input["current_scenario"].as<std::string>(tc.input.current_scenario);
  tc.input.current_status =
      input["current_status"].as<std::string>(tc.input.current_status);

  const auto expected = node["expected"];
  tc.expected.scenario = expected["scenario"].as<std::string>();
  tc.expected.grade = expected["grade"].as<std::string>(tc.expected.grade);

  return tc;
}

void AppendScenarioCases(const YAML::Node& cases_node,
                         std::vector<ScenarioCase>* cases) {
  for (const auto& node : cases_node) {
    cases->push_back(ParseScenarioCase(node));
  }
}

}  // namespace

std::vector<ScenarioCase> ScenarioLoader::Load(const std::string& file_path) {
  std::vector<ScenarioCase> cases;
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    if (config["test_cases"]) {
      AppendScenarioCases(config["test_cases"], &cases);
      return cases;
    }

    if (config["test_suites"]) {
      for (const auto& suite : config["test_suites"]) {
        if (!suite["cases"]) {
          continue;
        }
        AppendScenarioCases(suite["cases"], &cases);
      }
      return cases;
    }

    AERROR << "Failed to load scenario test cases from " << file_path
           << ": missing test_cases or test_suites";
  } catch (const std::exception& e) {
    AERROR << "Failed to load scenario test cases from " << file_path << ": "
           << e.what();
  }
  return cases;
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
