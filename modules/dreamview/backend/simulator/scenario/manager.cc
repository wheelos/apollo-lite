#include "modules/dreamview/backend/simulator/scenario/manager.h"

#include <cmath>
#include <fstream>

#include "nlohmann/json.hpp"

#include "modules/dreamview/backend/simulator/entity/ego/ego_model.h"
#include "modules/dreamview/backend/simulator/entity/ego/ego_perfect_model.h"
#include "modules/dreamview/backend/simulator/entity/env/obstacle_manager.h"
#include "modules/dreamview/backend/simulator/entity/env/traffic_light_manager.h"
#include "modules/dreamview/backend/simulator/scenario/parser.h"

namespace apollo {
namespace dreamview {

bool ScenarioManager::LoadScenario(const std::string& scenario_file) {
  std::ifstream ifs(scenario_file);
  if (!ifs.is_open()) {
    AERROR << "Failed to open scenario file: " << scenario_file;
    return false;
  }

  nlohmann::json scenario_json;
  try {
    ifs >> scenario_json;
  } catch (const nlohmann::json::parse_error& e) {
    AERROR << "Failed to parse scenario file: " << scenario_file
           << ", reason: " << e.what();
    return false;
  }

  Scenario scenario;
  if (!ScenarioParser::FromJson(scenario_json, &scenario)) {
    AERROR << "Failed to convert scenario JSON: " << scenario_file;
    return false;
  }

  return LoadScenarioData(scenario);
}

bool ScenarioManager::LoadScenarioData(const Scenario& scenario) {
  if (!node_ || map_service_ == nullptr) {
    AERROR << "ScenarioManager is not initialized with node/map service";
    return false;
  }

  // 1. Instantiate Ego
  auto ego_model = std::make_shared<EgoPerfectModel>(node_, map_service_);
  if (!ego_model->Init()) {
    AERROR << "Failed to init ego model";
    return false;
  }

  if (!scenario.ego_waypoints.empty()) {
    if (scenario.ego_waypoints.size() >= 2) {
      const auto& start = scenario.ego_waypoints[0];
      const auto& next = scenario.ego_waypoints[1];
      double heading = std::atan2(next.y() - start.y(), next.x() - start.x());
      ego_model->SetPose(start.x(), start.y(), heading, 0.0);
    } else {
      const auto& start = scenario.ego_waypoints[0];
      ego_model->SetPose(start.x(), start.y(), 0.0, 0.0);
    }
    ego_model->RequestRouting(scenario.ego_waypoints);
  }

  ego_ = ego_model;

  // 2. Instantiate ObstacleManager and TrafficLightManager as entities
  auto obstacle_manager =
      std::make_shared<ObstacleManager>(node_, map_service_);
  if (!obstacle_manager->Init()) {
    AERROR << "Failed to init obstacle manager";
    return false;
  }
  obstacle_manager->LoadFromScenario(scenario);

  auto traffic_light_manager = std::make_shared<TrafficLightManager>(node_);
  if (!traffic_light_manager->Init()) {
    AERROR << "Failed to init traffic light manager";
    return false;
  }
  traffic_light_manager->LoadFromScenario(scenario);

  entities_.clear();
  entities_.push_back(obstacle_manager);
  entities_.push_back(traffic_light_manager);

  return true;
}

bool ScenarioManager::LoadScenarioData(const Scenario& scenario) {
  // 1. 清理旧数据
  entities_.clear();
  ego_ = nullptr;

  // 2. 实例化 Ego (工厂模式)
  if (scenario.ego_config.type == "PERFECT_CONTROL") {
    ego_ = std::make_shared<EgoPerfectModel>(node_, map_service_);
  } else {
    // AERROR << "Unsupported ego type: " << scenario.ego_config.type;
    return false;
  }

  // 初始化 Ego 状态
  ego_->Init();
  ego_->Teleport(scenario.ego_config.x, scenario.ego_config.y,
                 scenario.ego_config.heading, scenario.ego_config.v);

  // 3. 实例化并配置障碍物
  auto obstacle_manager =
      std::make_shared<ObstacleManager>(node_, map_service_);
  obstacle_manager->Init();
  obstacle_manager->LoadFromScenario(scenario);  // 内部解析 scenario.obstacles

  // 4. 汇总到实体列表
  entities_.push_back(ego_);
  entities_.push_back(obstacle_manager);

  // 5. 实例化红绿灯、评估器等并加入 entities_
  // ...

  return true;
}

}  // namespace dreamview
}  // namespace apollo
