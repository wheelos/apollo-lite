#include "modules/dreamview/backend/simulator/entity/env/obstacle_manager.h"

#include <cmath>

#include "modules/common/util/message_util.h"

namespace apollo {
namespace dreamview {

ObstacleManager::ObstacleManager(std::shared_ptr<cyber::Node> node,
                                 MapService* map_service)
    : node_(node), map_service_(map_service) {
  perception_writer_ =
      node_->CreateWriter<apollo::perception::PerceptionObstacles>(
          "/apollo/perception/obstacles");
}

void ObstacleManager::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  static_obstacles_.clear();
  dynamic_obstacles_.clear();
  // Do not reset next_obstacle_id_ to avoid ID churn across resets.
  start_time_ = 0.0;
}

void ObstacleManager::LoadObstacles(const nlohmann::json& scenario_json) {
  std::lock_guard<std::mutex> lock(mutex_);
  static_obstacles_.clear();
  dynamic_obstacles_.clear();
  start_time_ = 0.0;

  auto convert_to_double = [](const nlohmann::json& val) -> double {
    if (val.is_number()) return val.get<double>();
    if (val.is_string()) return std::stod(val.get<std::string>());
    return 0.0;
  };

  if (scenario_json.contains("s")) {
    const auto& s_array = scenario_json["s"];
    for (const auto& item : s_array) {
      if (!item.contains("p") || item["p"].size() < 2) continue;
      SimStaticObstacle obs;
      obs.id = next_obstacle_id_++;
      obs.x = convert_to_double(item["p"][0]);
      obs.y = convert_to_double(item["p"][1]);
      obs.heading = item.contains("r") ? convert_to_double(item["r"]) : 0.0;
      obs.width = item.contains("w") ? convert_to_double(item["w"]) : 2.0;
      obs.length = item.contains("h") ? convert_to_double(item["h"]) : 5.0;
      static_obstacles_.push_back(obs);
    }
  }

  if (scenario_json.contains("d")) {
    const auto& d_array = scenario_json["d"];
    for (const auto& item : d_array) {
      if (!item.contains("p") || item["p"].size() < 2) continue;
      if (!item.contains("v") || item["v"].size() < 2) continue;

      SimDynamicObstacle obs;
      // Respect provided id if present to keep IDs stable for downstream
      // modules
      if (item.contains("id")) {
        obs.id = static_cast<int>(convert_to_double(item["id"]));
      } else {
        obs.id = next_obstacle_id_++;
      }
      // Expect dynamic obstacle positions in Cartesian (x,y).
      obs.x = convert_to_double(item["p"][0]);
      obs.y = convert_to_double(item["p"][1]);
      obs.vx = convert_to_double(item["v"][0]);
      obs.vy = convert_to_double(item["v"][1]);
      obs.width = item.contains("w") ? convert_to_double(item["w"]) : 2.0;
      obs.length = item.contains("h") ? convert_to_double(item["h"]) : 5.0;
      obs.trigger_time =
          item.contains("t") ? convert_to_double(item["t"]) : 0.0;
      obs.is_active = false;

      dynamic_obstacles_.push_back(obs);
    }
  }
}

void ObstacleManager::LoadFromScenario(
    const apollo::dreamview::Scenario& scenario) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Do not reset next_obstacle_id_ to preserve ID stability
  static_obstacles_.clear();
  dynamic_obstacles_.clear();

  for (const auto& s : scenario.static_obstacles) {
    SimStaticObstacle obs;
    obs.id = (s.id != 0) ? s.id : next_obstacle_id_++;
    obs.x = s.x;
    obs.y = s.y;
    obs.heading = s.heading;
    obs.width = s.width;
    obs.length = s.length;
    static_obstacles_.push_back(obs);
  }

  for (const auto& d : scenario.dynamic_obstacles) {
    SimDynamicObstacle obs;
    obs.id = (d.id != 0) ? d.id : next_obstacle_id_++;
    obs.x = d.x;
    obs.y = d.y;
    obs.vx = d.vx;
    obs.vy = d.vy;
    obs.width = d.width;
    obs.length = d.length;
    obs.trigger_time = d.trigger_time;
    obs.is_active = false;
    dynamic_obstacles_.push_back(obs);
  }
}

bool ObstacleManager::Init() { return true; }

void ObstacleManager::Step(double dt) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (dt < 0.0) {
    dt = 0.0;
  }
  start_time_ += dt;
  double elapsed_time = start_time_;

  for (auto& dyn_obs : dynamic_obstacles_) {
    if (!dyn_obs.is_active && elapsed_time >= dyn_obs.trigger_time) {
      dyn_obs.is_active = true;
    }
    if (dyn_obs.is_active) {
      // Update Cartesian position
      dyn_obs.x += dyn_obs.vx * dt;
      dyn_obs.y += dyn_obs.vy * dt;
    }
  }
}

void ObstacleManager::Publish() {
  if (static_obstacles_.empty() && dynamic_obstacles_.empty()) {
    return;
  }

  auto perception_obstacles =
      std::make_shared<apollo::perception::PerceptionObstacles>();
  apollo::common::util::FillHeader("ObstacleManager",
                                   perception_obstacles.get());
  perception_obstacles->mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());

  // Add Static
  for (const auto& obs : static_obstacles_) {
    auto* perception_obstacle = perception_obstacles->add_perception_obstacle();
    perception_obstacle->set_id(obs.id);
    perception_obstacle->mutable_position()->set_x(obs.x);
    perception_obstacle->mutable_position()->set_y(obs.y);
    perception_obstacle->set_theta(obs.heading);
    perception_obstacle->set_length(obs.length);
    perception_obstacle->set_width(obs.width);
    perception_obstacle->set_height(2.0);
    perception_obstacle->set_type(
        apollo::perception::PerceptionObstacle::VEHICLE);
    perception_obstacle->mutable_velocity()->set_x(0);
    perception_obstacle->mutable_velocity()->set_y(0);
  }

  // Add Dynamic (Frenet -> XY using MapService)
  for (const auto& obs : dynamic_obstacles_) {
    if (!obs.is_active) continue;
    // Dynamic obstacles are stored in Cartesian; publish directly.
    auto* p_obs = perception_obstacles->add_perception_obstacle();
    p_obs->set_id(obs.id);
    p_obs->mutable_position()->set_x(obs.x);
    p_obs->mutable_position()->set_y(obs.y);
    // Heading: infer from velocity vector if available
    double theta = 0.0;
    if (std::abs(obs.vx) > 1e-6 || std::abs(obs.vy) > 1e-6) {
      theta = std::atan2(obs.vy, obs.vx);
    }
    p_obs->set_theta(theta);
    p_obs->set_length(obs.length);
    p_obs->set_width(obs.width);
    p_obs->set_height(2.0);
    p_obs->set_type(apollo::perception::PerceptionObstacle::VEHICLE);
    p_obs->mutable_velocity()->set_x(obs.vx);
    p_obs->mutable_velocity()->set_y(obs.vy);
  }

  perception_writer_->Write(perception_obstacles);
}

}  // namespace dreamview
}  // namespace apollo
