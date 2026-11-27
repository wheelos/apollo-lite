#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "nlohmann/json.hpp"

#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/backend/simulator/core/engine.h"
#include "modules/dreamview/backend/simulator/scenario/manager.h"

namespace apollo {
namespace dreamview {

class Simulation {
 public:
  explicit Simulation(MapService* map_service = nullptr);
  virtual ~Simulation();

  bool Init(bool set_start_point, double start_velocity = 0.0,
            double start_acceleration = 0.0,
            double start_heading = std::numeric_limits<double>::max());

  void Start();
  void Stop();
  void Restart();

  enum class SimulationState { kIdle, kLoading, kRunning, kPaused, kFault };

  bool IsEnabled() const { return state_ == SimulationState::kRunning; }
  SimulationState GetState() const { return state_; }

  // Scenario operations
  bool PlayScenario(const std::string& scenario_path);
  bool PlayScenario(const nlohmann::json& scenario_data);
  void ResetScenario();

  uint64_t GetTickCount() const;
  double GetSimTimeSec() const;
  bool IsRunning() const;

 private:
  std::unique_ptr<SimulationEngine> engine_;
  std::unique_ptr<ScenarioManager> scenario_manager_;
  MapService* map_service_;
  SimulationState state_ = SimulationState::kIdle;
};

}  // namespace dreamview
}  // namespace apollo
