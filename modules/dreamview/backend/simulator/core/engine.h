#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/backend/simulator/base/entity.h"
#include "modules/dreamview/backend/simulator/base/frame_context.h"
#include "modules/dreamview/backend/simulator/entity/ego/ego_model.h"
#include "modules/dreamview/backend/simulator/entity/env/obstacle_manager.h"
#include "modules/dreamview/backend/simulator/evaluation/evaluator.h"

namespace apollo {
namespace dreamview {

class SimulationEngine {
 public:
  SimulationEngine(MapService* map_service);
  ~SimulationEngine();

  bool Init();

  std::shared_ptr<cyber::Node> GetNode() const { return node_; }
  void AddEntity(std::shared_ptr<SimEntity> entity);

  // Load a parsed Scenario (preferred) rather than depending on JSON in-engine.
  // We keep this or let SimulationManager build and inject the environment
  // objects. Actually, wait, since SimulationManager builds everything, we just
  // AddEntity. But maybe let's keep LoadScenario for backward compatibility or
  // refactor gracefully. Actually let's remove LoadScenario and do it all via
  // SimManager.
  void ClearEntities();

  void Tick();

  uint64_t GetTickCount() const;
  double GetSimTimeSec() const;

 private:
  void Tick();

  std::shared_ptr<cyber::Node> node_;
  std::unique_ptr<cyber::Timer> tick_timer_;
  MapService* map_service_;

  std::vector<std::shared_ptr<SimEntity>> entities_;

  double last_tick_time_ = 0.0;
  double sim_time_sec_ = 0.0;
  uint64_t tick_count_ = 0;
  bool is_running_ = false;
  mutable std::mutex mutex_;

  // Caches used to hold temporary snapshots passed to Evaluator
  FrameContext frame_context_cache_;
  EgoState ego_state_cache_;
  std::vector<ObstacleStatus> obstacles_cache_;
};

}  // namespace dreamview
}  // namespace apollo
