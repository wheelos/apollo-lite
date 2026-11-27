#include "modules/dreamview/backend/simulator/core/engine.h"

#include <cmath>

namespace apollo {
namespace dreamview {

SimulationEngine::SimulationEngine(MapService* map_service)
    : node_(cyber::CreateNode("simulation_engine")),
      map_service_(map_service) {}

SimulationEngine::~SimulationEngine() { Stop(); }

bool SimulationEngine::Init() {
  tick_timer_.reset(new cyber::Timer(10, [this]() { this->Tick(); }, false));
  return true;
}

void SimulationEngine::AddEntity(std::shared_ptr<SimEntity> entity) {
  std::lock_guard<std::mutex> lock(mutex_);
  entities_.push_back(std::move(entity));
}

void SimulationEngine::ClearEntities() {
  std::lock_guard<std::mutex> lock(mutex_);
  entities_.clear();
}

void SimulationEngine::Play() {
  std::lock_guard<std::mutex> lock(mutex_);
  last_tick_time_ = cyber::Time::Now().ToSecond();
  sim_time_sec_ = 0.0;
  tick_count_ = 0;
  is_running_ = true;
  if (tick_timer_) {
    tick_timer_->Start();
  }
}

void SimulationEngine::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  is_running_ = false;
  if (tick_timer_) {
    tick_timer_->Stop();
  }
}

void SimulationEngine::Reset() {
  Stop();
  std::lock_guard<std::mutex> lock(mutex_);
  sim_time_sec_ = 0.0;
  tick_count_ = 0;
  for (auto& entity : entities_) {
    entity->Reset();
  }
}

bool SimulationEngine::IsRunning() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return is_running_;
}

uint64_t SimulationEngine::GetTickCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return tick_count_;
}

double SimulationEngine::GetSimTimeSec() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return sim_time_sec_;
}

void SimulationEngine::Tick() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_running_) return;

  double current_time = cyber::Time::Now().ToSecond();
  double dt = current_time - last_tick_time_;
  last_tick_time_ = current_time;
  if (dt < 0.0) {
    dt = 0.0;
  }
  sim_time_sec_ += dt;
  ++tick_count_;

  // 1. Update calculation (Step)
  for (auto& entity : entities_) {
    entity->Step(dt);
  }

  // 2. Evaluation: build FrameContext from entity snapshots and run evaluators
  frame_context_cache_.dt = dt;
  frame_context_cache_.sim_time_sec = sim_time_sec_;
  frame_context_cache_.frame_index = tick_count_;

  // Reset caches
  ego_state_cache_ = EgoState();
  obstacles_cache_.clear();
  frame_context_cache_.ego_state = nullptr;
  frame_context_cache_.obstacles = nullptr;

  for (const auto& entity : entities_) {
    // Try ego
    if (!frame_context_cache_.ego_state) {
      auto ego = std::dynamic_pointer_cast<EgoModel>(entity);
      if (ego) {
        ego_state_cache_ = ego->GetState();
        frame_context_cache_.ego_state = &ego_state_cache_;
      }
    }
    // Try obstacle manager
    auto obs = std::dynamic_pointer_cast<ObstacleManager>(entity);
    if (obs) {
      obstacles_cache_ = obs->GetObstacleStatuses();
      frame_context_cache_.obstacles = &obstacles_cache_;
    }
  }

  // Invoke Evaluate on evaluator entities
  for (const auto& entity : entities_) {
    auto evaluator = std::dynamic_pointer_cast<Evaluator>(entity);
    if (evaluator) {
      evaluator->Evaluate(frame_context_cache_);
    }
  }

  // 3. Data output (Publish)
  for (auto& entity : entities_) {
    entity->Publish();
  }
}

}  // namespace dreamview
}  // namespace apollo
