#include "modules/dreamview/backend/simulator/simulation.h"

#include "modules/dreamview/backend/simulator/scenario/parser.h"
#include "modules/dreamview/backend/simulator/simulation_manager.h"

namespace apollo {
namespace dreamview {

Simulation::Simulation(MapService* map_service) : map_service_(map_service) {
  engine_ = std::make_unique<SimulationEngine>(map_service_);
}

Simulation::~Simulation() { Stop(); }

bool Simulation::Init(bool set_start_point, double start_velocity,
                      double start_acceleration, double start_heading) {
  (void)set_start_point;
  (void)start_velocity;
  (void)start_acceleration;
  (void)start_heading;
  if (engine_) {
    engine_->Init();
    scenario_manager_ =
        std::make_unique<ScenarioManager>(engine_->GetNode(), map_service_);
  }
  state_ = SimulationState::kIdle;
  return engine_ != nullptr && scenario_manager_ != nullptr;
}

void Simulation::Start() {
  if (state_ == State::kIdle || state_ == State::kPaused) {
    engine_->Start();
    state_ = State::kRunning;
  }
}

void Simulation::Stop() {
  engine_->Stop();
  state_ = State::kPaused;
}

void Simulation::Restart() {
  Stop();
  // 1. 让 ScenarioManager 重新格式化初始状态
  // 2. 让 Ego 瞬移到 (x, y)
  auto ego = scenario_manager_->GetEgo();
  if (ego) {
    ego->Teleport(x, y, 0.0);  // 默认 Heading 0
    ego->Reset();              // 清除旧的 Planning 轨迹
  }
  // 3. 重置评估器
  engine_->GetEvaluator()->Reset();
  Start();
}

nlohmann::json Simulation::LoadDynamicModels() { return nlohmann::json(); }

bool Simulation::AddDynamicModel(const std::string& dynamic_model_name) {
  return true;
}

bool Simulation::ChangeDynamicModel(const std::string& dynamic_model_name) {
  return true;
}

bool Simulation::DeleteDynamicModel(const std::string& dynamic_model_name) {
  return true;
}

void Simulation::ResetDynamicModel() {}

bool Simulation::PlayScenario(const std::string& scenario_path) {
  if (!engine_ || !scenario_manager_) {
    state_ = SimulationState::kFault;
    return false;
  }

  state_ = SimulationState::kLoading;

  engine_->Stop();
  engine_->ClearEntities();

  if (scenario_manager_->LoadScenario(scenario_path)) {
    engine_->AddEntity(scenario_manager_->GetEgo());
    for (const auto& entity : scenario_manager_->GetEntities()) {
      engine_->AddEntity(entity);
    }

    auto evaluation_manager = std::make_shared<EvaluationManager>();
    evaluation_manager->Init();
    engine_->AddEntity(evaluation_manager);

    engine_->Play();
    state_ = SimulationState::kRunning;
    return true;
  } else {
    state_ = SimulationState::kFault;
    return false;
  }
}

bool Simulation::PlayScenarioData(const nlohmann::json& scenario_data) {
  if (!engine_ || !scenario_manager_) {
    state_ = SimulationState::kFault;
    return false;
  }

  Scenario scenario;
  if (!ScenarioParser::FromJson(scenario_data, &scenario)) {
    state_ = SimulationState::kFault;
    return false;
  }

  state_ = SimulationState::kLoading;
  engine_->Stop();
  engine_->ClearEntities();

  if (!scenario_manager_->LoadScenarioData(scenario)) {
    state_ = SimulationState::kFault;
    return false;
  }

  engine_->AddEntity(scenario_manager_->GetEgo());
  for (const auto& entity : scenario_manager_->GetEntities()) {
    engine_->AddEntity(entity);
  }

  auto evaluation_manager = std::make_shared<EvaluationManager>();
  evaluation_manager->Init();
  engine_->AddEntity(evaluation_manager);

  engine_->Play();
  state_ = SimulationState::kRunning;
  return true;
}

void Simulation::ResetScenario() {
  if (engine_) {
    engine_->Reset();
  }
}

uint64_t Simulation::GetTickCount() const {
  if (!engine_) {
    return 0;
  }
  return engine_->GetTickCount();
}

double Simulation::GetSimTimeSec() const {
  if (!engine_) {
    return 0.0;
  }
  return engine_->GetSimTimeSec();
}

bool Simulation::IsRunning() const {
  if (!engine_) {
    return false;
  }
  return engine_->IsRunning();
}

}  // namespace dreamview
}  // namespace apollo
