#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/backend/simulator/base/entity.h"
#include "modules/dreamview/backend/simulator/entity/ego/ego_model_base.h"
#include "modules/dreamview/backend/simulator/scenario/scenario.h"

namespace apollo {
namespace dreamview {

class ScenarioManager {
 public:
  ScenarioManager(std::shared_ptr<cyber::Node> node, MapService* map_service)
      : node_(std::move(node)), map_service_(map_service) {}
  ~ScenarioManager() = default;

  bool LoadScenario(const std::string& scenario_file);
  bool LoadScenarioData(const Scenario& scenario);

  std::shared_ptr<EgoModelBase> GetEgo() const { return ego_; }

  const std::vector<std::shared_ptr<SimEntity>>& GetEntities() const {
    return entities_;
  }

 private:
  std::shared_ptr<cyber::Node> node_;
  MapService* map_service_ = nullptr;
  std::shared_ptr<EgoModelBase> ego_;
  std::vector<std::shared_ptr<SimEntity>> entities_;
};

}  // namespace dreamview
}  // namespace apollo
