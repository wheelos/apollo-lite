// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2026-09-10
//  Author: daohu527

#pragma once

#include <memory>

#include "cyber/component/timer_component.h"
#include "modules/simulation/adapter/cyber_adapter.h"
#include "modules/simulation/common/simulation_gflags.h"
#include "modules/simulation/core/simulation_engine.h"

namespace apollo {
namespace simulation {

/**
 * @class SimulationComponent
 * @brief Main Apollo Cyber timer component for vehicle control simulation.
 */
class SimulationComponent final : public apollo::cyber::TimerComponent {
 public:
  SimulationComponent() = default;
  ~SimulationComponent() override = default;

  bool Init() override;
  bool Proc() override;

 private:
  std::unique_ptr<CyberAdapter> adapter_;
  std::unique_ptr<SimulationEngine> engine_;

  uint64_t proc_count_{0};
};

CYBER_REGISTER_COMPONENT(SimulationComponent)

}  // namespace simulation
}  // namespace apollo
