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

#include "modules/simulation/simulation_component.h"

#include <cmath>

#include "cyber/common/log.h"

namespace apollo {
namespace simulation {

bool SimulationComponent::Init() {
  AINFO << "Initializing SimulationComponent ...";

  adapter_ = std::make_unique<CyberAdapter>();
  if (!adapter_->Init(node_)) {
    AERROR << "Failed to initialize CyberAdapter!";
    return false;
  }

  engine_ = std::make_unique<SimulationEngine>();
  if (!engine_->SetPhysicsDt(FLAGS_sim_physics_dt)) {
    AERROR << "Invalid simulation physics dt: " << FLAGS_sim_physics_dt;
    return false;
  }
  if (!std::isfinite(FLAGS_sim_control_dt) || FLAGS_sim_control_dt <= 0.0) {
    AERROR << "Invalid simulation control dt: " << FLAGS_sim_control_dt;
    return false;
  }
  if (!engine_->SetCommandTimeout(FLAGS_sim_command_timeout)) {
    AERROR << "Invalid simulation command timeout: "
           << FLAGS_sim_command_timeout;
    return false;
  }
  engine_->SetControlMode(FLAGS_sim_control_mode);
  engine_->SetSpeedKp(FLAGS_sim_speed_kp);

  if (!engine_->Init(FLAGS_sim_backend_type, FLAGS_sim_model_path)) {
    AERROR << "Failed to initialize SimulationEngine with backend: "
           << FLAGS_sim_backend_type;
    return false;
  }

  engine_->Reset(FLAGS_sim_init_x, FLAGS_sim_init_y, FLAGS_sim_init_yaw);
  AINFO << "SimulationComponent initialized successfully. Backend: "
        << FLAGS_sim_backend_type << " at (" << FLAGS_sim_init_x << ", "
        << FLAGS_sim_init_y << ", yaw=" << FLAGS_sim_init_yaw << ")";
  return true;
}

bool SimulationComponent::Proc() {
  // 1. Poll incoming control command from Apollo
  VehicleCommand cmd{};
  const bool command_received = adapter_->PollCommand(&cmd);

  // 2. Step simulation engine with physics substepping
  if (!engine_->Step(cmd, FLAGS_sim_control_dt, command_received)) {
    AERROR << "SimulationEngine failed to advance.";
    return false;
  }

  // 3. Retrieve ground truth vehicle state
  VehicleState state{};
  if (engine_->GetVehicleState(&state)) {
    // 4. Publish Chassis and Localization feedback to Apollo
    adapter_->PublishFeedback(state);
  }

  ++proc_count_;
  return true;
}

}  // namespace simulation
}  // namespace apollo
