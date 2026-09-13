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

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "wheelos_msgs/config_msgs/vehicle_config.pb.h"

namespace apollo {
namespace simulation {
namespace {

constexpr char kDefaultVehicleConfigPath[] =
    "/apollo/modules/common/data/vehicle_param.pb.txt";
constexpr char kWorkspaceVehicleConfigPath[] =
    "modules/common/data/vehicle_param.pb.txt";

bool LoadVehicleConfig(common::VehicleConfig* config) {
  std::vector<std::string> candidate_paths{FLAGS_vehicle_config_path};
  if (FLAGS_vehicle_config_path == kDefaultVehicleConfigPath) {
    candidate_paths.emplace_back(kWorkspaceVehicleConfigPath);
    const char* test_srcdir = std::getenv("TEST_SRCDIR");
    const char* test_workspace = std::getenv("TEST_WORKSPACE");
    if (test_srcdir != nullptr && test_workspace != nullptr) {
      candidate_paths.emplace_back(std::string(test_srcdir) + "/" +
                                   test_workspace + "/" +
                                   kWorkspaceVehicleConfigPath);
    }
  }
  for (const auto& path : candidate_paths) {
    if (cyber::common::PathExists(path) &&
        cyber::common::GetProtoFromFile(path, config)) {
      AINFO << "Loaded simulation vehicle configuration from " << path;
      return true;
    }
  }
  AERROR << "Unable to load simulation vehicle configuration from "
         << FLAGS_vehicle_config_path;
  return false;
}

}  // namespace

bool SimulationComponent::Init() {
  AINFO << "Initializing SimulationComponent ...";

  adapter_ = std::make_unique<CyberAdapter>();
  common::VehicleConfig vehicle_config;
  if (!LoadVehicleConfig(&vehicle_config) ||
      !vehicle_config.has_vehicle_param()) {
    return false;
  }
  const auto& vehicle_param = vehicle_config.vehicle_param();
  if ((!vehicle_param.has_max_front_wheel_steer() &&
       (!std::isfinite(vehicle_param.max_steer_angle()) ||
        !std::isfinite(vehicle_param.steer_ratio()) ||
        vehicle_param.steer_ratio() <= 0.0)) ||
      !vehicle_param.has_track_width()) {
    AERROR << "Vehicle configuration is missing steering or track width.";
    return false;
  }
  const double max_steer_angle_rad = vehicle_param.has_max_front_wheel_steer()
                                         ? vehicle_param.max_front_wheel_steer()
                                         : vehicle_param.max_steer_angle() /
                                               vehicle_param.steer_ratio();
  const double max_rear_steer_angle_rad =
      vehicle_param.has_max_back_wheel_steer()
          ? vehicle_param.max_back_wheel_steer()
          : 0.0;
  const double track_width_m = vehicle_param.track_width();
  const double wheelbase_m = vehicle_param.wheel_base();
  const double wheel_radius_m = vehicle_param.wheel_rolling_radius();
  max_steer_angle_rad_ = max_steer_angle_rad;
  if (!std::isfinite(max_steer_angle_rad) || max_steer_angle_rad <= 0.0) {
    AERROR << "Invalid vehicle max_front_wheel_steer: "
           << max_steer_angle_rad;
    return false;
  }
  if (!std::isfinite(max_rear_steer_angle_rad) ||
      max_rear_steer_angle_rad < 0.0 ||
      max_rear_steer_angle_rad > max_steer_angle_rad) {
    AERROR << "Invalid vehicle max_back_wheel_steer: "
           << max_rear_steer_angle_rad;
    return false;
  }
  if (!std::isfinite(track_width_m) || track_width_m <= 0.0 ||
      !std::isfinite(wheelbase_m) || wheelbase_m <= 0.0 ||
      !std::isfinite(wheel_radius_m) || wheel_radius_m <= 0.0) {
    AERROR << "Invalid vehicle geometry: wheelbase=" << wheelbase_m
           << ", track_width=" << track_width_m
           << ", wheel_radius=" << wheel_radius_m;
    return false;
  }
  adapter_->SetMaxSteerAngle(max_steer_angle_rad);
  if (!adapter_->Init(node_)) {
    AERROR << "Failed to initialize CyberAdapter!";
    return false;
  }

  engine_ = std::make_unique<SimulationEngine>();
  engine_->SetMaxSteerAngle(max_steer_angle_rad);
  engine_->SetMaxRearSteerAngle(max_rear_steer_angle_rad);
  engine_->SetVehicleGeometry(wheelbase_m, track_width_m, wheel_radius_m);
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
    state.steering_percentage = std::clamp(
        state.front_steering_rad / max_steer_angle_rad_ * 100.0, -100.0,
        100.0);
    // 4. Publish Chassis and Localization feedback to Apollo
    adapter_->PublishFeedback(state);
  }

  ++proc_count_;
  return true;
}

}  // namespace simulation
}  // namespace apollo
