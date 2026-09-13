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

#include "modules/simulation/core/simulation_engine.h"

#include <algorithm>
#include <cmath>

#include "cyber/common/log.h"
#include "modules/simulation/backend/kinematic_backend.h"
#include "modules/simulation/backend/mujoco_backend.h"
#include "modules/simulation/common/simulation_gflags.h"

namespace apollo {
namespace simulation {

SimulationEngine::SimulationEngine() {
  backend_ = std::make_unique<KinematicBackend>();
}

bool SimulationEngine::Init(const std::string& backend_type,
                            const std::string& model_path) {
  vehicle_model_.SetMaxSteerAngle(max_steer_angle_rad_);
  vehicle_model_.SetMaxRearSteerAngle(max_rear_steer_angle_rad_);
  vehicle_model_.SetGeometry(wheelbase_m_, track_width_m_, wheel_radius_m_);
  if (backend_type == "mujoco") {
    backend_ = std::make_unique<MujocoBackend>();
    AINFO << "SimulationEngine configured with MuJoCo backend.";
  } else if (backend_type == "kinematic") {
    backend_ = std::make_unique<KinematicBackend>();
    AINFO << "SimulationEngine configured with Kinematic bicycle backend.";
  } else {
    AERROR << "Unknown simulation backend type: " << backend_type;
    return false;
  }

  if (!backend_->Init(model_path)) {
    AERROR << "Backend initialization failed: " << backend_->Name();
    return false;
  }
  if (!backend_->SetVehicleGeometry(wheelbase_m_, track_width_m_,
                                    wheel_radius_m_) ||
      !backend_->SetMaxSteerAngle(max_steer_angle_rad_)) {
    AERROR << "Vehicle configuration is incompatible with backend "
           << backend_->Name();
    return false;
  }

  Reset(0.0, 0.0, 0.0);
  return true;
}

void SimulationEngine::Reset(double x, double y, double yaw) {
  if (backend_) {
    backend_->Reset(x, y, yaw);
    if (!backend_->GetVehicleState(&current_state_)) {
      AERROR << "Failed to read backend state after reset: "
             << backend_->Name();
    }
  }
  last_cmd_time_sec_ = 0.0;
  has_received_command_ = false;
  step_count_ = 0;
  odometer_m_ = 0.0;
  last_position_x_ = current_state_.x;
  last_position_y_ = current_state_.y;
  current_state_.odometer_m = 0.0;
}

bool SimulationEngine::Step(const VehicleCommand& cmd, double control_dt_sec,
                            bool command_received) {
  if (!backend_ || !std::isfinite(control_dt_sec) || control_dt_sec <= 0.0) {
    return false;
  }

  VehicleCommand active_cmd = cmd;
  double current_time = backend_->SimulationTime();

  if (command_received) {
    has_received_command_ = true;
    last_cmd_time_sec_ = current_time;
  }

  // Command timeout check. The receive state is supplied by the adapter; a
  // repeated cached command must not keep the watchdog alive.
  const bool command_timed_out =
      !has_received_command_ ||
      current_time - last_cmd_time_sec_ > command_timeout_sec_;
  if (command_timed_out) {
    AWARN_EVERY(100) << "Control command timed out! Applying failsafe brake.";
    active_cmd.throttle = 0.0;
    active_cmd.brake = 0.5;
    active_cmd.front_steering_rad =
        current_state_.front_steering_rad * 0.95;  // Gradual return
  }

  if (control_mode_ == "speed" && !command_timed_out) {
    active_cmd.throttle = 0.0;
    active_cmd.brake = 0.0;
    active_cmd.target_acceleration_mps2 =
        speed_kp_ *
        (active_cmd.target_speed_mps - current_state_.linear_velocity_mps);
  }

  // Require an integral number of fixed physics steps. This keeps the backend
  // timestep and simulation clock deterministic.
  const double step_ratio = control_dt_sec / physics_dt_sec_;
  const int sub_steps = static_cast<int>(std::round(step_ratio));
  if (sub_steps <= 0 || std::abs(step_ratio - sub_steps) > 1e-9) {
    AERROR
        << "Control dt must be an integral multiple of physics dt: control_dt="
        << control_dt_sec << ", physics_dt=" << physics_dt_sec_;
    return false;
  }

  VehicleActuation last_actuation{};
  for (int step = 0; step < sub_steps; ++step) {
    last_actuation = vehicle_model_.ComputeActuation(active_cmd, current_state_,
                                                     physics_dt_sec_);
    if (!backend_->ApplyActuation(last_actuation)) {
      AERROR << "Backend failed to apply actuation: " << backend_->Name();
      return false;
    }
    if (!backend_->Step(physics_dt_sec_)) {
      AERROR << "Backend failed to advance simulation: " << backend_->Name();
      return false;
    }
    if (!backend_->GetVehicleState(&current_state_)) {
      AERROR << "Backend failed to provide vehicle state: " << backend_->Name();
      return false;
    }
  }

  current_state_.sequence_num = ++step_count_;
  odometer_m_ += std::hypot(current_state_.x - last_position_x_,
                            current_state_.y - last_position_y_);
  last_position_x_ = current_state_.x;
  last_position_y_ = current_state_.y;
  current_state_.odometer_m = odometer_m_;
  current_state_.current_gear = active_cmd.gear;
  current_state_.steering_percentage_cmd =
      active_cmd.front_steering_rad / max_steer_angle_rad_ * 100.0;
  current_state_.throttle_percentage = active_cmd.throttle * 100.0;
  current_state_.throttle_percentage_cmd = current_state_.throttle_percentage;
  current_state_.brake_percentage = active_cmd.brake * 100.0;
  current_state_.brake_percentage_cmd = current_state_.brake_percentage;
  if (current_state_.is_collision) {
    AWARN_EVERY(100) << "Simulation collision detected at t="
                     << current_state_.timestamp_sec << ", position=("
                     << current_state_.x << ", " << current_state_.y << ", "
                     << current_state_.z << ")";
  }
  const bool invalid_state =
      !std::isfinite(current_state_.x) || !std::isfinite(current_state_.y) ||
      !std::isfinite(current_state_.z) ||
      !std::isfinite(current_state_.linear_velocity_mps) ||
      !std::isfinite(current_state_.angular_velocity_yaw_radps);
  if (invalid_state || current_state_.z < -0.1 ||
      std::abs(current_state_.linear_velocity_mps) > 100.0 ||
      std::abs(current_state_.angular_velocity_yaw_radps) > 20.0) {
    AERROR << "Invalid simulation physics state at t="
           << current_state_.timestamp_sec << ": position=(" << current_state_.x
           << ", " << current_state_.y << ", " << current_state_.z
           << "), velocity=" << current_state_.linear_velocity_mps
           << ", yaw_rate=" << current_state_.angular_velocity_yaw_radps;
  }
  if (FLAGS_sim_debug_log && FLAGS_sim_debug_log_interval > 0 &&
      step_count_ % FLAGS_sim_debug_log_interval == 0) {
    AINFO << "Simulation debug: t=" << current_state_.timestamp_sec
          << " cmd=(throttle=" << active_cmd.throttle
          << ", brake=" << active_cmd.brake
          << ", steer=" << active_cmd.front_steering_rad
          << ") state=(x=" << current_state_.x << ", y=" << current_state_.y
          << ", yaw=" << current_state_.yaw
          << ", vx=" << current_state_.linear_velocity_mps
          << ", vy=" << current_state_.lateral_velocity_mps
          << ") actuation=(steer_fl=" << last_actuation.wheel_steer_rad[0]
          << ", steer_fr=" << last_actuation.wheel_steer_rad[1]
          << ", drive_fl=" << last_actuation.drive_torque_nm[0]
          << ", brake_fl=" << last_actuation.brake_torque_nm[0] << ")";
  }
  return true;
}

bool SimulationEngine::GetVehicleState(VehicleState* state) const {
  if (!state) {
    return false;
  }
  *state = current_state_;
  return true;
}

}  // namespace simulation
}  // namespace apollo
