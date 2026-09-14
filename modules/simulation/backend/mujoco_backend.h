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
#include <array>
#include <string>

#include "modules/simulation/backend/simulator_backend.h"

namespace apollo {
namespace simulation {

/**
 * @class MujocoBackend
 * @brief Physics backend wrapping MuJoCo physics engine C API.
 */
class MujocoBackend : public ISimulatorBackend {
 public:
  MujocoBackend();
  ~MujocoBackend() override;

  bool Init(const std::string& model_path) override;
  bool ApplyActuation(const VehicleActuation& actuation) override;
  bool Step(double dt_sec) override;
  bool GetVehicleState(VehicleState* state) const override;
  void Reset(double x, double y, double yaw) override;
  bool SetVehicleGeometry(double wheelbase_m, double track_width_m,
                          double wheel_radius_m) override;
  bool SetMaxSteerAngle(double max_steer_angle_rad) override;
  bool SetMaxRearSteerAngle(double max_rear_steer_angle_rad) override;
  double SimulationTime() const override { return sim_time_sec_; }
  const std::string& Name() const override { return name_; }

 private:
  std::string name_{"MuJoCo"};
  std::string model_path_;
  double sim_time_sec_{0.0};
  double max_steer_angle_rad_{0.50};
  double model_wheelbase_m_{0.0};
  double model_track_width_m_{0.0};
  double model_wheel_radius_m_{0.0};
  double model_max_steer_angle_rad_{0.0};

  // MuJoCo opaque pointers (void* when headers not available)
  void* mj_model_{nullptr};
  void* mj_data_{nullptr};

  // Joint and actuator indices (cached from model)
  int steer_fl_id_{-1};
  int steer_fr_id_{-1};
  int steer_rl_id_{-1};
  int steer_rr_id_{-1};
  int steer_fl_joint_id_{-1};
  int steer_fr_joint_id_{-1};
  int steer_rl_joint_id_{-1};
  int steer_rr_joint_id_{-1};
  int drive_fl_id_{-1};
  int drive_fr_id_{-1};
  int drive_rl_id_{-1};
  int drive_rr_id_{-1};
  int wheel_fl_joint_id_{-1};
  int wheel_fr_joint_id_{-1};
  int wheel_rl_joint_id_{-1};
  int wheel_rr_joint_id_{-1};
  int vehicle_body_id_{-1};
  int freejoint_id_{-1};
  double init_z_{0.35};

  // State cache
  mutable VehicleState cached_state_{};
  mutable double previous_velocity_mps_{0.0};
  mutable double previous_lateral_velocity_mps_{0.0};
  mutable double previous_state_time_sec_{0.0};
  mutable double previous_yaw_{0.0};
  mutable std::array<double, 3> previous_world_velocity_mps_{0.0, 0.0, 0.0};
  mutable bool has_previous_state_{false};
  uint64_t debug_step_count_{0};
  VehicleActuation current_actuation_{};
  std::array<double, 4> wheel_velocity_sign_{0.0, 0.0, 0.0, 0.0};
};

}  // namespace simulation
}  // namespace apollo
