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

#include "modules/simulation/common/simulation_gflags.h"

DEFINE_string(sim_backend_type, "kinematic",
              "Simulation backend type: kinematic or mujoco");
DEFINE_string(sim_control_mode, "throttle",
              "Longitudinal control mode: throttle or speed");
DEFINE_string(sim_model_path, "",
              "Path to physics model asset (e.g. MJCF XML)");
DEFINE_double(sim_physics_dt, 0.002,
              "Physics substepping dt in seconds (e.g. 500 Hz)");
DEFINE_double(sim_control_dt, 0.02,
              "Deterministic simulation control period in seconds");
DEFINE_double(sim_command_timeout, 0.20,
              "Command timeout threshold in seconds");
DEFINE_double(sim_speed_kp, 1.5, "Proportional gain for target-speed control");
DEFINE_double(sim_init_x, 0.0, "Initial vehicle X position in meters");
DEFINE_double(sim_init_y, 0.0, "Initial vehicle Y position in meters");
DEFINE_double(sim_init_yaw, 0.0, "Initial vehicle heading angle in radians");
DEFINE_bool(sim_debug_log, false,
            "Enable sampled simulation physics debug logs");
DEFINE_int32(sim_debug_log_interval, 20,
             "Simulation debug log interval in backend or control steps");
