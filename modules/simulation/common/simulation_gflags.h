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

#include "gflags/gflags.h"

DECLARE_string(sim_backend_type);
DECLARE_string(sim_control_mode);
DECLARE_string(sim_model_path);
DECLARE_double(sim_physics_dt);
DECLARE_double(sim_control_dt);
DECLARE_double(sim_command_timeout);
DECLARE_double(sim_speed_kp);
DECLARE_double(sim_init_x);
DECLARE_double(sim_init_y);
DECLARE_double(sim_init_yaw);
DECLARE_bool(sim_debug_log);
DECLARE_int32(sim_debug_log_interval);
