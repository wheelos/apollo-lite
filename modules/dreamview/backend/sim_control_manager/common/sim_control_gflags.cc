/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/dreamview/backend/sim_control_manager/common/sim_control_gflags.h"

DEFINE_string(sim_control_module_name, "sim_control", "Module name");

DEFINE_string(dynamic_model_name, "perfect_control",
              "the name for the selected model");

DEFINE_string(calibration_conf_file,
              "sim_control_lincoln.pb.txt",
              "the name for the selected model");

DEFINE_string(fnn_model_path, "sim_control/conf/fnn_model.bin",
              "fnn model file path");

DEFINE_string(backward_fnn_model_path,
              "sim_control/conf/fnn_model_backward.bin",
              "fnn model file path for reverse driving");

DEFINE_string(echo_lincoln_conf_file,
              "sim_control/conf/echo_lincoln_conf.pb.txt",
              "echo lincoln conf file path");

DEFINE_int32(reader_pending_queue_size, 10, "Cyber Reader pending queue size.");

// echosim model
DEFINE_string(echosim_license_file, "sim_control/conf/echosim.lic",
              "echo lincoln license file");
DEFINE_string(echosim_parameters_file, "sim_control/conf/echosim_lincoln.par",
              "echo lincoln vehicle parameter file");
DEFINE_double(echosim_simulation_step, 0.001, "echosim simulation step");
// DEFINE_double(dynamics_echosim_friction_coefficient, 0.85,
//               "ground friction coefficient");

DEFINE_bool(echo_lincoln_switch_model, false,
            "switch to point mass model at forward low speed");

DEFINE_bool(enable_steering_latency, false, "add steering latecy to control");

DEFINE_bool(enable_backward_fnn_model, true,
            "switch to add backward fnn model");

DEFINE_bool(enable_sim_at_nonauto_mode, true,
            "enable the simulation even before starting auto");

DEFINE_bool(enable_sim_control_custom_prediction, false,
            "publish custom prediction obstacles from sim control");

DEFINE_string(sim_control_custom_prediction_file, "",
              "custom prediction obstacle file for sim control; supports "
              "PredictionObstacles or PerceptionObstacles text/binary proto");

DEFINE_string(sim_control_spawn_mode, "legacy",
              "sim control ego spawn mode: legacy, localization_start, "
              "routing_start");

DEFINE_string(sim_control_prediction_mode, "legacy",
              "sim control prediction mode: legacy, empty, custom_file, "
              "external_passthrough");

DEFINE_string(sim_control_status_topic, "/apollo/sim_control/status",
              "status topic for structured sim control runtime state");

// cascade model
DEFINE_string(torch_gp_model_file,
              "sim_control/conf/LiteTrans-20201023-1043/gp_model.pt",
              "Gaussian process regression model file");

DEFINE_bool(use_cuda_in_cascade_model, false, "use CUDA for torch.");

DEFINE_string(
    cascade_model_conf_file,
    "sim_control/conf/LiteTrans-20201023-1043/standardization_factors.bin",
    "cascade model conf file path");

namespace apollo {
namespace dreamview {

apollo::sim_control::SimControlSpawnMode GetConfiguredSimControlSpawnMode() {
  if (FLAGS_sim_control_spawn_mode == "legacy") {
    return apollo::sim_control::SIM_CONTROL_SPAWN_MODE_LEGACY;
  }
  if (FLAGS_sim_control_spawn_mode == "localization_start") {
    return apollo::sim_control::SIM_CONTROL_SPAWN_MODE_LOCALIZATION_START;
  }
  if (FLAGS_sim_control_spawn_mode == "routing_start") {
    return apollo::sim_control::SIM_CONTROL_SPAWN_MODE_ROUTING_START;
  }
  return apollo::sim_control::SIM_CONTROL_SPAWN_MODE_UNKNOWN;
}

apollo::sim_control::SimControlPredictionMode
GetConfiguredSimControlPredictionMode() {
  if (FLAGS_sim_control_prediction_mode == "legacy") {
    return apollo::sim_control::SIM_CONTROL_PREDICTION_MODE_LEGACY;
  }
  if (FLAGS_sim_control_prediction_mode == "empty") {
    return apollo::sim_control::SIM_CONTROL_PREDICTION_MODE_EMPTY;
  }
  if (FLAGS_sim_control_prediction_mode == "custom_file") {
    return apollo::sim_control::SIM_CONTROL_PREDICTION_MODE_CUSTOM_FILE;
  }
  if (FLAGS_sim_control_prediction_mode == "external_passthrough") {
    return apollo::sim_control::SIM_CONTROL_PREDICTION_MODE_EXTERNAL_PASSTHROUGH;
  }
  return apollo::sim_control::SIM_CONTROL_PREDICTION_MODE_UNKNOWN;
}

std::string SimControlSpawnModeToString(
    apollo::sim_control::SimControlSpawnMode mode) {
  switch (mode) {
    case apollo::sim_control::SIM_CONTROL_SPAWN_MODE_LEGACY:
      return "legacy";
    case apollo::sim_control::SIM_CONTROL_SPAWN_MODE_EXPLICIT_START:
      return "explicit_start";
    case apollo::sim_control::SIM_CONTROL_SPAWN_MODE_LOCALIZATION_START:
      return "localization_start";
    case apollo::sim_control::SIM_CONTROL_SPAWN_MODE_ROUTING_START:
      return "routing_start";
    case apollo::sim_control::SIM_CONTROL_SPAWN_MODE_UNKNOWN:
    default:
      return "unknown";
  }
}

std::string SimControlPredictionModeToString(
    apollo::sim_control::SimControlPredictionMode mode) {
  switch (mode) {
    case apollo::sim_control::SIM_CONTROL_PREDICTION_MODE_LEGACY:
      return "legacy";
    case apollo::sim_control::SIM_CONTROL_PREDICTION_MODE_EMPTY:
      return "empty";
    case apollo::sim_control::SIM_CONTROL_PREDICTION_MODE_CUSTOM_FILE:
      return "custom_file";
    case apollo::sim_control::SIM_CONTROL_PREDICTION_MODE_EXTERNAL_PASSTHROUGH:
      return "external_passthrough";
    case apollo::sim_control::SIM_CONTROL_PREDICTION_MODE_UNKNOWN:
    default:
      return "unknown";
  }
}

}  // namespace dreamview
}  // namespace apollo
