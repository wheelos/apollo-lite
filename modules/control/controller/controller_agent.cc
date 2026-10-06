/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/control/controller/controller_agent.h"

#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/controller/lat_controller.h"
#include "modules/control/controller/lon_controller.h"
#include "modules/control/controller/lon_speed_controller.h"
#include "modules/control/controller/mpc_controller.h"
#include "modules/control/controller/diff_drive_lat_controller.h"

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::cyber::Clock;

Status ControllerAgent::ValidateControllerSet(const ControlConf& conf) {
  unsigned lateral = 0;
  unsigned longitudinal = 0;
  bool coupled = false;
  for (const auto type : conf.active_controllers()) {
    switch (type) {
      case ControlConf::LAT_CONTROLLER:
      case ControlConf::DIFF_DRIVE_LAT_CONTROLLER:
        ++lateral;
        break;
      case ControlConf::LON_CONTROLLER:
      case ControlConf::LON_SPEED_CONTROLLER:
        ++longitudinal;
        break;
      case ControlConf::MPC_CONTROLLER:
        coupled = true;
        break;
      default:
        return Status(ErrorCode::CONTROL_INIT_ERROR,
                      "unknown controller type");
    }
  }
  if (conf.active_controllers_size() == 0 || lateral > 1 ||
      longitudinal > 1 ||
      (coupled && conf.active_controllers_size() != 1)) {
    return Status(ErrorCode::CONTROL_INIT_ERROR,
                  "controller set is empty or has conflicting output owners");
  }
  return Status::OK();
}

void ControllerAgent::RegisterControllers(const ControlConf *control_conf) {
  AINFO << "Initializing mutually exclusive controller output owners";
  for (auto active_controller : control_conf->active_controllers()) {
    switch (active_controller) {
      case ControlConf::MPC_CONTROLLER:
        controller_factory_.Register(
            ControlConf::MPC_CONTROLLER,
            []() -> Controller * { return new MPCController(); });
        break;
      case ControlConf::LAT_CONTROLLER:
        controller_factory_.Register(
            ControlConf::LAT_CONTROLLER,
            []() -> Controller * { return new LatController(); });
        break;
      case ControlConf::LON_CONTROLLER:
        controller_factory_.Register(
            ControlConf::LON_CONTROLLER,
            []() -> Controller * { return new LonController(); });
        break;
      case ControlConf::LON_SPEED_CONTROLLER:
        controller_factory_.Register(
            ControlConf::LON_SPEED_CONTROLLER,
            []() -> Controller * { return new LonSpeedController(); });
        break;
      case ControlConf::DIFF_DRIVE_LAT_CONTROLLER:
        controller_factory_.Register(
            ControlConf::DIFF_DRIVE_LAT_CONTROLLER,
            []() -> Controller * { return new DiffDriveLatController(); });
        break;
      default:
        AERROR << "Unknown active controller type:" << active_controller;
    }
  }
}

Status ControllerAgent::InitializeConf(const ControlConf *control_conf) {
  if (!control_conf) {
    AERROR << "control_conf is null";
    return Status(ErrorCode::CONTROL_INIT_ERROR, "Failed to load config");
  }
  control_conf_ = control_conf;
  for (auto controller_type : control_conf_->active_controllers()) {
    auto controller = controller_factory_.CreateObject(
        static_cast<ControlConf::ControllerType>(controller_type));
    if (controller) {
      controller_list_.emplace_back(std::move(controller));
    } else {
      AERROR << "Controller: " << controller_type << "is not supported";
      return Status(ErrorCode::CONTROL_INIT_ERROR,
                    "Invalid controller type: " + std::to_string(controller_type));
    }
  }
  return Status::OK();
}

Status ControllerAgent::Init(std::shared_ptr<DependencyInjector> injector,
                             const ControlConf *control_conf) {
  if (control_conf == nullptr || injector == nullptr ||
      !controller_list_.empty()) {
    AERROR << "Invalid controller agent initialization";
    return Status(ErrorCode::CONTROL_INIT_ERROR,
                  "agent requires configuration, injector and fresh state");
  }
  const auto validated = ValidateControllerSet(*control_conf);
  if (!validated.ok()) {
    AERROR << validated.error_message();
    return validated;
  }
  injector_ = injector;
  RegisterControllers(control_conf);
  const auto configured = InitializeConf(control_conf);
  if (!configured.ok()) {
    AERROR << configured.error_message();
    return configured;
  }
  for (auto &controller : controller_list_) {
    if (controller == nullptr) {
      return Status(ErrorCode::CONTROL_INIT_ERROR, "Controller is null.");
    }
    const auto initialized = controller->Init(injector, control_conf_);
    if (!initialized.ok()) {
      AERROR << "Controller <" << controller->Name() << "> init failed!";
      return initialized;
    }
    AINFO << "Controller <" << controller->Name() << "> init done!";
  }
  initialized_ = true;
  return Status::OK();
}

Status ControllerAgent::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory,
    control::ControlCommand *cmd) {
  if (!initialized_ || reset_failed_ || localization == nullptr ||
      chassis == nullptr || trajectory == nullptr || cmd == nullptr) {
    AERROR << "Controller computation requires initialized, reset-safe state";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "invalid controller state or input");
  }
  ControlCommand candidate = *cmd;
  for (auto &controller : controller_list_) {
    ADEBUG << "controller:" << controller->Name() << " processing ...";
    double start_timestamp = Clock::NowInSeconds();
    const auto computed = controller->ComputeControlCommand(
        localization, chassis, trajectory, &candidate);
    double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;

    ADEBUG << "controller: " << controller->Name()
           << " calculation time is: " << time_diff_ms << " ms.";
    candidate.mutable_latency_stats()->add_controller_time_ms(time_diff_ms);
    if (!computed.ok()) {
      AERROR << "Controller <" << controller->Name()
             << "> compute failed: " << computed.error_message();
      return computed;
    }
  }
  cmd->Swap(&candidate);
  return Status::OK();
}

Status ControllerAgent::Reset() {
  if (!initialized_) {
    AERROR << "Cannot reset an uninitialized controller agent";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "controller agent is not initialized");
  }
  Status first_failure = Status::OK();
  for (auto &controller : controller_list_) {
    ADEBUG << "controller:" << controller->Name() << " reset...";
    const auto reset = controller->Reset();
    if (!reset.ok()) {
      AERROR << "Controller <" << controller->Name()
             << "> reset failed: " << reset.error_message();
      if (first_failure.ok()) {
        first_failure = reset;
      }
    }
  }
  reset_failed_ = !first_failure.ok();
  return first_failure;
}

std::vector<std::string> ControllerAgent::ControllerNames() const {
  std::vector<std::string> names;
  for (const auto& controller : controller_list_) {
    names.push_back(controller->Name());
  }
  return names;
}

}  // namespace control
}  // namespace apollo
