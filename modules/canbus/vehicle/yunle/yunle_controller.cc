/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/yunle/yunle_controller.h"

#include "modules/common_msgs/basic_msgs/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/canbus/vehicle/yunle/yunle_message_manager.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace yunle {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}  // namespace

ErrorCode YunleController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "YunleController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }

  vehicle_params_.CopyFrom(
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param());
  params_.CopyFrom(params);
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part
  scu_1_121_ = dynamic_cast<Scu1121*>(
      message_manager_->GetMutableProtocolDataById(Scu1121::ID));
  if (scu_1_121_ == nullptr) {
    AERROR << "Scu1121 does not exist in the YunleMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  scu_tq_123_ = dynamic_cast<Scutq123*>(
      message_manager_->GetMutableProtocolDataById(Scutq123::ID));
  if (scu_tq_123_ == nullptr) {
    AERROR << "Scutq123 does not exist in the YunleMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Scu1121::ID, scu_1_121_, false);
  can_sender_->AddMessage(Scutq123::ID, scu_tq_123_, false);

  // need sleep to ensure all messages received
  AINFO << "YunleController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

YunleController::~YunleController() {}

bool YunleController::Start() {
  if (!is_initialized_) {
    AERROR << "YunleController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void YunleController::Stop() {
  if (!is_initialized_) {
    AERROR << "YunleController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "YunleController stopped.";
  }
}

Chassis YunleController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);
  /* ADD YOUR OWN CAR CHASSIS OPERATION
   */
  return chassis_;
}

void YunleController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode YunleController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;

  scu_1_121_->set_scu_drive_mode_req(1);

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
  return ErrorCode::OK;
}

ErrorCode YunleController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode YunleController::EnableSteeringOnlyMode() { return ErrorCode::OK; }

ErrorCode YunleController::EnableSpeedOnlyMode() { return ErrorCode::OK; }

// NEUTRAL, REVERSE, DRIVE
void YunleController::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }

  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      scu_1_121_->set_scu_shiftlevel_req(2);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      scu_1_121_->set_scu_shiftlevel_req(3);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      scu_1_121_->set_scu_shiftlevel_req(1);
      break;
    }
    default: {
      scu_1_121_->set_scu_shiftlevel_req(0);
      break;
    }
  }
}

// brake with pedal
// pedal:0.00~99.99 unit:
void YunleController::Brake(double pedal) {
  // double real_value = vehicle_params_.max_acceleration() * acceleration /
  // 100;
  // TODO(All) :  Update brake value based on mode
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }
}

// drive with pedal
// pedal:0.00~99.99 unit:
void YunleController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
}

// confirm the car is driven by acceleration command or drive/brake pedal
// drive with acceleration/deceleration
// acc:-7.0 ~ 5.0, unit:m/s^2
void YunleController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
}

void YunleController::Speed(double speed) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set speed.";
    return;
  }

  scu_1_121_->set_scu_target_speed(speed);
}

// yunle default, +470 ~ -470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with angle
// angle:-99.99~0.00~99.99, unit:, left:+, right:-
void YunleController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }

  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  // reverse sign
  scu_1_121_->set_scu_steering_wheel_angle_f(-real_angle);
  scu_1_121_->set_scu_steering_wheel_angle_r(real_angle);
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:+, right:-
// angle_spd:0.00~99.99, unit:deg/s
void YunleController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
}

void YunleController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    scu_1_121_->set_scu_brk_en(1);
  } else {
    scu_1_121_->set_scu_brk_en(0);
  }
}

void YunleController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void YunleController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void YunleController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  auto signal = command.signal().turn_signal();
  if (signal == common::VehicleSignal::TURN_LEFT) {
    turnsignal_68_->set_turn_left();
  } else if (signal == common::VehicleSignal::TURN_RIGHT) {
    turnsignal_68_->set_turn_right();
  } else {
    turnsignal_68_->set_turn_none();
  }
  */
}

void YunleController::ResetProtocol() { message_manager_->ResetSendMessages(); }

bool YunleController::CheckChassisError() {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
   */
  return false;
}

void YunleController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

  if (can_sender_ == nullptr) {
    AERROR << "Failed to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = 0;
  int64_t end = 0;
  while (can_sender_->IsRunning()) {
    start = ::apollo::cyber::Time::Now().ToMicrosecond();
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. horizontal control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false) {
      ++horizontal_ctrl_fail;
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      horizontal_ctrl_fail = 0;
    }

    // 2. vertical control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false)) {
      ++vertical_ctrl_fail;
      if (vertical_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vertical_ctrl_fail = 0;
    }
    if (CheckChassisError()) {
      set_chassis_error_code(Chassis::CHASSIS_ERROR);
      emergency_mode = true;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE);
      message_manager_->ResetSendMessages();
    }
    end = ::apollo::cyber::Time::Now().ToMicrosecond();
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR << "Too much time consumption in YunleController looping process:"
             << elapsed.count();
    }
  }
}

bool YunleController::CheckResponse(const int32_t flags, bool need_wait) {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
   */
  return false;
}

void YunleController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t YunleController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode YunleController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void YunleController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
