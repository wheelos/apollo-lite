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

#include "modules/canbus/vehicle/mk_mini/mk_mini_controller.h"

#include "modules/common_msgs/basic_msgs/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/canbus/vehicle/mk_mini/mk_mini_message_manager.h"
#include "modules/canbus/vehicle/mk_mini/protocol/ctrl_fb_18c4d2ef.h"
#include "modules/canbus/vehicle/mk_mini/protocol/io_fb_18c4daef.h"
#include "modules/canbus/vehicle/mk_mini/protocol/veh_fb_diag_18c4eaef.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace mk_mini {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const double kMessageFeedbackPeriodThresholdRatio = 0.1;
const int32_t CHECK_RESPONSE_VCU_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_EPS_UNIT_FLAG = 2;
}  // namespace

ErrorCode Mk_miniController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "Mk_miniController has already been initiated.";
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
  ctrl_cmd_18c4d2d0_ = dynamic_cast<Ctrlcmd18c4d2d0*>(
      message_manager_->GetMutableProtocolDataById(Ctrlcmd18c4d2d0::ID));
  if (ctrl_cmd_18c4d2d0_ == nullptr) {
    AERROR << "Ctrlcmd18c4d2d0 does not exist in the Mk_miniMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  io_cmd_18c4d7d0_ = dynamic_cast<Iocmd18c4d7d0*>(
      message_manager_->GetMutableProtocolDataById(Iocmd18c4d7d0::ID));
  if (io_cmd_18c4d7d0_ == nullptr) {
    AERROR << "Iocmd18c4d7d0 does not exist in the Mk_miniMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_->AddMessage(Ctrlcmd18c4d2d0::ID, ctrl_cmd_18c4d2d0_, false);
  can_sender_->AddMessage(Iocmd18c4d7d0::ID, io_cmd_18c4d7d0_, false);

  // need sleep to ensure all messages received
  AINFO << "Mk_miniController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

Mk_miniController::~Mk_miniController() {}

bool Mk_miniController::Start() {
  if (!is_initialized_) {
    AERROR << "Mk_miniController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void Mk_miniController::Stop() {
  if (!is_initialized_) {
    AERROR << "Mk_miniController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "Mk_miniController stopped.";
  }
}

Chassis Mk_miniController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3 engine_started
  chassis_.set_engine_started(true);

  if (!chassis_detail.has_mk_mini()) {
    AERROR << "NO mk_mini chassis information!";
    return chassis_;
  }
  Mk_mini mk_mini = chassis_detail.mk_mini();

  // 5 speed_mps
  if (mk_mini.has_ctrl_fb_18c4d2ef() &&
      mk_mini.ctrl_fb_18c4d2ef().has_ctrl_fb_velocity()) {
    chassis_.set_speed_mps(mk_mini.ctrl_fb_18c4d2ef().ctrl_fb_velocity());
  } else {
    chassis_.set_speed_mps(0);
  }

  // 11 steering_percentage
  if (mk_mini.has_ctrl_fb_18c4d2ef() &&
      mk_mini.ctrl_fb_18c4d2ef().has_ctrl_fb_steering()) {
    chassis_.set_steering_percentage(static_cast<float>(
        ((mk_mini.ctrl_fb_18c4d2ef().ctrl_fb_steering() * M_PI / 180)) * 100.0 /
        vehicle_params_.max_steer_angle()));
  } else {
    chassis_.set_steering_percentage(0);
  }

  // 23 gear_location
  if (mk_mini.has_ctrl_fb_18c4d2ef() &&
      mk_mini.ctrl_fb_18c4d2ef().has_ctrl_fb_gear()) {
    int current_gear = mk_mini.ctrl_fb_18c4d2ef().ctrl_fb_gear();
    Chassis::GearPosition gear_pos = Chassis::GEAR_NONE;
    switch (current_gear) {
      case 0:
        gear_pos = Chassis::GEAR_INVALID;
        break;
      case 1:
        gear_pos = Chassis::GEAR_PARKING;
        break;
      case 2:
        gear_pos = Chassis::GEAR_REVERSE;
        break;
      case 3:
        gear_pos = Chassis::GEAR_NEUTRAL;
        break;
      case 4:
        gear_pos = Chassis::GEAR_DRIVE;
        break;
      default:
        AERROR << "Gear command is invalid! " << current_gear;
        break;
    }
    chassis_.set_gear_location(gear_pos);
  }

  // wheel speed
  // left rear
  if (mk_mini.has_lr_wheel_fb_18c4d7ef() &&
      mk_mini.lr_wheel_fb_18c4d7ef().has_lr_wheel_fb_velocity()) {
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rl_valid(true);
    if (mk_mini.lr_wheel_fb_18c4d7ef().lr_wheel_fb_velocity() > 1e-3) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
          WheelSpeed::FORWARD);
    } else if (mk_mini.lr_wheel_fb_18c4d7ef().lr_wheel_fb_velocity() < -1e-3) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
          WheelSpeed::BACKWARD);
    } else {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
          WheelSpeed::STANDSTILL);
    }
    chassis_.mutable_wheel_speed()->set_wheel_spd_rl(
        mk_mini.lr_wheel_fb_18c4d7ef().lr_wheel_fb_velocity());
  } else {
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rl_valid(false);
    chassis_.mutable_wheel_speed()->set_wheel_direction_rl(WheelSpeed::INVALID);
    chassis_.mutable_wheel_speed()->set_wheel_spd_rl(0);
  }
  // right rear
  if (mk_mini.has_rr_wheel_fb_18c4d8ef() &&
      mk_mini.rr_wheel_fb_18c4d8ef().has_rr_wheel_fb_velocity()) {
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rr_valid(true);
    if (mk_mini.rr_wheel_fb_18c4d8ef().rr_wheel_fb_velocity() > 1e-3) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rr(
          WheelSpeed::FORWARD);
    } else if (mk_mini.rr_wheel_fb_18c4d8ef().rr_wheel_fb_velocity() < -1e-3) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rr(
          WheelSpeed::BACKWARD);
    } else {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rr(
          WheelSpeed::STANDSTILL);
    }
    chassis_.mutable_wheel_speed()->set_wheel_spd_rr(
        mk_mini.rr_wheel_fb_18c4d8ef().rr_wheel_fb_velocity());
  } else {
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rr_valid(false);
    chassis_.mutable_wheel_speed()->set_wheel_direction_rr(WheelSpeed::INVALID);
    chassis_.mutable_wheel_speed()->set_wheel_spd_rr(0);
  }

  // bms
  if (mk_mini.has_bms_flag_infor_18c4e2ef()) {
    chassis_.set_battery_soc_percentage(
        mk_mini.bms_flag_infor_18c4e2ef().bms_flag_infor_soc());
  }

  return chassis_;
}

void Mk_miniController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
  if (chassis_error_code() == Chassis::NO_ERROR) {
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
  }
}

ErrorCode Mk_miniController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  // there no control command needed to enable speed and steering control

  // enable lamp control
  io_cmd_18c4d7d0_->set_io_cmd_enable(true);

  set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
  return ErrorCode::OK;
}

ErrorCode Mk_miniController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode Mk_miniController::EnableSteeringOnlyMode() { return ErrorCode::OK; }

ErrorCode Mk_miniController::EnableSpeedOnlyMode() { return ErrorCode::OK; }

// NEUTRAL, REVERSE, DRIVE
void Mk_miniController::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }

  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      ctrl_cmd_18c4d2d0_->set_ctrl_cmd_gear(3);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      ctrl_cmd_18c4d2d0_->set_ctrl_cmd_gear(2);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      ctrl_cmd_18c4d2d0_->set_ctrl_cmd_gear(4);
      break;
    }
    case Chassis::GEAR_PARKING: {
      ctrl_cmd_18c4d2d0_->set_ctrl_cmd_gear(1);
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!" << gear_position;
      ctrl_cmd_18c4d2d0_->set_ctrl_cmd_gear(0);
      break;
    }
    default: {
      ctrl_cmd_18c4d2d0_->set_ctrl_cmd_gear(0);
      break;
    }
  }
}

void Mk_miniController::Brake(double pedal) {}

void Mk_miniController::Throttle(double pedal) {}

void Mk_miniController::Acceleration(double acc) {}

void Mk_miniController::Speed(double speed) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current driving mode does not need to set speed.";
    return;
  }
  // speed is in m/s
  ctrl_cmd_18c4d2d0_->set_ctrl_cmd_velocity(speed);
}

void Mk_miniController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  ctrl_cmd_18c4d2d0_->set_ctrl_cmd_steering(real_angle);
}

void Mk_miniController::Steer(double angle, double angle_spd) {
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  ctrl_cmd_18c4d2d0_->set_ctrl_cmd_steering(real_angle);
}

void Mk_miniController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void Mk_miniController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void Mk_miniController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void Mk_miniController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  auto signal = command.signal().turn_signal();
  if (signal == common::VehicleSignal::TURN_LEFT) {
    io_cmd_18c4d7d0_->set_io_cmd_turn_lamp(1);
  } else if (signal == common::VehicleSignal::TURN_RIGHT) {
    io_cmd_18c4d7d0_->set_io_cmd_turn_lamp(2);
  } else {
    io_cmd_18c4d7d0_->set_io_cmd_turn_lamp(0);
  }
}

void Mk_miniController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool Mk_miniController::CheckChassisError() {
  // TODO(Pride Leong): check alive count and bcc
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  auto error_report = chassis_detail.mk_mini().veh_fb_diag_18c4eaef();

  int32_t error_cnt = 0;
  int32_t chassis_error_mask = 0;

  // Steer fault
  bool steer_fault =
      error_report.veh_fb_epsdisonline() | error_report.veh_fb_epsmosfetot() |
      error_report.veh_fb_epsfault() | error_report.veh_fb_epsdiswork() |
      error_report.veh_fb_epswarning() | error_report.veh_fb_epsovercurrent();

  chassis_error_mask |= error_report.veh_fb_epsdisonline() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_epsmosfetot() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_epsfault() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_epsdiswork() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_epswarning() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_epsovercurrent() << (error_cnt++);

  // Motor fault
  bool motor_fault =
      error_report.veh_fb_ldrvmcufault() | error_report.veh_fb_rdrvmcufault();

  chassis_error_mask |= error_report.veh_fb_ldrvmcufault() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_rdrvmcufault() << (error_cnt++);

  // EHB fault
  bool ehb_fault =
      error_report.veh_fb_ehboilfault() |
      error_report.veh_fb_ehboilpresssensorfault() |
      error_report.veh_fb_ehbmotorfault() |
      error_report.veh_fb_ehbsensorabnomal() |
      error_report.veh_fb_ehbpowerfault() | error_report.veh_fb_ehbot() |
      error_report.veh_fb_ehbangulefault() | error_report.veh_fb_ehbdisen() |
      error_report.veh_fb_ehbworkmodelfault() |
      error_report.veh_fb_ehbdisonline() | error_report.veh_fb_ehbecufault();

  chassis_error_mask |= error_report.veh_fb_ehboilfault() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_ehboilpresssensorfault()
                        << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_ehbmotorfault() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_ehbsensorabnomal() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_ehbpowerfault() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_ehbot() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_ehbangulefault() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_ehbdisen() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_ehbworkmodelfault()
                        << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_ehbdisonline() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_ehbecufault() << (error_cnt++);

  // can communication fault
  bool can_fault =
      error_report.veh_fb_autoiocancmd() | error_report.veh_fb_autocanctrlcmd();
  chassis_error_mask |= error_report.veh_fb_autoiocancmd() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_autocanctrlcmd() << (error_cnt++);

  // system fault
  bool system_fault = static_cast<bool>(error_report.veh_fb_faultlevel());
  chassis_error_mask |= static_cast<bool>(error_report.veh_fb_faultlevel())
                        << (error_cnt++);

  // emergency stop
  bool emergency_stopped = error_report.veh_fb_auxscram();
  chassis_error_mask |= error_report.veh_fb_auxscram() << (error_cnt++);

  // bms can fault
  bool bms_can_fault = error_report.veh_fb_auxbmsdisonline();
  // stickcontrol fault
  bool stickcontrol_fault = error_report.veh_fb_auxremotedisonline() |
                            error_report.veh_fb_auxremoteclose();
  chassis_error_mask |= error_report.veh_fb_auxbmsdisonline() << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_auxremotedisonline()
                        << (error_cnt++);
  chassis_error_mask |= error_report.veh_fb_auxremoteclose() << (error_cnt++);

  set_chassis_error_mask(chassis_error_mask);

  if (bms_can_fault || stickcontrol_fault) {
    AWARN << "BMS or stick control fault detected. ";
  }

  if (system_fault || emergency_stopped || can_fault || ehb_fault ||
      motor_fault || steer_fault) {
    AERROR << "Error detected: "
           << "system_fault: " << system_fault
           << ", emergency_stopped: " << emergency_stopped
           << ", can_fault: " << can_fault << ", ehb_fault: " << ehb_fault
           << ", motor_fault: " << motor_fault
           << ", steer_fault: " << steer_fault
           << ", error_report: " << error_report.DebugString();
    return true;
  }

  return false;
}

void Mk_miniController::SecurityDogThreadFunc() {
  if (can_sender_ == nullptr) {
    AERROR << "Failed to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = cyber::Time::Now().ToMicrosecond();

  int32_t vcu_ctrl_fail = 0;
  int32_t eps_ctrl_fail = 0;
  int32_t chassis_error = 0;

  while (can_sender_->IsRunning()) {
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_VCU_UNIT_FLAG, false)) {
      ++vcu_ctrl_fail;
      if (vcu_ctrl_fail >= kMaxFailAttempt) {
        AERROR << "VCU control failed for " << kMaxFailAttempt
               << " times, entering emergency mode. "
               << "Please check the VCU connection and status. "
               << "Current driving mode: " << Chassis::DrivingMode_Name(mode)
               << ".";
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vcu_ctrl_fail = 0;
    }

    // 2. steer check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_EPS_UNIT_FLAG, false)) {
      ++eps_ctrl_fail;
      if (eps_ctrl_fail >= kMaxFailAttempt) {
        AERROR << "EPS control failed for " << kMaxFailAttempt
               << " times, entering emergency mode. "
               << "Please check the EPS connection and status. "
               << "Current driving mode: " << Chassis::DrivingMode_Name(mode)
               << ".";
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      eps_ctrl_fail = 0;
    }

    // 3. chassis error check
    if (CheckChassisError()) {
      ++chassis_error;
      if (chassis_error >= kMaxFailAttempt) {
        AERROR << "Chassis error detected for " << kMaxFailAttempt
               << " times, entering emergency mode. "
               << "Please check the chassis status. "
               << "Chassis Error Mask: 0x" << std::hex << chassis_error_mask();
        emergency_mode = true;
        set_chassis_error_code(Chassis::CHASSIS_ERROR);
      }
    } else {
      chassis_error = 0;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      Emergency();
    }
    int64_t end = cyber::Time::Now().ToMicrosecond();
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
      start = cyber::Time::Now().ToMicrosecond();
    } else {
      AERROR_EVERY(100)
          << "Too much time consumption in LincolnController looping process:"
          << elapsed.count();
      start = end;
    }
  }
}

bool Mk_miniController::CheckResponse(const int32_t flags, bool need_wait) {
  // CheckResponse commonly takes 300ms. We leave a 100ms buffer for it.
  int32_t retry_num = 20;
  ChassisDetail chassis_detail;
  bool is_eps_online = false;
  bool is_vcu_online = false;

  do {
    if (message_manager_->GetSensorData(&chassis_detail) != ErrorCode::OK) {
      AERROR_EVERY(100) << "Get chassis detail failed.";
      return false;
    }
    bool check_ok = true;
    // TODO(Pride Leong): check alive count and bcc of feedback messages
    const auto error_report = chassis_detail.mk_mini().veh_fb_diag_18c4eaef();
    if (flags & CHECK_RESPONSE_VCU_UNIT_FLAG) {
      // check if motor in fault state
      is_vcu_online = (!error_report.veh_fb_rdrvmcufault() &&
                       !error_report.veh_fb_ldrvmcufault());
      ;
      check_ok = check_ok && is_vcu_online;
    }

    if (flags & CHECK_RESPONSE_EPS_UNIT_FLAG) {
      // check if eps online
      is_eps_online = (!error_report.veh_fb_epsdisonline() &&
                       !error_report.veh_fb_epsmosfetot() &&
                       !error_report.veh_fb_epsfault() &&
                       !error_report.veh_fb_epsdiswork() &&
                       !error_report.veh_fb_epswarning() &&
                       !error_report.veh_fb_epsovercurrent());
      check_ok = check_ok && is_eps_online;
    }

    if (check_ok) {
      return true;
    }
    AINFO << "Need to check response again.";
    if (need_wait) {
      --retry_num;
      std::this_thread::sleep_for(
          std::chrono::duration<double, std::milli>(20));
    }
  } while (need_wait && retry_num);

  AERROR << "check_response fail: is_eps_online:" << is_eps_online
         << ", is_vcu_online:" << is_vcu_online;
  return false;
}

void Mk_miniController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t Mk_miniController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode Mk_miniController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void Mk_miniController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
