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
#include "modules/canbus/vehicle/yunle/protocol/ccu_status_51.h"
#include "modules/canbus/vehicle/yunle/protocol/waring_level_77.h"
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
const double kWheelRadius = 0.133;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
const double kMessageFeedbackPeriodThresholdRatio = 0.1;
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

  const auto yunle = chassis_detail.yunle();

  // soc
  if (yunle.has_bms_soc_101()) {
    chassis_.set_battery_soc_percentage(yunle.bms_soc_101().rsoc());
  } else {
    chassis_.clear_battery_soc_percentage();
  }

  // wheel speed and vehicle speed
  if (yunle.has_wheel_speed_feedback_rpm_168()) {
    const auto wheel_speed_fb = yunle.wheel_speed_feedback_rpm_168();
    // front left
    double fl_speed =
        wheel_speed_fb.front_left_rpm() / 60 * 2 * M_PI * kWheelRadius;
    chassis_.mutable_wheel_speed()->set_wheel_spd_fl(fl_speed);
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fl_valid(true);
    if (wheel_speed_fb.front_left_rpm() > 1e-1) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fl(
          WheelSpeed::FORWARD);
    } else if (wheel_speed_fb.front_left_rpm() < -1e-1) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fl(
          WheelSpeed::BACKWARD);
    } else {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fl(
          WheelSpeed::STANDSTILL);
    }
    // front right
    double fr_speed =
        wheel_speed_fb.front_right_rpm() / 60 * 2 * M_PI * kWheelRadius;
    chassis_.mutable_wheel_speed()->set_wheel_spd_fr(fr_speed);
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fr_valid(true);
    if (wheel_speed_fb.front_right_rpm() > 1e-1) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fr(
          WheelSpeed::FORWARD);
    } else if (wheel_speed_fb.front_right_rpm() < -1e-1) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fr(
          WheelSpeed::BACKWARD);
    } else {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fr(
          WheelSpeed::STANDSTILL);
    }
    // rear left
    double rl_speed =
        wheel_speed_fb.rear_left_rpm() / 60 * 2 * M_PI * kWheelRadius;
    chassis_.mutable_wheel_speed()->set_wheel_spd_rl(rl_speed);
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rl_valid(true);
    if (wheel_speed_fb.rear_left_rpm() > 1e-1) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
          WheelSpeed::FORWARD);
    } else if (wheel_speed_fb.rear_left_rpm() < -1e-1) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
          WheelSpeed::BACKWARD);
    } else {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
          WheelSpeed::STANDSTILL);
    }
    // rear right
    double rr_speed =
        wheel_speed_fb.rear_right_rpm() / 60 * 2 * M_PI * kWheelRadius;
    chassis_.mutable_wheel_speed()->set_wheel_spd_rr(rr_speed);
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rr_valid(true);
    if (wheel_speed_fb.rear_right_rpm() > 1e-1) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rr(
          WheelSpeed::FORWARD);
    } else if (wheel_speed_fb.rear_right_rpm() < -1e-1) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rr(
          WheelSpeed::BACKWARD);
    } else {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rr(
          WheelSpeed::STANDSTILL);
    }
  } else {
    chassis_.mutable_wheel_speed()->clear_wheel_spd_fl();
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fl_valid(false);
    chassis_.mutable_wheel_speed()->set_wheel_direction_fl(WheelSpeed::INVALID);
    chassis_.mutable_wheel_speed()->clear_wheel_spd_fr();
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fr_valid(false);
    chassis_.mutable_wheel_speed()->set_wheel_direction_fr(WheelSpeed::INVALID);
    chassis_.mutable_wheel_speed()->clear_wheel_spd_rl();
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rl_valid(false);
    chassis_.mutable_wheel_speed()->set_wheel_direction_rl(WheelSpeed::INVALID);
    chassis_.mutable_wheel_speed()->clear_wheel_spd_rr();
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rr_valid(false);
    chassis_.mutable_wheel_speed()->set_wheel_direction_rr(WheelSpeed::INVALID);
  }

  if (yunle.has_ccu_status_51()) {
    auto ccu_status = yunle.ccu_status_51();

    // gear
    switch (ccu_status.ccu_shiftlevel_sts()) {
      case 0x0:
        chassis_.set_gear_location(Chassis::GEAR_INVALID);
        break;
      case 0x1:
        chassis_.set_gear_location(Chassis::GEAR_DRIVE);
        break;
      case 0x2:
        chassis_.set_gear_location(Chassis::GEAR_NEUTRAL);
        break;
      case 0x3:
        chassis_.set_gear_location(Chassis::GEAR_REVERSE);
        break;
      default:
        AERROR << "Unknown gear location: " << ccu_status.ccu_shiftlevel_sts();
        chassis_.set_gear_location(Chassis::GEAR_INVALID);
        break;
    }
    // epb
    if (ccu_status.ccu_p_sts()) {
      chassis_.set_parking_brake(true);
      chassis_.set_gear_location(Chassis::GEAR_PARKING);
    } else {
      chassis_.set_parking_brake(false);
    }

    // vehicle speed
    // convert to m/s
    double vehicle_speed = ccu_status.ccu_vehicle_speed() / 3.6;
    chassis_.set_speed_mps(vehicle_speed);

    // steer
    double steer_direction_factor = 1.0;
    if (ccu_status.steering_wheel_direction()) {
      steer_direction_factor = -1.0;  // reverse sign
    }
    // 120 is the max steering angle in Yunle, we convert it to percentage
    double steer_angle_percentage = ccu_status.ccu_steering_wheel_angle() /
                                    120 * 100 * steer_direction_factor;
    chassis_.set_steering_percentage(steer_angle_percentage);

    // turn signal
    if (ccu_status.left_turn_light_sts() && ccu_status.right_turn_light_sts()) {
      chassis_.mutable_signal()->set_turn_signal(
          common::VehicleSignal::TURN_HAZARD_WARNING);
    } else if (ccu_status.left_turn_light_sts()) {
      chassis_.mutable_signal()->set_turn_signal(
          common::VehicleSignal::TURN_LEFT);
    } else if (ccu_status.right_turn_light_sts()) {
      chassis_.mutable_signal()->set_turn_signal(
          common::VehicleSignal::TURN_RIGHT);
    } else {
      chassis_.mutable_signal()->set_turn_signal(
          common::VehicleSignal::TURN_NONE);
    }

    // beam
    if (ccu_status.lowbeam_sts()) {
      chassis_.mutable_signal()->set_low_beam(true);
    } else {
      chassis_.mutable_signal()->set_low_beam(false);
    }
  }

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
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }
  // enable brake, vehicle will brake with fixed deceleration
  // scu_1_121_->set_scu_brk_en(true);
  // TODO(All) :  Update brake value based on mode
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
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
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

  // convert to m/s
  scu_1_121_->set_scu_target_speed(speed * 3.6);
}

// yunle default, -120 ~ +120, left:-, right:+
// need to be compatible with control module, so reverse
// steering with angle
// angle:-99.99~0.00~99.99, unit:, left:+, right:-
void YunleController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }

  int32_t real_angle = static_cast<int32_t>(angle / 100 * 120);
  // limit the angle to [-120, 120]
  if (real_angle > 120) {
    real_angle = 120;
  } else if (real_angle < -120) {
    real_angle = -120;
  }
  int32_t real_angle_f = real_angle;
  int32_t real_angle_r = real_angle;
  // here, we use the minimum steering radius mode, rear steering angle is
  // negative of front, the vehicle already has this mode, so just set the same
  // TODO(All): if using another steering mode, such as Ackermann or crab
  // steering, you should change the implementation below in yunle vechicle:
  //  0~120 means turning right
  //  256~136 means turning left
  if (angle > 0) {
    real_angle_f = 256 - real_angle;
    real_angle_r = 256 - real_angle;
  } else if (angle < 0) {
    real_angle_f = 0 - real_angle;
    real_angle_r = 0 - real_angle;
  } else {
    real_angle_f = 0;
    real_angle_r = 0;
  }

  scu_1_121_->set_scu_steering_wheel_angle_f(real_angle_f);
  scu_1_121_->set_scu_steering_wheel_angle_r(real_angle_r);
}

// yunle default, -120 ~ +120, left:-, right:+
// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:+, right:-
// angle_spd:0.00~99.99, unit:deg/s
void YunleController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }

  int32_t real_angle = static_cast<int32_t>(angle / 100 * 120);
  // limit the angle to [-120, 120]
  if (real_angle > 120) {
    real_angle = 120;
  } else if (real_angle < -120) {
    real_angle = -120;
  }
  int32_t real_angle_f = real_angle;
  int32_t real_angle_r = real_angle;
  // here, we use the minimum steering radius mode, rear steering angle is
  // negative of front, the vehicle already has this mode, so just set the same
  // TODO(All): if using another steering mode, such as Ackermann or crab
  // steering, you should change the implementation below in yunle vechicle:
  //  0~120 means turning right
  //  256~136 means turning left
  if (angle > 0) {
    real_angle_f = 256 - real_angle;
    real_angle_r = 256 - real_angle;
  } else if (angle < 0) {
    real_angle_f = 0 - real_angle;
    real_angle_r = 0 - real_angle;
  } else {
    real_angle_f = 0;
    real_angle_r = 0;
  }

  scu_1_121_->set_scu_steering_wheel_angle_f(real_angle_f);
  scu_1_121_->set_scu_steering_wheel_angle_r(real_angle_r);
}

void YunleController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    scu_1_121_->set_scu_brk_en(1);
  } else {
    scu_1_121_->set_scu_brk_en(0);
  }
}

void YunleController::SetBeam(const ControlCommand& command) {
  // set position light always
  // brake light and position light use the same device setting `on` means brake
  // light, so no need to set position light
  // TODO(All): Maybe other type of vehicle have different settings
  // scu_1_121_->set_gw_position_light_req(1);

  if (command.signal().high_beam()) {
    // None
    scu_1_121_->set_gw_lowbeam_req(1);
  } else if (command.signal().low_beam()) {
    scu_1_121_->set_gw_lowbeam_req(1);
  } else {
    scu_1_121_->set_gw_lowbeam_req(0);
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
  if (command.signal().has_turn_signal()) {
    auto signal = command.signal().turn_signal();
    if (signal == common::VehicleSignal::TURN_HAZARD_WARNING) {
      scu_1_121_->set_gw_left_turn_light_req(1);
      scu_1_121_->set_gw_right_turn_light_req(1);
    } else if (signal == common::VehicleSignal::TURN_LEFT) {
      scu_1_121_->set_gw_left_turn_light_req(1);
      scu_1_121_->set_gw_right_turn_light_req(0);
    } else if (signal == common::VehicleSignal::TURN_RIGHT) {
      scu_1_121_->set_gw_left_turn_light_req(0);
      scu_1_121_->set_gw_right_turn_light_req(1);
    } else {
      scu_1_121_->set_gw_left_turn_light_req(0);
      scu_1_121_->set_gw_right_turn_light_req(0);
    }
  } else {
    // reset to 0 if no turn signal input
    scu_1_121_->set_gw_left_turn_light_req(0);
    scu_1_121_->set_gw_right_turn_light_req(0);
  }
}

void YunleController::ResetProtocol() { message_manager_->ResetSendMessages(); }

bool YunleController::CheckChassisError() {
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  int32_t error_cnt = 0;
  int32_t chassis_error_mask = 0;

  auto warnings = chassis_detail.yunle().waring_level_77();

  // in yunle vehicle, level means:
  //  level 0, no warning
  //  level 1, warning notice
  //  level 2, deceleration
  //  level 3, emergency stop
  // we consider level 0 and 1 as no error, and level 2 and 3 as error
  // TODO(All): configurable warning level or make more strict

  bool steer_warning = warnings.turn_disconnect_warning() > 1 ||
                       warnings.turn_lock_warning() > 1 ||
                       warnings.turn_unstoppable_warning() > 1;

  chassis_error_mask |= (warnings.turn_disconnect_warning() > 1)
                        << (error_cnt++);
  chassis_error_mask |= (warnings.turn_lock_warning() > 1) << (error_cnt++);
  chassis_error_mask |= (warnings.turn_unstoppable_warning() > 1)
                        << (error_cnt++);

  bool speed_warning =
      warnings.mcu_speed_warning() > 1 || warnings.mcu_motor_warning() > 1;
  chassis_error_mask |= (warnings.mcu_speed_warning() > 1) << (error_cnt++);
  chassis_error_mask |= (warnings.mcu_motor_warning() > 1) << (error_cnt++);

  bool mcu_warning =
      warnings.mcu_disconnect_warning() > 1 || warnings.mcu_cur_warning() > 1 ||
      warnings.mcu_temperature_warning() > 1 || warnings.mcu_vol_warning() > 1;
  chassis_error_mask |= (warnings.mcu_disconnect_warning() > 1)
                        << (error_cnt++);
  chassis_error_mask |= (warnings.mcu_cur_warning() > 1) << (error_cnt++);
  chassis_error_mask |= (warnings.mcu_temperature_warning() > 1)
                        << (error_cnt++);
  chassis_error_mask |= (warnings.mcu_vol_warning() > 1) << (error_cnt++);

  bool bms_warning = warnings.bms_temperature_warning() > 1 ||
                     warnings.bms_soc_warning() > 1 ||
                     warnings.bms_dischargecur_warning() > 1 ||
                     warnings.bms_chargecur_warning() > 1;
  chassis_error_mask |= (warnings.bms_temperature_warning() > 1)
                        << (error_cnt++);
  chassis_error_mask |= (warnings.bms_soc_warning() > 1) << (error_cnt++);
  chassis_error_mask |= (warnings.bms_dischargecur_warning() > 1)
                        << (error_cnt++);
  chassis_error_mask |= (warnings.bms_chargecur_warning() > 1) << (error_cnt++);

  set_chassis_error_mask(chassis_error_mask);

  if (steer_warning || speed_warning || mcu_warning || bms_warning) {
    AERROR_EVERY(100) << "Chassis error detected: "
                      << "steer_warning: " << steer_warning
                      << ", speed_warning: " << speed_warning
                      << ", mcu_warning: " << mcu_warning
                      << ", bms_warning: " << bms_warning
                      << ", chassis_error_mask: " << chassis_error_mask;
    return true;
  }

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
    const auto warnings = chassis_detail.yunle().waring_level_77();
    if (flags & CHECK_RESPONSE_SPEED_UNIT_FLAG) {
      // check if motor and speed in warning level
      is_vcu_online = (!(warnings.mcu_speed_warning() > 1) &&
                       !(warnings.mcu_motor_warning() > 1));
      ;
      check_ok = check_ok && is_vcu_online;
    }

    if (flags & CHECK_RESPONSE_STEER_UNIT_FLAG) {
      // check if eps online
      is_eps_online = (!(warnings.turn_unstoppable_warning() > 1) &&
                       !(warnings.turn_lock_warning() > 1) &&
                       !(warnings.turn_disconnect_warning() > 1));
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

  AERROR_EVERY(100) << "check_response fail: is_eps_online:" << is_eps_online
                    << ", is_vcu_online:" << is_vcu_online;
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
