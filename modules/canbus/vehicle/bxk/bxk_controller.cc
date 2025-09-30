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

#include "modules/canbus/vehicle/bxk/bxk_controller.h"

#include "modules/common_msgs/basic_msgs/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/canbus/vehicle/bxk/bxk_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace bxk {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}  // namespace

ErrorCode BxkController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "BxkController has already been initiated.";
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
  light_control_7ff_ = dynamic_cast<Lightcontrol7ff*>(
      message_manager_->GetMutableProtocolDataById(Lightcontrol7ff::ID));
  if (light_control_7ff_ == nullptr) {
    AERROR << "Lightcontrol7ff does not exist in the BxkMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  wheel_fl_control_2b7_ = dynamic_cast<Wheelflcontrol2b7*>(
      message_manager_->GetMutableProtocolDataById(Wheelflcontrol2b7::ID));
  if (wheel_fl_control_2b7_ == nullptr) {
    AERROR << "Wheelflcontrol2b7 does not exist in the BxkMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  wheel_fr_control_2b6_ = dynamic_cast<Wheelfrcontrol2b6*>(
      message_manager_->GetMutableProtocolDataById(Wheelfrcontrol2b6::ID));
  if (wheel_fr_control_2b6_ == nullptr) {
    AERROR << "Wheelfrcontrol2b6 does not exist in the BxkMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  wheel_rl_control_2b8_ = dynamic_cast<Wheelrlcontrol2b8*>(
      message_manager_->GetMutableProtocolDataById(Wheelrlcontrol2b8::ID));
  if (wheel_rl_control_2b8_ == nullptr) {
    AERROR << "Wheelrlcontrol2b8 does not exist in the BxkMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  wheel_rr_control_2b9_ = dynamic_cast<Wheelrrcontrol2b9*>(
      message_manager_->GetMutableProtocolDataById(Wheelrrcontrol2b9::ID));
  if (wheel_rr_control_2b9_ == nullptr) {
    AERROR << "Wheelrrcontrol2b9 does not exist in the BxkMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Lightcontrol7ff::ID, light_control_7ff_, false);
  can_sender_->AddMessage(Wheelflcontrol2b7::ID, wheel_fl_control_2b7_, false);
  can_sender_->AddMessage(Wheelfrcontrol2b6::ID, wheel_fr_control_2b6_, false);
  can_sender_->AddMessage(Wheelrlcontrol2b8::ID, wheel_rl_control_2b8_, false);
  can_sender_->AddMessage(Wheelrrcontrol2b9::ID, wheel_rr_control_2b9_, false);

  // need sleep to ensure all messages received
  AINFO << "BxkController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

BxkController::~BxkController() {}

bool BxkController::Start() {
  if (!is_initialized_) {
    AERROR << "BxkController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void BxkController::Stop() {
  if (!is_initialized_) {
    AERROR << "BxkController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "BxkController stopped.";
  }
}

Chassis BxkController::chassis() {
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

  // fr wheel (ID: 438)
  if (chassis_detail.has_bxk() &&
      chassis_detail.bxk().has_wheel_fr_status_1b6()) {
    auto fr_report = chassis_detail.bxk().wheel_fr_status_1b6();
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fr_valid(true);
    chassis_.mutable_wheel_speed()->set_wheel_spd_fr(fr_report.speed_fr());
    chassis_.mutable_wheel_speed()->set_wheel_direction_fr(
        (fr_report.speed_fr() > 1e-3)
            ? WheelSpeed::FORWARD
            : ((fr_report.speed_fr() < -1e-3) ? WheelSpeed::BACKWARD
                                              : WheelSpeed::STANDSTILL));
  } else {
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fr_valid(false);
    chassis_.mutable_wheel_speed()->set_wheel_spd_fr(0.0);
    chassis_.mutable_wheel_speed()->set_wheel_direction_fr(WheelSpeed::INVALID);
  }

  // fl wheel (ID: 439)
  if (chassis_detail.has_bxk() &&
      chassis_detail.bxk().has_wheel_fl_status_1b7()) {
    auto fl_report = chassis_detail.bxk().wheel_fl_status_1b7();
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fl_valid(true);
    chassis_.mutable_wheel_speed()->set_wheel_spd_fl(fl_report.speed_fl());
    chassis_.mutable_wheel_speed()->set_wheel_direction_fl(
        (fl_report.speed_fl() > 1e-3)
            ? WheelSpeed::FORWARD
            : ((fl_report.speed_fl() < -1e-3) ? WheelSpeed::BACKWARD
                                              : WheelSpeed::STANDSTILL));
  } else {
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fl_valid(false);
    chassis_.mutable_wheel_speed()->set_wheel_spd_fl(0.0);
    chassis_.mutable_wheel_speed()->set_wheel_direction_fl(WheelSpeed::INVALID);
  }

  // rl wheel (ID: 440)
  if (chassis_detail.has_bxk() &&
      chassis_detail.bxk().has_wheel_rl_status_1b8()) {
    auto rl_report = chassis_detail.bxk().wheel_rl_status_1b8();
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rl_valid(true);
    chassis_.mutable_wheel_speed()->set_wheel_spd_rl(rl_report.speed_rl());
    chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
        (rl_report.speed_rl() > 1e-3)
            ? WheelSpeed::FORWARD
            : ((rl_report.speed_rl() < -1e-3) ? WheelSpeed::BACKWARD
                                              : WheelSpeed::STANDSTILL));
  } else {
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rl_valid(false);
    chassis_.mutable_wheel_speed()->set_wheel_spd_rl(0.0);
    chassis_.mutable_wheel_speed()->set_wheel_direction_rl(WheelSpeed::INVALID);
  }

  // rr wheel (ID: 441)
  if (chassis_detail.has_bxk() &&
      chassis_detail.bxk().has_wheel_rr_status_1b9()) {
    auto rr_report = chassis_detail.bxk().wheel_rr_status_1b9();
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rr_valid(true);
    chassis_.mutable_wheel_speed()->set_wheel_spd_rr(rr_report.speed_rr());
    chassis_.mutable_wheel_speed()->set_wheel_direction_rr(
        (rr_report.speed_rr() > 1e-3)
            ? WheelSpeed::FORWARD
            : ((rr_report.speed_rr() < -1e-3) ? WheelSpeed::BACKWARD
                                              : WheelSpeed::STANDSTILL));
  } else {
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rr_valid(false);
    chassis_.mutable_wheel_speed()->set_wheel_spd_rr(0.0);
    chassis_.mutable_wheel_speed()->set_wheel_direction_rr(WheelSpeed::INVALID);
  }
  // TODO(All): calculate vehicle speed from wheel speed

  // TODO(All): add more chassis feedbacks

  return chassis_;
}

void BxkController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode BxkController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  // fr wheel control
  // reset speed
  wheel_fr_control_2b6_->set_speed_fr_ctrl(0);
  // enable running
  wheel_fr_control_2b6_->set_runningstate_fr(
      Wheel_fr_control_2b6::RUNNINGSTATE_FR_RUNNING);
  // forward
  wheel_fr_control_2b6_->set_directionstate_fr(
      Wheel_fr_control_2b6::DIRECTIONSTATE_FR_FORWARD);

  // fl wheel control
  wheel_fl_control_2b7_->set_speed_fl_ctrl(0);
  wheel_fl_control_2b7_->set_runningstate_fl(
      Wheel_fl_control_2b7::RUNNINGSTATE_FL_RUNNING);
  wheel_fl_control_2b7_->set_directionstate_fl(
      Wheel_fl_control_2b7::DIRECTIONSTATE_FL_FORWARD);

  // rl wheel control
  wheel_rl_control_2b8_->set_speed_rl_ctrl(0);
  wheel_rl_control_2b8_->set_runningstate_rl(
      Wheel_rl_control_2b8::RUNNINGSTATE_RL_RUNNING);
  wheel_rl_control_2b8_->set_directionstate_rl(
      Wheel_rl_control_2b8::DIRECTIONSTATE_RL_FORWARD);

  // rr wheel control
  wheel_rr_control_2b9_->set_speed_rr_ctrl(0);
  wheel_rr_control_2b9_->set_runningstate_rr(
      Wheel_rr_control_2b9::RUNNINGSTATE_RR_RUNNING);
  wheel_rr_control_2b9_->set_directionstate_rr(
      Wheel_rr_control_2b9::DIRECTIONSTATE_RR_FORWARD);

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

ErrorCode BxkController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode BxkController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode.";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;
}

ErrorCode BxkController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;
}

// NEUTRAL, REVERSE, DRIVE
void BxkController::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      gear_66_->set_gear_neutral();
      break;
    }
    case Chassis::GEAR_REVERSE: {
      gear_66_->set_gear_reverse();
      break;
    }
    case Chassis::GEAR_DRIVE: {
      gear_66_->set_gear_drive();
      break;
    }
    case Chassis::GEAR_PARKING: {
      gear_66_->set_gear_park();
      break;
    }
    case Chassis::GEAR_LOW: {
      gear_66_->set_gear_low();
      break;
    }
    case Chassis::GEAR_NONE: {
      gear_66_->set_gear_none();
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      gear_66_->set_gear_none();
      break;
    }
    default: {
      gear_66_->set_gear_none();
      break;
    }
  }
  */
}

// brake with pedal
// pedal:0.00~99.99 unit:
void BxkController::Brake(double pedal) {
  // double real_value = vehicle_params_.max_acceleration() * acceleration /
  // 100;
  // TODO(All) :  Update brake value based on mode
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_pedal(pedal);
  */
}

// drive with pedal
// pedal:0.00~99.99 unit:
void BxkController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  throttle_62_->set_pedal(pedal);
  */
}

// drive with speed
// unit: m/s, fwd:+, rev:-
void BxkController::Speed(double speed) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set speed.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
   */
}

// confirm the car is driven by acceleration command or drive/brake pedal
// drive with acceleration/deceleration
// acc:-7.0 ~ 5.0, unit:m/s^2
void BxkController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
   */
}

// bxk default, +470 ~ -470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with angle
// angle:-99.99~0.00~99.99, unit:, left:+, right:-
void BxkController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  // const double real_angle =
  //     vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  // reverse sign
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  steering_64_->set_steering_angle(real_angle)->set_steering_angle_speed(200);
  */
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:+, right:-
// angle_spd:0.00~99.99, unit:deg/s
void BxkController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  const double real_angle_spd =
  ProtocolData<::apollo::canbus::ChassisDetail>::BoundedValue(
      vehicle_params_.min_steer_angle_rate(),
  vehicle_params_.max_steer_angle_rate(), vehicle_params_.max_steer_angle_rate()
  * angle_spd / 100.0); steering_64_->set_steering_angle(real_angle)
      ->set_steering_angle_speed(real_angle_spd);
  */
}

void BxkController::DifferentialSpeed(double linear_vel, double angular_vel) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "Skip wheel speed control: not in auto drive mode.";
    return;
  }

  // === vehicle params ===
  // TODO(All): read from vehicle_params_
  const double wheel_radius = 0.23;  // m
  const double wheel_base = 1.16;    // m (L)
  // TODO(All): add these params fields and read from vehicle_params_
  const double wheel_track_f = 0.86;  // m (Wf)
  const double wheel_track_r = 0.67;  // m (Wr)

  // === convert function m/s -> RPM ===
  auto mps_to_rpm = [&](double v) {
    return (v / (2 * M_PI * wheel_radius)) * 60.0;
  };

  // === calculate speed of wheels ===
  double v_fr, v_fl, v_rl, v_rr;

  if (fabs(angular_vel) < 1e-6) {
    // go straight
    v_fr = v_fl = v_rl = v_rr = linear_vel;
  } else {
    double r = linear_vel / angular_vel;  // turning radius

    v_fr = angular_vel *
           sqrt(pow(r + wheel_track_f / 2.0, 2) + pow(wheel_base / 2.0, 2));
    v_fl = angular_vel *
           sqrt(pow(r - wheel_track_f / 2.0, 2) + pow(wheel_base / 2.0, 2));
    v_rl = angular_vel *
           sqrt(pow(r - wheel_track_r / 2.0, 2) + pow(wheel_base / 2.0, 2));
    v_rr = angular_vel *
           sqrt(pow(r + wheel_track_r / 2.0, 2) + pow(wheel_base / 2.0, 2));
  }

  // === convert to RPM ===
  double rpm_fr = mps_to_rpm(v_fr);
  double rpm_fl = mps_to_rpm(v_fl);
  double rpm_rl = mps_to_rpm(v_rl);
  double rpm_rr = mps_to_rpm(v_rr);

  // === set protocol data ===
  wheel_fr_control_2b6_->set_speed_fr_ctrl(rpm_fr);
  wheel_fr_control_2b6_->set_runningstate_fr(
      fabs(v_fr) > 1e-3 ? Wheel_fr_control_2b6::RUNNINGSTATE_FR_RUNNING
                        : Wheel_fr_control_2b6::RUNNINGSTATE_FR_STOPPED);
  wheel_fr_control_2b6_->set_directionstate_fr(
      v_fr >= 0 ? Wheel_fr_control_2b6::DIRECTIONSTATE_FR_FORWARD
                : Wheel_fr_control_2b6::DIRECTIONSTATE_FR_REVERSE);

  wheel_fl_control_2b7_->set_speed_fl_ctrl(rpm_fl);
  wheel_fl_control_2b7_->set_runningstate_fl(
      fabs(v_fl) > 1e-3 ? Wheel_fl_control_2b7::RUNNINGSTATE_FL_RUNNING
                        : Wheel_fl_control_2b7::RUNNINGSTATE_FL_STOPPED);
  wheel_fl_control_2b7_->set_directionstate_fl(
      v_fl >= 0 ? Wheel_fl_control_2b7::DIRECTIONSTATE_FL_FORWARD
                : Wheel_fl_control_2b7::DIRECTIONSTATE_FL_REVERSE);

  wheel_rl_control_2b8_->set_speed_rl_ctrl(rpm_rl);
  wheel_rl_control_2b8_->set_runningstate_rl(
      fabs(v_rl) > 1e-3 ? Wheel_rl_control_2b8::RUNNINGSTATE_RL_RUNNING
                        : Wheel_rl_control_2b8::RUNNINGSTATE_RL_STOPPED);
  wheel_rl_control_2b8_->set_directionstate_rl(
      v_rl >= 0 ? Wheel_rl_control_2b8::DIRECTIONSTATE_RL_FORWARD
                : Wheel_rl_control_2b8::DIRECTIONSTATE_RL_REVERSE);

  wheel_rr_control_2b9_->set_speed_rr_ctrl(rpm_rr);
  wheel_rr_control_2b9_->set_runningstate_rr(
      fabs(v_rr) > 1e-3 ? Wheel_rr_control_2b9::RUNNINGSTATE_RR_RUNNING
                        : Wheel_rr_control_2b9::RUNNINGSTATE_RR_STOPPED);
  wheel_rr_control_2b9_->set_directionstate_rr(
      v_rr >= 0 ? Wheel_rr_control_2b9::DIRECTIONSTATE_RR_FORWARD
                : Wheel_rr_control_2b9::DIRECTIONSTATE_RR_REVERSE);

  // TODO(All): send brake light command if decelerating

  ADEBUG << "cmd: v=" << linear_vel << " m/s, w=" << angular_vel
         << " rad/s -> RPMs [FR:" << rpm_fr << ", FL:" << rpm_fl
         << ", RR:" << rpm_rr << ", RL:" << rpm_rl << "]";
}

void BxkController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void BxkController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void BxkController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void BxkController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  // TODO(All): need to send directlly, all lights use the same message, and
  // identify by type
  if (command.has_signal() && command.signal().has_turn_signal()) {
    auto signal = command.signal().turn_signal();
    if (signal == common::VehicleSignal::TURN_LEFT) {
      light_control_7ff_->set_light_type(
          Light_control_7ff::LIGHT_TYPE_LEFTTURNLIGHT);
      light_control_7ff_->set_light_switch(Light_control_7ff::LIGHT_SWITCH_ON);
    } else if (signal == common::VehicleSignal::TURN_RIGHT) {
      light_control_7ff_->set_light_type(
          Light_control_7ff::LIGHT_TYPE_RIGHTTURNLIGHT);
      light_control_7ff_->set_light_switch(Light_control_7ff::LIGHT_SWITCH_ON);
    } else {
      // do nothing, fallback to steer or other function
    }
  }
}

void BxkController::ResetProtocol() { message_manager_->ResetSendMessages(); }

bool BxkController::CheckChassisError() {
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  int32_t error_cnt = 0;
  int32_t chassis_error_mask = 0;

  const auto fr_report = chassis_detail.bxk().wheel_fr_status_1b6();
  const auto fl_report = chassis_detail.bxk().wheel_fl_status_1b7();
  const auto rl_report = chassis_detail.bxk().wheel_rl_status_1b8();
  const auto rr_report = chassis_detail.bxk().wheel_rr_status_1b9();

  bool fl_error = fr_report.errorcode_fr() > 0;
  chassis_error_mask |= (fr_report.errorcode_fr() > 0) << (error_cnt++);
  bool fr_error = fl_report.errorcode_fl() > 0;
  chassis_error_mask |= (fl_report.errorcode_fl() > 0) << (error_cnt++);
  bool rl_error = rl_report.errorcode_rl() > 0;
  chassis_error_mask |= (rl_report.errorcode_rl() > 0) << (error_cnt++);
  bool rr_error = rr_report.errorcode_rr() > 0;
  chassis_error_mask |= (rr_report.errorcode_rr() > 0) << (error_cnt++);

  set_chassis_error_mask(chassis_error_mask);
  if (fl_error || fr_error || rl_error || rr_error) {
    AERROR_EVERY(100) << "Chassis error detected: "
                      << "fl_error: " << fl_report.errorcode_fl()
                      << ", fr_error: " << fr_report.errorcode_fr()
                      << ", rl_error: " << rl_report.errorcode_rl()
                      << ", rr_error: " << rr_report.errorcode_rr()
                      << ", total error count: " << error_cnt
                      << ", chassis_error_mask: " << chassis_error_mask;
    return true;
  }

  return false;
}

void BxkController::SecurityDogThreadFunc() {
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
      AERROR << "Too much time consumption in BxkController looping process:"
             << elapsed.count();
    }
  }
}

bool BxkController::CheckResponse(const int32_t flags, bool need_wait) {
  int32_t retry_num = 20;
  ChassisDetail chassis_detail;
  bool is_vcu_online = false;

  do {
    if (message_manager_->GetSensorData(&chassis_detail) != ErrorCode::OK) {
      AERROR_EVERY(100) << "get chassis detail failed.";
      return false;
    }

    bool check_ok = true;
    const auto fr_report = chassis_detail.bxk().wheel_fr_status_1b6();
    const auto fl_report = chassis_detail.bxk().wheel_fl_status_1b7();
    const auto rl_report = chassis_detail.bxk().wheel_rl_status_1b8();
    const auto rr_report = chassis_detail.bxk().wheel_rr_status_1b9();

    if (flags & CHECK_RESPONSE_SPEED_UNIT_FLAG) {
      // check if 4 motors no error
      is_vcu_online =
          (!(fr_report.errorcode_fr() > 0) && !(fl_report.errorcode_fl() > 0) &&
           !(rl_report.errorcode_rl() > 0) && !(rr_report.errorcode_rr() > 0));

      check_ok = check_ok && is_vcu_online;
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

  AINFO << "check_response fail: is_vcu_online:" << is_vcu_online;
  return false;
}

void BxkController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t BxkController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode BxkController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void BxkController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace bxk
}  // namespace canbus
}  // namespace apollo
