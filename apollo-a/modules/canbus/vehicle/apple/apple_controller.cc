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

#include "modules/canbus/vehicle/apple/apple_controller.h"

#include "modules/common_msgs/basic_msgs/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "modules/canbus/vehicle/apple/apple_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "cyber/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace apple {

using ::apollo::drivers::canbus::ProtocolData;
using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}

ErrorCode AppleController::Init(
	const VehicleParameter& params,
	CanSender<::apollo::canbus::ChassisDetail> *const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail> *const message_manager) {
  if (is_initialized_) {
    AINFO << "AppleController has already been initiated.";
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



  // need sleep to ensure all messages received
  AINFO << "AppleController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

AppleController::~AppleController() {}

bool AppleController::Start() {
  if (!is_initialized_) {
    AERROR << "AppleController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void AppleController::Stop() {
  if (!is_initialized_) {
    AERROR << "AppleController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "AppleController stopped.";
  }
}

Chassis AppleController::chassis() {
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
  // 检查右前轮状态报文 (ID: 438)
  if (chassis_detail.has_apple() && chassis_detail.apple().has_wheel_fr_status_438()) {
    auto fr_report = chassis_detail.apple().wheel_fr_status_438();
    if (fr_report.has_speed_fr()) {
        total_speed += fr_report.speed_fr();
        valid_wheel_count++;
    }
}

  // 检查左前轮状态报文 (ID: 439)  
  if (chassis_detail.has_apple() && chassis_detail.apple().has_wheel_fl_status_439()) {
    auto fl_report = chassis_detail.apple().wheel_fl_status_439();
    if (fl_report.has_speed_fl()) {
        total_speed += fl_report.speed_fl();
        valid_wheel_count++;
    }
}

  // 检查左后轮状态报文 (ID: 440)
  if (chassis_detail.has_apple() && chassis_detail.apple().has_wheel_rl_status_440()) {
    auto rl_report = chassis_detail.apple().wheel_rl_status_440();
    if (rl_report.has_speed_rl()) {
        total_speed += rl_report.speed_rl();
        valid_wheel_count++;
    }
}

  // 检查右后轮状态报文 (ID: 441)
  if (chassis_detail.has_apple() && chassis_detail.apple().has_wheel_rr_status_441()) {
    auto rr_report = chassis_detail.apple().wheel_rr_status_441();
    if (rr_report.has_speed_rr()) {
        total_speed += rr_report.speed_rr();
        valid_wheel_count++;
    }
}

  return chassis_;
}

void AppleController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode AppleController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;
  // 右前轮控制
  wheel_fr_control_->set_speed_fr_ctrl(0); // 设置目标速度
  wheel_fr_control_->set_running_state_fr(2); // 设置为运行状态
  wheel_fr_control_->set_direction_state_fr(1); // 设置为前进
  
  // 左前轮控制
  wheel_fl_control_->set_speed_fl_ctrl(0);
  wheel_fl_control_->set_running_state_fl(2);
  wheel_fl_control_->set_direction_state_fl(1);
  
  // 左后轮控制
  wheel_rl_control_->set_speed_rl_ctrl(0);
  wheel_rl_control_->set_running_state_rl(2);
  wheel_rl_control_->set_direction_state_rl(1);
  
  // 右后轮控制
  wheel_rr_control_->set_speed_rr_ctrl(0);
  wheel_rr_control_->set_running_state_rr(2);
  wheel_rr_control_->set_direction_state_rr(1);

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
  */
}

ErrorCode AppleController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode AppleController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode.";
    return ErrorCode::OK;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_disable();
  throttle_62_->set_disable();
  steering_64_->set_enable();

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_STEER_ONLY);
  AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
  return ErrorCode::OK;
  */
}

ErrorCode AppleController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_enable();
  throttle_62_->set_enable();
  steering_64_->set_disable();

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_SPEED_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_SPEED_ONLY);
  AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
  return ErrorCode::OK;
  */
}

// NEUTRAL, REVERSE, DRIVE
void AppleController::Gear(Chassis::GearPosition gear_position) {
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
void AppleController::Brake(double pedal) {
  // double real_value = vehicle_params_.max_acceleration() * acceleration / 100;
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
void AppleController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  throttle_62_->set_pedal(pedal);
  */
}
void ApolloController::Speed(double linear_vel, double angular_vel) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "Skip wheel speed control: not in auto drive mode.";
    return;
  }

  // === 车辆参数（实际数值需要你填） ===
  const double wheel_radius = 0.23;  // m
  const double wheel_base   = 1.16;  // m (L)
  const double wheel_track_f = 0.86; // m (Wf)
  const double wheel_track_r = 0.67; // m (Wr)

  // === 转换函数 m/s -> RPM ===
  auto mps_to_rpm = [&](double v) {
    return (v / (2 * M_PI * wheel_radius)) * 60.0;
  };

  // === 四轮速度计算 ===
  double v_fr, v_fl, v_rr, v_rl;

  if (fabs(angular_vel) < 1e-6) {
    // 直行，四轮速度相同
    v_fr = v_fl = v_rr = v_rl = linear_vel;
  } else {
    double Ric = linear_vel / angular_vel; // 瞬时转向半径

    v_fr = angular_vel * sqrt(pow(Ric + wheel_track_f/2.0, 2) + pow(wheel_base/2.0, 2));
    v_fl = angular_vel * sqrt(pow(Ric - wheel_track_f/2.0, 2) + pow(wheel_base/2.0, 2));
    v_rr = angular_vel * sqrt(pow(Ric + wheel_track_r/2.0, 2) + pow(wheel_base/2.0, 2));
    v_rl = angular_vel * sqrt(pow(Ric - wheel_track_r/2.0, 2) + pow(wheel_base/2.0, 2));
  }

  // === 转换成 RPM ===
  double rpm_fr = mps_to_rpm(v_fr);
  double rpm_fl = mps_to_rpm(v_fl);
  double rpm_rr = mps_to_rpm(v_rr);
  double rpm_rl = mps_to_rpm(v_rl);

  // === 下发到报文 ===
  wheel_fr_control_694_->set_speed_fr_ctrl(rpm_fr);
  wheel_fr_control_694_->set_runningstate_fr(fabs(v_fr) > 1e-3 ? 2 : 0);
  wheel_fr_control_694_->set_directionstate_fr(v_fr >= 0 ? 1 : 0);

  wheel_fl_control_695_->set_speed_fl_ctrl(rpm_fl);
  wheel_fl_control_695_->set_runningstate_fl(fabs(v_fl) > 1e-3 ? 2 : 0);
  wheel_fl_control_695_->set_directionstate_fl(v_fl >= 0 ? 1 : 0);

  wheel_rr_control_697_->set_speed_rr_ctrl(rpm_rr);
  wheel_rr_control_697_->set_runningstate_rr(fabs(v_rr) > 1e-3 ? 2 : 0);
  wheel_rr_control_697_->set_directionstate_rr(v_rr >= 0 ? 1 : 0);

  wheel_rl_control_696_->set_speed_rl_ctrl(rpm_rl);
  wheel_rl_control_696_->set_runningstate_rl(fabs(v_rl) > 1e-3 ? 2 : 0);
  wheel_rl_control_696_->set_directionstate_rl(v_rl >= 0 ? 1 : 0);

  AINFO << "Cmd: v=" << linear_vel << " m/s, w=" << angular_vel
        << " rad/s -> RPMs [FR:" << rpm_fr << ", FL:" << rpm_fl
        << ", RR:" << rpm_rr << ", RL:" << rpm_rl << "]";
}

// confirm the car is driven by acceleration command or drive/brake pedal
// drive with acceleration/deceleration
// acc:-7.0 ~ 5.0, unit:m/s^2
void AppleController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
}

// apple default, +470 ~ -470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with angle
// angle:-99.99~0.00~99.99, unit:, left:+, right:-
void AppleController::Steer(double angle) {
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
void AppleController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  const double real_angle_spd = ProtocolData<::apollo::canbus::ChassisDetail>::BoundedValue(
      vehicle_params_.min_steer_angle_rate(), vehicle_params_.max_steer_angle_rate(),
      vehicle_params_.max_steer_angle_rate() * angle_spd / 100.0);
  steering_64_->set_steering_angle(real_angle)
      ->set_steering_angle_speed(real_angle_spd);
  */
}

void AppleController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void AppleController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void AppleController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void AppleController::SetTurningSignal(const ControlCommand& command) {
   // Set Turn Signal
   auto signal = command.signal().turn_signal();
  // 1. 转向灯
  if (signal == common::VehicleSignal::TURN_LEFT) {
    light_control_->set_light_type(Light_Control::Light_Type::LEFT_TURN_LIGHT);
    light_control_->set_light_switch(Light_Control::Light_Switch::ON);
  } else if (signal == common::VehicleSignal::TURN_RIGHT) {
    light_control_->set_light_type(Light_Control::Light_Type::RIGHT_TURN_LIGHT);
    light_control_->set_light_switch(Light_Control::Light_Switch::ON);
  } else {
    // 没有转向信号时关闭转向灯
    light_control_->set_light_switch(Light_Control::Light_Switch::OFF);
  }

  // 2. 刹车灯
  if (command.brake()) {
    light_control_->set_light_type(Light_Control::Light_Type::BRAKE_LIGHT);
    light_control_->set_light_switch(Light_Control::Light_Switch::ON);
  } else {
    light_control_->set_light_type(Light_Control::Light_Type::BRAKE_LIGHT);
    light_control_->set_light_switch(Light_Control::Light_Switch::OFF);
  }
}


void AppleController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool AppleController::CheckChassisError() {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
  return false;
}

void AppleController::SecurityDogThreadFunc() {
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
      AERROR
          << "Too much time consumption in AppleController looping process:"
          << elapsed.count();
    }
  }
}

bool AppleController::CheckResponse(const int32_t flags, bool need_wait) {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
   int32_t retry_num = 20;
  ChassisDetail chassis_detail;
  bool is_vcu_online = false;

  do {
    if (message_manager_->GetSensorData(&chassis_detail) != ErrorCode::OK) {
      AERROR_EVERY(100) << "get chassis detail failed.";
      return false;
    }

    bool check_ok = true;

    if (flags & CHECK_RESPONSE_SPEED_UNIT_FLAG) {
      // 判断 VCU 四个轮子的运行状态是否在线（假设 1 表示在线）
      is_vcu_online =
          chassis_detail.vcu().running_state_fr() == 1 &&
          chassis_detail.vcu().running_state_fl() == 1 &&
          chassis_detail.vcu().running_state_rl() == 1 &&
          chassis_detail.vcu().running_state_rr() == 1;

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

void AppleController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t AppleController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode AppleController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void AppleController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace apple
}  // namespace canbus
}  // namespace apollo
