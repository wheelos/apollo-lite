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

#include "modules/canbus/vehicle/yunle/protocol/ccu_status_51.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace yunle {

using ::apollo::drivers::canbus::Byte;

Ccustatus51::Ccustatus51() {}
const int32_t Ccustatus51::ID = 0x51;

uint32_t Ccustatus51::GetPeriod() const {
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Ccustatus51::Parse(const std::uint8_t* bytes, int32_t length,
                        ChassisDetail* chassis) const {
  chassis->mutable_yunle()->mutable_ccu_status_51()->set_scu_brake_singal(
      scu_brake_singal(bytes, length));
  chassis->mutable_yunle()->mutable_ccu_status_51()->set_emergency_brake(
      emergency_brake(bytes, length));
  chassis->mutable_yunle()->mutable_ccu_status_51()->set_remote_brake(
      remote_brake(bytes, length));
  chassis->mutable_yunle()->mutable_ccu_status_51()->set_ccu_drive_mode_shift(
      ccu_drive_mode_shift(bytes, length));
  chassis->mutable_yunle()->mutable_ccu_status_51()->set_position_light_sts(
      position_light_sts(bytes, length));
  chassis->mutable_yunle()
      ->mutable_ccu_status_51()
      ->set_steering_wheel_direction(steering_wheel_direction(bytes, length));
  chassis->mutable_yunle()->mutable_ccu_status_51()->set_right_turn_light_sts(
      right_turn_light_sts(bytes, length));
  chassis->mutable_yunle()->mutable_ccu_status_51()->set_lowbeam_sts(
      lowbeam_sts(bytes, length));
  chassis->mutable_yunle()->mutable_ccu_status_51()->set_left_turn_light_sts(
      left_turn_light_sts(bytes, length));
  chassis->mutable_yunle()->mutable_ccu_status_51()->set_ccu_vehicle_speed(
      ccu_vehicle_speed(bytes, length));
  chassis->mutable_yunle()
      ->mutable_ccu_status_51()
      ->set_ccu_steering_wheel_angle(ccu_steering_wheel_angle(bytes, length));
  chassis->mutable_yunle()->mutable_ccu_status_51()->set_ccu_shiftlevel_sts(
      ccu_shiftlevel_sts(bytes, length));
  chassis->mutable_yunle()->mutable_ccu_status_51()->set_ccu_p_sts(
      ccu_p_sts(bytes, length));
  chassis->mutable_yunle()->mutable_ccu_status_51()->set_ccu_ignition_sts(
      ccu_ignition_sts(bytes, length));
  chassis->mutable_yunle()->mutable_ccu_status_51()->set_ccu_drive_mode(
      ccu_drive_mode(bytes, length));
}

// config detail: {'bit': 34, 'description': '自动驾驶模式刹车信号',
// 'is_signed_var': False, 'len': 1, 'name': 'scu_brake_singal', 'offset': 0.0,
// 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'bool'}
bool Ccustatus51::scu_brake_singal(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 33, 'description': '紧急制动按钮信号',
// 'is_signed_var': False, 'len': 1, 'name': 'emergency_brake', 'offset': 0.0,
// 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'bool'}
bool Ccustatus51::emergency_brake(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 32, 'description': '遥控器刹车信号', 'is_signed_var':
// False, 'len': 1, 'name': 'remote_brake', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
bool Ccustatus51::remote_brake(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 5, 'description': '驾驶模式切换按钮', 'is_signed_var':
// False, 'len': 1, 'name': 'ccu_drive_mode_shift', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
// 'type': 'bool'}
bool Ccustatus51::ccu_drive_mode_shift(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 59, 'description': '刹车灯状态', 'is_signed_var':
// False, 'len': 1, 'name': 'position_light_sts', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
// 'type': 'bool'}
bool Ccustatus51::position_light_sts(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 7, 'description': '前轮转向角度方向，左正右负',
// 'is_signed_var': False, 'len': 1, 'name': 'steering_wheel_direction',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'bool'}
bool Ccustatus51::steering_wheel_direction(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 57, 'description': '右转向灯状态', 'is_signed_var':
// False, 'len': 1, 'name': 'right_turn_light_sts', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
// 'type': 'bool'}
bool Ccustatus51::right_turn_light_sts(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 60, 'description': '近光灯状态', 'is_signed_var':
// False, 'len': 1, 'name': 'lowbeam_sts', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
bool Ccustatus51::lowbeam_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 56, 'description': '左转向灯状态', 'is_signed_var':
// False, 'len': 1, 'name': 'left_turn_light_sts', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
// 'type': 'bool'}
bool Ccustatus51::left_turn_light_sts(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 20, 'description': '底盘速度', 'is_signed_var': False,
// 'len': 9, 'name': 'ccu_vehicle_speed', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|51]', 'physical_unit': 'km/h', 'precision': 0.1,
// 'type': 'double'}
double Ccustatus51::ccu_vehicle_speed(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 5);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 8, 'description': '前轮转向角度，120对应实际转角27°',
// 'is_signed_var': False, 'len': 12, 'name': 'ccu_steering_wheel_angle',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|120]',
// 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double Ccustatus51::ccu_steering_wheel_angle(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'description': '底盘档位状态', 'is_signed_var':
// False, 'len': 2, 'name': 'ccu_shiftlevel_sts', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0,
// 'type': 'int'}
int Ccustatus51::ccu_shiftlevel_sts(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 2, 'description': '底盘刹车状态', 'is_signed_var':
// False, 'len': 1, 'name': 'ccu_p_sts', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
bool Ccustatus51::ccu_p_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 3, 'description': 'VCU点火信号状态', 'is_signed_var':
// False, 'len': 2, 'name': 'ccu_ignition_sts', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Ccustatus51::ccu_ignition_sts(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 29, 'description': '底盘驾驶模式', 'is_signed_var':
// False, 'len': 3, 'name': 'ccu_drive_mode', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Ccustatus51::ccu_drive_mode(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 3);

  int ret = x;
  return ret;
}
}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
