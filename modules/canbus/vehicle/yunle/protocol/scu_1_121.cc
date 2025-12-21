/******************************************************************************
 * Copyright 2025 The WheelOS Team. All Rights Reserved.
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

#include "modules/canbus/vehicle/yunle/protocol/scu_1_121.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace yunle {

using ::apollo::drivers::canbus::Byte;

const int32_t Scu1121::ID = 0x121;

// public
Scu1121::Scu1121() { Reset(); }

uint32_t Scu1121::GetPeriod() const {
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Scu1121::Parse(const std::uint8_t* bytes, int32_t length,
                    ChassisDetail* chassis) const {
  chassis->mutable_yunle()->mutable_scu_1_121()->set_angular_speed_flag(
      angular_speed_flag(bytes, length));
  chassis->mutable_yunle()->mutable_scu_1_121()->set_scu_steering_wheel_angle_r(
      scu_steering_wheel_angle_r(bytes, length));
  chassis->mutable_yunle()->mutable_scu_1_121()->set_scu_steering_wheel_angle_f(
      scu_steering_wheel_angle_f(bytes, length));
  chassis->mutable_yunle()->mutable_scu_1_121()->set_gw_lowbeam_req(
      gw_lowbeam_req(bytes, length));
  chassis->mutable_yunle()->mutable_scu_1_121()->set_gw_position_light_req(
      gw_position_light_req(bytes, length));
  chassis->mutable_yunle()->mutable_scu_1_121()->set_gw_right_turn_light_req(
      gw_right_turn_light_req(bytes, length));
  chassis->mutable_yunle()->mutable_scu_1_121()->set_gw_left_turn_light_req(
      gw_left_turn_light_req(bytes, length));
  chassis->mutable_yunle()->mutable_scu_1_121()->set_scu_brk_en(
      scu_brk_en(bytes, length));
  chassis->mutable_yunle()->mutable_scu_1_121()->set_scu_target_speed(
      scu_target_speed(bytes, length));
  chassis->mutable_yunle()->mutable_scu_1_121()->set_scu_shiftlevel_req(
      scu_shiftlevel_req(bytes, length));
  chassis->mutable_yunle()->mutable_scu_1_121()->set_scu_drive_mode_req(
      scu_drive_mode_req(bytes, length));
}

void Scu1121::UpdateData(uint8_t* data) {
  set_p_angular_speed_flag(data, angular_speed_flag_);
  set_p_scu_steering_wheel_angle_r(data, scu_steering_wheel_angle_r_);
  set_p_scu_steering_wheel_angle_f(data, scu_steering_wheel_angle_f_);
  set_p_gw_lowbeam_req(data, gw_lowbeam_req_);
  set_p_gw_position_light_req(data, gw_position_light_req_);
  set_p_gw_right_turn_light_req(data, gw_right_turn_light_req_);
  set_p_gw_left_turn_light_req(data, gw_left_turn_light_req_);
  set_p_scu_brk_en(data, scu_brk_en_);
  set_p_scu_target_speed(data, scu_target_speed_);
  set_p_scu_shiftlevel_req(data, scu_shiftlevel_req_);
  set_p_scu_drive_mode_req(data, scu_drive_mode_req_);
}

void Scu1121::Reset() {
  // TODO(All) :  you should check this manually
  angular_speed_flag_ = false;
  scu_steering_wheel_angle_r_ = 0;
  scu_steering_wheel_angle_f_ = 0;
  gw_lowbeam_req_ = 0;
  gw_position_light_req_ = 0;
  gw_right_turn_light_req_ = 0;
  gw_left_turn_light_req_ = 0;
  scu_brk_en_ = false;
  scu_target_speed_ = 0.0;
  scu_shiftlevel_req_ = 0;
  scu_drive_mode_req_ = 0;
}

Scu1121* Scu1121::set_angular_speed_flag(bool angular_speed_flag) {
  angular_speed_flag_ = angular_speed_flag;
  return this;
}

// config detail: {'bit': 60, 'description':
// '只针对个别车型使用，不适用所有车型', 'is_signed_var': True, 'len': 1,
// 'name': 'angular_speed_flag', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
void Scu1121::set_p_angular_speed_flag(uint8_t* data, bool angular_speed_flag) {
  int x = angular_speed_flag;

  Byte to_set(data + 7);
  to_set.set_value(x, 4, 1);
}

Scu1121* Scu1121::set_scu_steering_wheel_angle_r(
    int scu_steering_wheel_angle_r) {
  scu_steering_wheel_angle_r_ = scu_steering_wheel_angle_r;
  return this;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
// 'SCU_Steering_Wheel_Angle_R', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
void Scu1121::set_p_scu_steering_wheel_angle_r(uint8_t* data,
                                               int scu_steering_wheel_angle_r) {
  scu_steering_wheel_angle_r =
      ProtocolData::BoundedValue(0, 255, scu_steering_wheel_angle_r);
  int x = scu_steering_wheel_angle_r;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 8);
}

Scu1121* Scu1121::set_scu_steering_wheel_angle_f(
    int scu_steering_wheel_angle_f) {
  scu_steering_wheel_angle_f_ = scu_steering_wheel_angle_f;
  return this;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
// 'SCU_Steering_Wheel_Angle_F', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
void Scu1121::set_p_scu_steering_wheel_angle_f(uint8_t* data,
                                               int scu_steering_wheel_angle_f) {
  scu_steering_wheel_angle_f =
      ProtocolData::BoundedValue(0, 255, scu_steering_wheel_angle_f);
  int x = scu_steering_wheel_angle_f;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}

Scu1121* Scu1121::set_gw_lowbeam_req(int gw_lowbeam_req) {
  gw_lowbeam_req_ = gw_lowbeam_req;
  return this;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 2, 'name':
// 'GW_LowBeam_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Scu1121::set_p_gw_lowbeam_req(uint8_t* data, int gw_lowbeam_req) {
  gw_lowbeam_req = ProtocolData::BoundedValue(0, 3, gw_lowbeam_req);
  int x = gw_lowbeam_req;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 2);
}

Scu1121* Scu1121::set_gw_position_light_req(int gw_position_light_req) {
  gw_position_light_req_ = gw_position_light_req;
  return this;
}

// config detail: {'bit': 46, 'is_signed_var': False, 'len': 2, 'name':
// 'GW_Position_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Scu1121::set_p_gw_position_light_req(uint8_t* data,
                                          int gw_position_light_req) {
  gw_position_light_req =
      ProtocolData::BoundedValue(0, 3, gw_position_light_req);
  int x = gw_position_light_req;

  Byte to_set(data + 5);
  to_set.set_value(x, 6, 2);
}

Scu1121* Scu1121::set_gw_right_turn_light_req(int gw_right_turn_light_req) {
  gw_right_turn_light_req_ = gw_right_turn_light_req;
  return this;
}

// config detail: {'bit': 42, 'is_signed_var': False, 'len': 2, 'name':
// 'GW_Right_Turn_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Scu1121::set_p_gw_right_turn_light_req(uint8_t* data,
                                            int gw_right_turn_light_req) {
  gw_right_turn_light_req =
      ProtocolData::BoundedValue(0, 3, gw_right_turn_light_req);
  int x = gw_right_turn_light_req;

  Byte to_set(data + 5);
  to_set.set_value(x, 2, 2);
}

Scu1121* Scu1121::set_gw_left_turn_light_req(int gw_left_turn_light_req) {
  gw_left_turn_light_req_ = gw_left_turn_light_req;
  return this;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 2, 'name':
// 'GW_Left_Turn_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Scu1121::set_p_gw_left_turn_light_req(uint8_t* data,
                                           int gw_left_turn_light_req) {
  gw_left_turn_light_req =
      ProtocolData::BoundedValue(0, 3, gw_left_turn_light_req);
  int x = gw_left_turn_light_req;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 2);
}

Scu1121* Scu1121::set_scu_brk_en(bool scu_brk_en) {
  scu_brk_en_ = scu_brk_en;
  return this;
}

// config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name':
// 'SCU_Brk_En', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Scu1121::set_p_scu_brk_en(uint8_t* data, bool scu_brk_en) {
  int x = scu_brk_en;

  Byte to_set(data + 4);
  to_set.set_value(x, 1, 1);
}

Scu1121* Scu1121::set_scu_target_speed(double scu_target_speed) {
  scu_target_speed_ = scu_target_speed;
  return this;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 9, 'name':
// 'SCU_Target_Speed', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|51]', 'physical_unit': 'km/h', 'precision': 0.1, 'type': 'double'}
void Scu1121::set_p_scu_target_speed(uint8_t* data, double scu_target_speed) {
  scu_target_speed = ProtocolData::BoundedValue(0.0, 51.0, scu_target_speed);
  int x = scu_target_speed / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 3);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0x1;
  Byte to_set1(data + 4);
  to_set1.set_value(t, 0, 1);
}

Scu1121* Scu1121::set_scu_shiftlevel_req(int scu_shiftlevel_req) {
  scu_shiftlevel_req_ = scu_shiftlevel_req;
  return this;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name':
// 'SCU_ShiftLevel_Req', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Scu1121::set_p_scu_shiftlevel_req(uint8_t* data, int scu_shiftlevel_req) {
  scu_shiftlevel_req = ProtocolData::BoundedValue(0, 3, scu_shiftlevel_req);
  int x = scu_shiftlevel_req;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 2);
}

Scu1121* Scu1121::set_scu_drive_mode_req(int scu_drive_mode_req) {
  scu_drive_mode_req_ = scu_drive_mode_req;
  return this;
}

// config detail: {'bit': 6, 'is_signed_var': False, 'len': 2, 'name':
// 'SCU_Drive_Mode_Req', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Scu1121::set_p_scu_drive_mode_req(uint8_t* data, int scu_drive_mode_req) {
  scu_drive_mode_req = ProtocolData::BoundedValue(0, 3, scu_drive_mode_req);
  int x = scu_drive_mode_req;

  Byte to_set(data + 0);
  to_set.set_value(x, 6, 2);
}

bool Scu1121::angular_speed_flag(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(4, 1);

  x <<= 31;
  x >>= 31;

  bool ret = x;
  return ret;
}

int Scu1121::scu_steering_wheel_angle_r(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int Scu1121::scu_steering_wheel_angle_f(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int Scu1121::gw_lowbeam_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

int Scu1121::gw_position_light_req(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

int Scu1121::gw_right_turn_light_req(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(2, 2);

  int ret = x;
  return ret;
}

int Scu1121::gw_left_turn_light_req(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

bool Scu1121::scu_brk_en(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

double Scu1121::scu_target_speed(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

int Scu1121::scu_shiftlevel_req(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

int Scu1121::scu_drive_mode_req(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}
}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
