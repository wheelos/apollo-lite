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

#include "modules/canbus/vehicle/mk_mini/protocol/ctrl_cmd_18c4d2d0.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace mk_mini {

using ::apollo::drivers::canbus::Byte;

const int32_t Ctrlcmd18c4d2d0::ID = 0x98c4d2d0;

// public
Ctrlcmd18c4d2d0::Ctrlcmd18c4d2d0() { Reset(); }

uint32_t Ctrlcmd18c4d2d0::GetPeriod() const {
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Ctrlcmd18c4d2d0::UpdateData(uint8_t* data) {
  set_p_ctrl_cmd_brake(data, ctrl_cmd_brake_);
  set_p_ctrl_cmd_steering(data, ctrl_cmd_steering_);
  set_p_ctrl_cmd_velocity(data, ctrl_cmd_velocity_);
  set_p_ctrl_cmd_gear(data, ctrl_cmd_gear_);

  set_p_ctrl_cmd_alive_cnt(data, ctrl_cmd_alive_cnt_);
  set_p_ctrl_cmd_check_bcc(data, ctrl_cmd_check_bcc_);
}

void Ctrlcmd18c4d2d0::Reset() {
  // TODO(All) :  you should check this manually
  ctrl_cmd_check_bcc_ = 0;
  ctrl_cmd_alive_cnt_ = 0;
  ctrl_cmd_brake_ = 0;
  ctrl_cmd_steering_ = 0.0;
  ctrl_cmd_velocity_ = 0.0;
  ctrl_cmd_gear_ = 0;
}

Ctrlcmd18c4d2d0* Ctrlcmd18c4d2d0::set_ctrl_cmd_check_bcc(
    int ctrl_cmd_check_bcc) {
  ctrl_cmd_check_bcc_ = ctrl_cmd_check_bcc;
  return this;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'ctrl_cmd_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Ctrlcmd18c4d2d0::set_p_ctrl_cmd_check_bcc(uint8_t* data,
                                               int ctrl_cmd_check_bcc) {
  uint8_t checksum = 0;
  for (std::size_t i = 0; i < 7; ++i) {
    checksum ^= data[i];
  }

  Byte to_set(data + 7);
  to_set.set_value(checksum, 0, 8);
}

Ctrlcmd18c4d2d0* Ctrlcmd18c4d2d0::set_ctrl_cmd_alive_cnt(
    int ctrl_cmd_alive_cnt) {
  ctrl_cmd_alive_cnt_ = ctrl_cmd_alive_cnt;
  return this;
}

// config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name':
// 'ctrl_cmd_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Ctrlcmd18c4d2d0::set_p_ctrl_cmd_alive_cnt(uint8_t* data,
                                               int ctrl_cmd_alive_cnt) {
  static uint8_t x = 0;
  x = (x + 1) & 0xF;

  Byte to_set(data + 6);
  to_set.set_value(x, 4, 4);
}

Ctrlcmd18c4d2d0* Ctrlcmd18c4d2d0::set_ctrl_cmd_brake(int ctrl_cmd_brake) {
  ctrl_cmd_brake_ = ctrl_cmd_brake;
  return this;
}

// config detail: {'bit': 36, 'is_signed_var': False, 'len': 8, 'name':
// 'ctrl_cmd_Brake', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|127]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Ctrlcmd18c4d2d0::set_p_ctrl_cmd_brake(uint8_t* data, int ctrl_cmd_brake) {
  ctrl_cmd_brake = ProtocolData::BoundedValue(0, 127, ctrl_cmd_brake);
  int x = ctrl_cmd_brake;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 4);
}

Ctrlcmd18c4d2d0* Ctrlcmd18c4d2d0::set_ctrl_cmd_steering(
    double ctrl_cmd_steering) {
  ctrl_cmd_steering_ = ctrl_cmd_steering;
  return this;
}

// config detail: {'bit': 20, 'is_signed_var': True, 'len': 16, 'name':
// 'ctrl_cmd_steering', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 0.01, 'type': 'double'}
void Ctrlcmd18c4d2d0::set_p_ctrl_cmd_steering(uint8_t* data,
                                              double ctrl_cmd_steering) {
  ctrl_cmd_steering =
      ProtocolData::BoundedValue(-32.0, 32.0, ctrl_cmd_steering);
  int x = ctrl_cmd_steering / 0.010000;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xF;
  Byte to_set2(data + 4);
  to_set2.set_value(t, 0, 4);
}

Ctrlcmd18c4d2d0* Ctrlcmd18c4d2d0::set_ctrl_cmd_velocity(
    double ctrl_cmd_velocity) {
  ctrl_cmd_velocity_ = ctrl_cmd_velocity;
  return this;
}

// config detail: {'bit': 4, 'is_signed_var': False, 'len': 16, 'name':
// 'ctrl_cmd_velocity', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 0.001, 'type': 'double'}
void Ctrlcmd18c4d2d0::set_p_ctrl_cmd_velocity(uint8_t* data,
                                              double ctrl_cmd_velocity) {
  ctrl_cmd_velocity = ProtocolData::BoundedValue(0.0, 2.5, ctrl_cmd_velocity);
  int x = ctrl_cmd_velocity / 0.001000;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 0);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xF;
  Byte to_set2(data + 2);
  to_set2.set_value(t, 0, 4);
}

Ctrlcmd18c4d2d0* Ctrlcmd18c4d2d0::set_ctrl_cmd_gear(int ctrl_cmd_gear) {
  ctrl_cmd_gear_ = ctrl_cmd_gear;
  return this;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 4, 'name':
// 'ctrl_cmd_gear', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Ctrlcmd18c4d2d0::set_p_ctrl_cmd_gear(uint8_t* data, int ctrl_cmd_gear) {
  ctrl_cmd_gear = ProtocolData::BoundedValue(0, 4, ctrl_cmd_gear);
  int x = ctrl_cmd_gear;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 4);
}

int Ctrlcmd18c4d2d0::ctrl_cmd_check_bcc(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int Ctrlcmd18c4d2d0::ctrl_cmd_alive_cnt(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

int Ctrlcmd18c4d2d0::ctrl_cmd_brake(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x;
  return ret;
}

double Ctrlcmd18c4d2d0::ctrl_cmd_steering(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 2);
  t = t2.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

double Ctrlcmd18c4d2d0::ctrl_cmd_velocity(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 0);
  t = t2.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

int Ctrlcmd18c4d2d0::ctrl_cmd_gear(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}
}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
