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

#include "modules/canbus/vehicle/yunle/protocol/scu_tq_123.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace yunle {

using ::apollo::drivers::canbus::Byte;

const int32_t Scutq123::ID = 0x123;

// public
Scutq123::Scutq123() { Reset(); }

uint32_t Scutq123::GetPeriod() const {
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Scutq123::Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const {
  chassis->mutable_yunle()->mutable_scu_tq_123()->set_torque_cmd_rear_r(
      torque_cmd_rear_r(bytes, length));
  chassis->mutable_yunle()->mutable_scu_tq_123()->set_torque_cmd_rear_l(
      torque_cmd_rear_l(bytes, length));
  chassis->mutable_yunle()->mutable_scu_tq_123()->set_torque_cmd_forward_r(
      torque_cmd_forward_r(bytes, length));
  chassis->mutable_yunle()->mutable_scu_tq_123()->set_torque_cmd_forward_l(
      torque_cmd_forward_l(bytes, length));
}

void Scutq123::UpdateData(uint8_t* data) {
  set_p_torque_cmd_rear_r(data, torque_cmd_rear_r_);
  set_p_torque_cmd_rear_l(data, torque_cmd_rear_l_);
  set_p_torque_cmd_forward_r(data, torque_cmd_forward_r_);
  set_p_torque_cmd_forward_l(data, torque_cmd_forward_l_);
}

void Scutq123::Reset() {
  // TODO(All) :  you should check this manually
  torque_cmd_rear_r_ = 0.0;
  torque_cmd_rear_l_ = 0.0;
  torque_cmd_forward_r_ = 0.0;
  torque_cmd_forward_l_ = 0.0;
}

Scutq123* Scutq123::set_torque_cmd_rear_r(double torque_cmd_rear_r) {
  torque_cmd_rear_r_ = torque_cmd_rear_r;
  return this;
}

// config detail: {'bit': 48, 'description': 'Torque available calculated by
// inverterTorque available calculated by inverter', 'is_signed_var': True,
// 'len': 16, 'name': 'Torque_cmd_Rear_R', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': 'Nm', 'precision': 0.1, 'type':
// 'double'}
void Scutq123::set_p_torque_cmd_rear_r(uint8_t* data,
                                       double torque_cmd_rear_r) {
  torque_cmd_rear_r = ProtocolData::BoundedValue(0.0, 0.0, torque_cmd_rear_r);
  int x = torque_cmd_rear_r / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 6);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 7);
  to_set1.set_value(t, 0, 8);
}

Scutq123* Scutq123::set_torque_cmd_rear_l(double torque_cmd_rear_l) {
  torque_cmd_rear_l_ = torque_cmd_rear_l;
  return this;
}

// config detail: {'bit': 32, 'is_signed_var': True, 'len': 16, 'name':
// 'Torque_cmd_Rear_L', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
void Scutq123::set_p_torque_cmd_rear_l(uint8_t* data,
                                       double torque_cmd_rear_l) {
  torque_cmd_rear_l = ProtocolData::BoundedValue(0.0, 0.0, torque_cmd_rear_l);
  int x = torque_cmd_rear_l / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 8);
}

Scutq123* Scutq123::set_torque_cmd_forward_r(double torque_cmd_forward_r) {
  torque_cmd_forward_r_ = torque_cmd_forward_r;
  return this;
}

// config detail: {'bit': 16, 'description': 'Torque available calculated by
// inverterTorque available calculated by inverter', 'is_signed_var': True,
// 'len': 16, 'name': 'Torque_cmd_Forward_R', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': 'Nm', 'precision': 0.1, 'type':
// 'double'}
void Scutq123::set_p_torque_cmd_forward_r(uint8_t* data,
                                          double torque_cmd_forward_r) {
  torque_cmd_forward_r =
      ProtocolData::BoundedValue(0.0, 0.0, torque_cmd_forward_r);
  int x = torque_cmd_forward_r / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}

Scutq123* Scutq123::set_torque_cmd_forward_l(double torque_cmd_forward_l) {
  torque_cmd_forward_l_ = torque_cmd_forward_l;
  return this;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name':
// 'Torque_cmd_Forward_L', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
void Scutq123::set_p_torque_cmd_forward_l(uint8_t* data,
                                          double torque_cmd_forward_l) {
  torque_cmd_forward_l =
      ProtocolData::BoundedValue(0.0, 0.0, torque_cmd_forward_l);
  int x = torque_cmd_forward_l / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 0);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}

double Scutq123::torque_cmd_rear_r(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

double Scutq123::torque_cmd_rear_l(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

double Scutq123::torque_cmd_forward_r(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

double Scutq123::torque_cmd_forward_l(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}
}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
