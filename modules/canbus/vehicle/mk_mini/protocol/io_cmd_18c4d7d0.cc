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

#include "modules/canbus/vehicle/mk_mini/protocol/io_cmd_18c4d7d0.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace mk_mini {

using ::apollo::drivers::canbus::Byte;

const int32_t Iocmd18c4d7d0::ID = 0x98c4d7d0;

// public
Iocmd18c4d7d0::Iocmd18c4d7d0() { Reset(); }

uint32_t Iocmd18c4d7d0::GetPeriod() const {
  static const uint32_t PERIOD = 50 * 1000;
  return PERIOD;
}

void Iocmd18c4d7d0::UpdateData(uint8_t* data) {
  set_p_io_cmd_discharge(data, io_cmd_discharge_);
  // set alive count before bcc, because bcc is calculated based on all
  set_p_io_cmd_alive_cnt(data, io_cmd_alive_cnt_);
  set_p_io_cmd_check_bcc(data, io_cmd_check_bcc_);
  set_p_io_cmd_speaker(data, io_cmd_speaker_);
  set_p_io_cmd_fog_lamp(data, io_cmd_fog_lamp_);
  set_p_io_cmd_clearance_lamp(data, io_cmd_clearance_lamp_);
  set_p_io_cmd_braking_lamp(data, io_cmd_braking_lamp_);
  set_p_io_cmd_turn_lamp(data, io_cmd_turn_lamp_);
  set_p_io_cmd_upper_beam_headlamp(data, io_cmd_upper_beam_headlamp_);
  set_p_io_cmd_lower_beam_headlamp(data, io_cmd_lower_beam_headlamp_);
  set_p_io_cmd_enable(data, io_cmd_enable_);
}

void Iocmd18c4d7d0::Reset() {
  // TODO(All) :  you should check this manually
  io_cmd_discharge_ = false;
  io_cmd_check_bcc_ = 0;
  io_cmd_alive_cnt_ = 0;
  io_cmd_speaker_ = false;
  io_cmd_fog_lamp_ = false;
  io_cmd_clearance_lamp_ = false;
  io_cmd_braking_lamp_ = false;
  io_cmd_turn_lamp_ = 0;
  io_cmd_upper_beam_headlamp_ = false;
  io_cmd_lower_beam_headlamp_ = false;
  io_cmd_enable_ = false;
}

Iocmd18c4d7d0* Iocmd18c4d7d0::set_io_cmd_discharge(bool io_cmd_discharge) {
  io_cmd_discharge_ = io_cmd_discharge;
  return this;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 1, 'name':
// 'io_cmd_disCharge', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Iocmd18c4d7d0::set_p_io_cmd_discharge(uint8_t* data,
                                           bool io_cmd_discharge) {
  int x = io_cmd_discharge;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 1);
}

Iocmd18c4d7d0* Iocmd18c4d7d0::set_io_cmd_check_bcc(int io_cmd_check_bcc) {
  io_cmd_check_bcc_ = io_cmd_check_bcc;
  return this;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'io_cmd_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Iocmd18c4d7d0::set_p_io_cmd_check_bcc(uint8_t* data,
                                           int io_cmd_check_bcc) {
  uint8_t checksum = 0;
  for (std::size_t i = 0; i < 7; ++i) {
    checksum ^= data[i];
  }

  Byte to_set(data + 7);
  to_set.set_value(checksum, 0, 8);
}

Iocmd18c4d7d0* Iocmd18c4d7d0::set_io_cmd_alive_cnt(int io_cmd_alive_cnt) {
  io_cmd_alive_cnt_ = io_cmd_alive_cnt;
  return this;
}

// config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name':
// 'io_cmd_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Iocmd18c4d7d0::set_p_io_cmd_alive_cnt(uint8_t* data,
                                           int io_cmd_alive_cnt) {
  static uint8_t x = 0;
  x = (x + 1) & 0xF;

  Byte to_set(data + 6);
  to_set.set_value(x, 4, 4);
}

Iocmd18c4d7d0* Iocmd18c4d7d0::set_io_cmd_speaker(bool io_cmd_speaker) {
  io_cmd_speaker_ = io_cmd_speaker;
  return this;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 1, 'name':
// 'io_cmd_speaker', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Iocmd18c4d7d0::set_p_io_cmd_speaker(uint8_t* data, bool io_cmd_speaker) {
  int x = io_cmd_speaker;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 1);
}

Iocmd18c4d7d0* Iocmd18c4d7d0::set_io_cmd_fog_lamp(bool io_cmd_fog_lamp) {
  io_cmd_fog_lamp_ = io_cmd_fog_lamp;
  return this;
}

// config detail: {'bit': 14, 'is_signed_var': False, 'len': 1, 'name':
// 'io_cmd_fog_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Iocmd18c4d7d0::set_p_io_cmd_fog_lamp(uint8_t* data, bool io_cmd_fog_lamp) {
  int x = io_cmd_fog_lamp;

  Byte to_set(data + 1);
  to_set.set_value(x, 6, 1);
}

Iocmd18c4d7d0* Iocmd18c4d7d0::set_io_cmd_clearance_lamp(
    bool io_cmd_clearance_lamp) {
  io_cmd_clearance_lamp_ = io_cmd_clearance_lamp;
  return this;
}

// config detail: {'bit': 13, 'is_signed_var': False, 'len': 1, 'name':
// 'io_cmd_clearance_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Iocmd18c4d7d0::set_p_io_cmd_clearance_lamp(uint8_t* data,
                                                bool io_cmd_clearance_lamp) {
  int x = io_cmd_clearance_lamp;

  Byte to_set(data + 1);
  to_set.set_value(x, 5, 1);
}

Iocmd18c4d7d0* Iocmd18c4d7d0::set_io_cmd_braking_lamp(
    bool io_cmd_braking_lamp) {
  io_cmd_braking_lamp_ = io_cmd_braking_lamp;
  return this;
}

// config detail: {'bit': 12, 'is_signed_var': False, 'len': 1, 'name':
// 'io_cmd_braking_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Iocmd18c4d7d0::set_p_io_cmd_braking_lamp(uint8_t* data,
                                              bool io_cmd_braking_lamp) {
  int x = io_cmd_braking_lamp;

  Byte to_set(data + 1);
  to_set.set_value(x, 4, 1);
}

Iocmd18c4d7d0* Iocmd18c4d7d0::set_io_cmd_turn_lamp(int io_cmd_turn_lamp) {
  io_cmd_turn_lamp_ = io_cmd_turn_lamp;
  return this;
}

// config detail: {'bit': 10, 'is_signed_var': False, 'len': 2, 'name':
// 'io_cmd_turn_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Iocmd18c4d7d0::set_p_io_cmd_turn_lamp(uint8_t* data,
                                           int io_cmd_turn_lamp) {
  io_cmd_turn_lamp = ProtocolData::BoundedValue(0, 0, io_cmd_turn_lamp);
  int x = io_cmd_turn_lamp;

  Byte to_set(data + 1);
  to_set.set_value(x, 2, 2);
}

Iocmd18c4d7d0* Iocmd18c4d7d0::set_io_cmd_upper_beam_headlamp(
    bool io_cmd_upper_beam_headlamp) {
  io_cmd_upper_beam_headlamp_ = io_cmd_upper_beam_headlamp;
  return this;
}

// config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name':
// 'io_cmd_upper_beam_headlamp', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
void Iocmd18c4d7d0::set_p_io_cmd_upper_beam_headlamp(
    uint8_t* data, bool io_cmd_upper_beam_headlamp) {
  int x = io_cmd_upper_beam_headlamp;

  Byte to_set(data + 1);
  to_set.set_value(x, 1, 1);
}

Iocmd18c4d7d0* Iocmd18c4d7d0::set_io_cmd_lower_beam_headlamp(
    bool io_cmd_lower_beam_headlamp) {
  io_cmd_lower_beam_headlamp_ = io_cmd_lower_beam_headlamp;
  return this;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name':
// 'io_cmd_lower_beam_headlamp', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
void Iocmd18c4d7d0::set_p_io_cmd_lower_beam_headlamp(
    uint8_t* data, bool io_cmd_lower_beam_headlamp) {
  int x = io_cmd_lower_beam_headlamp;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 1);
}

Iocmd18c4d7d0* Iocmd18c4d7d0::set_io_cmd_enable(bool io_cmd_enable) {
  io_cmd_enable_ = io_cmd_enable;
  return this;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name':
// 'io_cmd_enable', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Iocmd18c4d7d0::set_p_io_cmd_enable(uint8_t* data, bool io_cmd_enable) {
  int x = io_cmd_enable;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

bool Iocmd18c4d7d0::io_cmd_discharge(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

int Iocmd18c4d7d0::io_cmd_check_bcc(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int Iocmd18c4d7d0::io_cmd_alive_cnt(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

bool Iocmd18c4d7d0::io_cmd_speaker(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

bool Iocmd18c4d7d0::io_cmd_fog_lamp(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

bool Iocmd18c4d7d0::io_cmd_clearance_lamp(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

bool Iocmd18c4d7d0::io_cmd_braking_lamp(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

int Iocmd18c4d7d0::io_cmd_turn_lamp(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 2);

  int ret = x;
  return ret;
}

bool Iocmd18c4d7d0::io_cmd_upper_beam_headlamp(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

bool Iocmd18c4d7d0::io_cmd_lower_beam_headlamp(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

bool Iocmd18c4d7d0::io_cmd_enable(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
