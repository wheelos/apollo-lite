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

#include "modules/canbus/vehicle/mk_mini/protocol/sensor_reset_18ffffff.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace mk_mini {

using ::apollo::drivers::canbus::Byte;

const int32_t Sensorreset18ffffff::ID = 0x98ffffff;

// public
Sensorreset18ffffff::Sensorreset18ffffff() { Reset(); }

uint32_t Sensorreset18ffffff::GetPeriod() const {
  static const uint32_t PERIOD = 0 * 1000;
  return PERIOD;
}

void Sensorreset18ffffff::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_mk_mini()->mutable_sensor_reset_18ffffff()->set_close_candiag(close_candiag(bytes, length));
  chassis->mutable_mk_mini()->mutable_sensor_reset_18ffffff()->set_brake_reset(brake_reset(bytes, length));
  chassis->mutable_mk_mini()->mutable_sensor_reset_18ffffff()->set_steer_reset(steer_reset(bytes, length));
}

void Sensorreset18ffffff::UpdateData(uint8_t* data) {
  set_p_close_candiag(data, close_candiag_);
  set_p_brake_reset(data, brake_reset_);
  set_p_steer_reset(data, steer_reset_);
}

void Sensorreset18ffffff::Reset() {
  // TODO(All) :  you should check this manually
  close_candiag_ = 0;
  brake_reset_ = false;
  steer_reset_ = false;
}

Sensorreset18ffffff* Sensorreset18ffffff::set_close_candiag(
    int close_candiag) {
  close_candiag_ = close_candiag;
  return this;
 }

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'Close_candiag', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-128|127]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Sensorreset18ffffff::set_p_close_candiag(uint8_t* data,
    int close_candiag) {
  close_candiag = ProtocolData::BoundedValue(-128, 127, close_candiag);
  int x = close_candiag;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Sensorreset18ffffff* Sensorreset18ffffff::set_brake_reset(
    bool brake_reset) {
  brake_reset_ = brake_reset;
  return this;
 }

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'Brake_reset', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Sensorreset18ffffff::set_p_brake_reset(uint8_t* data,
    bool brake_reset) {
  int x = brake_reset;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 1);
}


Sensorreset18ffffff* Sensorreset18ffffff::set_steer_reset(
    bool steer_reset) {
  steer_reset_ = steer_reset;
  return this;
 }

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'steer_reset', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Sensorreset18ffffff::set_p_steer_reset(uint8_t* data,
    bool steer_reset) {
  int x = steer_reset;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}


int Sensorreset18ffffff::close_candiag(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

bool Sensorreset18ffffff::brake_reset(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

bool Sensorreset18ffffff::steer_reset(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
