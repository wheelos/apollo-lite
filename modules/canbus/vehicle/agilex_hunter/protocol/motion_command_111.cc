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

#include "modules/canbus/vehicle/agilex_hunter/protocol/motion_command_111.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace agilex_hunter {

using ::apollo::drivers::canbus::Byte;

const int32_t Motioncommand111::ID = 0x111;

// public
Motioncommand111::Motioncommand111() { Reset(); }

uint32_t Motioncommand111::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Motioncommand111::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_agilex_hunter()->mutable_motion_command_111()->set_speed_command(speed_command(bytes, length));
  chassis->mutable_agilex_hunter()->mutable_motion_command_111()->set_steering_command(steering_command(bytes, length));
}

void Motioncommand111::UpdateData(uint8_t* data) {
  set_p_speed_command(data, speed_command_);
  set_p_steering_command(data, steering_command_);
}

void Motioncommand111::Reset() {
  // TODO(All) :  you should check this manually
  speed_command_ = 0.0;
  steering_command_ = 0.0;
}

Motioncommand111* Motioncommand111::set_speed_command(
    double speed_command) {
  speed_command_ = speed_command;
  return this;
 }

// config detail: {'bit': 15, 'is_signed_var': True, 'len': 16, 'name': 'Speed_Command', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-4.8|4.8]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
void Motioncommand111::set_p_speed_command(uint8_t* data,
    double speed_command) {
  speed_command = ProtocolData::BoundedValue(-4.8, 4.8, speed_command);
  int x = speed_command / 0.001000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}


Motioncommand111* Motioncommand111::set_steering_command(
    double steering_command) {
  steering_command_ = steering_command;
  return this;
 }

// config detail: {'bit': 63, 'is_signed_var': True, 'len': 16, 'name': 'Steering_Command', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-0.4|0.4]', 'physical_unit': 'rad', 'precision': 0.001, 'type': 'double'}
void Motioncommand111::set_p_steering_command(uint8_t* data,
    double steering_command) {
  steering_command = ProtocolData::BoundedValue(-0.4, 0.4, steering_command);
  int x = steering_command / 0.001000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 8);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 7);
  to_set1.set_value(t, 0, 8);
}


double Motioncommand111::speed_command(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

double Motioncommand111::steering_command(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 8);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo
