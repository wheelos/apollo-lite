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

#include "modules/canbus/vehicle/bxk/protocol/wheel_fr_control_2b6.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace bxk {

using ::apollo::drivers::canbus::Byte;

const int32_t Wheelfrcontrol2b6::ID = 0x2B6;

// public
Wheelfrcontrol2b6::Wheelfrcontrol2b6() { Reset(); }

uint32_t Wheelfrcontrol2b6::GetPeriod() const {
  static const uint32_t PERIOD = 0 * 1000;
  return PERIOD;
}

void Wheelfrcontrol2b6::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_bxk()->mutable_wheel_fr_control_2b6()->set_speed_fr_ctrl(speed_fr_ctrl(bytes, length));
  chassis->mutable_bxk()->mutable_wheel_fr_control_2b6()->set_runningstate_fr(runningstate_fr(bytes, length));
  chassis->mutable_bxk()->mutable_wheel_fr_control_2b6()->set_directionstate_fr(directionstate_fr(bytes, length));
}

void Wheelfrcontrol2b6::UpdateData(uint8_t* data) {
  set_p_speed_fr_ctrl(data, speed_fr_ctrl_);
  set_p_runningstate_fr(data, runningstate_fr_);
  set_p_directionstate_fr(data, directionstate_fr_);
}

void Wheelfrcontrol2b6::Reset() {
  // TODO(All) :  you should check this manually
  speed_fr_ctrl_ = 0;
  runningstate_fr_ = Wheel_fr_control_2b6::RUNNINGSTATE_FR_STOPPED;
  directionstate_fr_ = Wheel_fr_control_2b6::DIRECTIONSTATE_FR_REVERSE;
}

Wheelfrcontrol2b6* Wheelfrcontrol2b6::set_speed_fr_ctrl(
    int speed_fr_ctrl) {
  speed_fr_ctrl_ = speed_fr_ctrl;
  return this;
 }

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 32, 'name': 'Speed_fr_Ctrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1000]', 'physical_unit': 'RPM', 'precision': 1.0, 'type': 'int'}
void Wheelfrcontrol2b6::set_p_speed_fr_ctrl(uint8_t* data,
    int speed_fr_ctrl) {
  speed_fr_ctrl = ProtocolData::BoundedValue(0, 1000, speed_fr_ctrl);
  int x = speed_fr_ctrl;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 0);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set2(data + 2);
  to_set2.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set3(data + 3);
  to_set3.set_value(t, 0, 8);
}


Wheelfrcontrol2b6* Wheelfrcontrol2b6::set_runningstate_fr(
    Wheel_fr_control_2b6::Runningstate_frType runningstate_fr) {
  runningstate_fr_ = runningstate_fr;
  return this;
 }

// config detail: {'bit': 32, 'enum': {0: 'RUNNINGSTATE_FR_STOPPED', 2: 'RUNNINGSTATE_FR_RUNNING'}, 'is_signed_var': False, 'len': 8, 'name': 'RunningState_fr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Wheelfrcontrol2b6::set_p_runningstate_fr(uint8_t* data,
    Wheel_fr_control_2b6::Runningstate_frType runningstate_fr) {
  int x = runningstate_fr;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 8);
}


Wheelfrcontrol2b6* Wheelfrcontrol2b6::set_directionstate_fr(
    Wheel_fr_control_2b6::Directionstate_frType directionstate_fr) {
  directionstate_fr_ = directionstate_fr;
  return this;
 }

// config detail: {'bit': 40, 'enum': {0: 'DIRECTIONSTATE_FR_REVERSE', 1: 'DIRECTIONSTATE_FR_FORWARD'}, 'is_signed_var': False, 'len': 8, 'name': 'DirectionState_fr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Wheelfrcontrol2b6::set_p_directionstate_fr(uint8_t* data,
    Wheel_fr_control_2b6::Directionstate_frType directionstate_fr) {
  int x = directionstate_fr;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 8);
}


int Wheelfrcontrol2b6::speed_fr_ctrl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 1);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 0);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 0;
  x >>= 0;

  int ret = x;
  return ret;
}

Wheel_fr_control_2b6::Runningstate_frType Wheelfrcontrol2b6::runningstate_fr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Wheel_fr_control_2b6::Runningstate_frType ret =  static_cast<Wheel_fr_control_2b6::Runningstate_frType>(x);
  return ret;
}

Wheel_fr_control_2b6::Directionstate_frType Wheelfrcontrol2b6::directionstate_fr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Wheel_fr_control_2b6::Directionstate_frType ret =  static_cast<Wheel_fr_control_2b6::Directionstate_frType>(x);
  return ret;
}
}  // namespace bxk
}  // namespace canbus
}  // namespace apollo
