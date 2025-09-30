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

#include "modules/canbus/vehicle/bxk/protocol/wheel_rl_control_2b8.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace bxk {

using ::apollo::drivers::canbus::Byte;

const int32_t Wheelrlcontrol2b8::ID = 0x2B8;

// public
Wheelrlcontrol2b8::Wheelrlcontrol2b8() { Reset(); }

uint32_t Wheelrlcontrol2b8::GetPeriod() const {
  static const uint32_t PERIOD = 0 * 1000;
  return PERIOD;
}

void Wheelrlcontrol2b8::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_bxk()->mutable_wheel_rl_control_2b8()->set_speed_rl_ctrl(speed_rl_ctrl(bytes, length));
  chassis->mutable_bxk()->mutable_wheel_rl_control_2b8()->set_runningstate_rl(runningstate_rl(bytes, length));
  chassis->mutable_bxk()->mutable_wheel_rl_control_2b8()->set_directionstate_rl(directionstate_rl(bytes, length));
}

void Wheelrlcontrol2b8::UpdateData(uint8_t* data) {
  set_p_speed_rl_ctrl(data, speed_rl_ctrl_);
  set_p_runningstate_rl(data, runningstate_rl_);
  set_p_directionstate_rl(data, directionstate_rl_);
}

void Wheelrlcontrol2b8::Reset() {
  // TODO(All) :  you should check this manually
  speed_rl_ctrl_ = 0;
  runningstate_rl_ = Wheel_rl_control_2b8::RUNNINGSTATE_RL_STOPPED;
  directionstate_rl_ = Wheel_rl_control_2b8::DIRECTIONSTATE_RL_REVERSE;
}

Wheelrlcontrol2b8* Wheelrlcontrol2b8::set_speed_rl_ctrl(
    int speed_rl_ctrl) {
  speed_rl_ctrl_ = speed_rl_ctrl;
  return this;
 }

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 32, 'name': 'Speed_rl_Ctrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1000]', 'physical_unit': 'RPM', 'precision': 1.0, 'type': 'int'}
void Wheelrlcontrol2b8::set_p_speed_rl_ctrl(uint8_t* data,
    int speed_rl_ctrl) {
  speed_rl_ctrl = ProtocolData::BoundedValue(0, 1000, speed_rl_ctrl);
  int x = speed_rl_ctrl;
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


Wheelrlcontrol2b8* Wheelrlcontrol2b8::set_runningstate_rl(
    Wheel_rl_control_2b8::Runningstate_rlType runningstate_rl) {
  runningstate_rl_ = runningstate_rl;
  return this;
 }

// config detail: {'bit': 32, 'enum': {0: 'RUNNINGSTATE_RL_STOPPED', 2: 'RUNNINGSTATE_RL_RUNNING'}, 'is_signed_var': False, 'len': 8, 'name': 'RunningState_rl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Wheelrlcontrol2b8::set_p_runningstate_rl(uint8_t* data,
    Wheel_rl_control_2b8::Runningstate_rlType runningstate_rl) {
  int x = runningstate_rl;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 8);
}


Wheelrlcontrol2b8* Wheelrlcontrol2b8::set_directionstate_rl(
    Wheel_rl_control_2b8::Directionstate_rlType directionstate_rl) {
  directionstate_rl_ = directionstate_rl;
  return this;
 }

// config detail: {'bit': 40, 'enum': {0: 'DIRECTIONSTATE_RL_REVERSE', 1: 'DIRECTIONSTATE_RL_FORWARD'}, 'is_signed_var': False, 'len': 8, 'name': 'DirectionState_rl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Wheelrlcontrol2b8::set_p_directionstate_rl(uint8_t* data,
    Wheel_rl_control_2b8::Directionstate_rlType directionstate_rl) {
  int x = directionstate_rl;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 8);
}


int Wheelrlcontrol2b8::speed_rl_ctrl(const std::uint8_t* bytes, int32_t length) const {
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

Wheel_rl_control_2b8::Runningstate_rlType Wheelrlcontrol2b8::runningstate_rl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Wheel_rl_control_2b8::Runningstate_rlType ret =  static_cast<Wheel_rl_control_2b8::Runningstate_rlType>(x);
  return ret;
}

Wheel_rl_control_2b8::Directionstate_rlType Wheelrlcontrol2b8::directionstate_rl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Wheel_rl_control_2b8::Directionstate_rlType ret =  static_cast<Wheel_rl_control_2b8::Directionstate_rlType>(x);
  return ret;
}
}  // namespace bxk
}  // namespace canbus
}  // namespace apollo
