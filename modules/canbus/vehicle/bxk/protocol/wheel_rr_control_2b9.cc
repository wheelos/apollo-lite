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

#include "modules/canbus/vehicle/bxk/protocol/wheel_rr_control_2b9.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace bxk {

using ::apollo::drivers::canbus::Byte;

const int32_t Wheelrrcontrol2b9::ID = 0x2B9;

// public
Wheelrrcontrol2b9::Wheelrrcontrol2b9() { Reset(); }

uint32_t Wheelrrcontrol2b9::GetPeriod() const {
  static const uint32_t PERIOD = 0 * 1000;
  return PERIOD;
}

void Wheelrrcontrol2b9::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_bxk()->mutable_wheel_rr_control_2b9()->set_speed_rr_ctrl(speed_rr_ctrl(bytes, length));
  chassis->mutable_bxk()->mutable_wheel_rr_control_2b9()->set_runningstate_rr(runningstate_rr(bytes, length));
  chassis->mutable_bxk()->mutable_wheel_rr_control_2b9()->set_directionstate_rr(directionstate_rr(bytes, length));
}

void Wheelrrcontrol2b9::UpdateData(uint8_t* data) {
  set_p_speed_rr_ctrl(data, speed_rr_ctrl_);
  set_p_runningstate_rr(data, runningstate_rr_);
  set_p_directionstate_rr(data, directionstate_rr_);
}

void Wheelrrcontrol2b9::Reset() {
  // TODO(All) :  you should check this manually
  speed_rr_ctrl_ = 0;
  runningstate_rr_ = Wheel_rr_control_2b9::RUNNINGSTATE_RR_STOPPED;
  directionstate_rr_ = Wheel_rr_control_2b9::DIRECTIONSTATE_RR_REVERSE;
}

Wheelrrcontrol2b9* Wheelrrcontrol2b9::set_speed_rr_ctrl(
    int speed_rr_ctrl) {
  speed_rr_ctrl_ = speed_rr_ctrl;
  return this;
 }

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 32, 'name': 'Speed_rr_Ctrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1000]', 'physical_unit': 'RPM', 'precision': 1.0, 'type': 'int'}
void Wheelrrcontrol2b9::set_p_speed_rr_ctrl(uint8_t* data,
    int speed_rr_ctrl) {
  speed_rr_ctrl = ProtocolData::BoundedValue(0, 1000, speed_rr_ctrl);
  int x = speed_rr_ctrl;
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


Wheelrrcontrol2b9* Wheelrrcontrol2b9::set_runningstate_rr(
    Wheel_rr_control_2b9::Runningstate_rrType runningstate_rr) {
  runningstate_rr_ = runningstate_rr;
  return this;
 }

// config detail: {'bit': 32, 'enum': {0: 'RUNNINGSTATE_RR_STOPPED', 2: 'RUNNINGSTATE_RR_RUNNING'}, 'is_signed_var': False, 'len': 8, 'name': 'RunningState_rr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Wheelrrcontrol2b9::set_p_runningstate_rr(uint8_t* data,
    Wheel_rr_control_2b9::Runningstate_rrType runningstate_rr) {
  int x = runningstate_rr;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 8);
}


Wheelrrcontrol2b9* Wheelrrcontrol2b9::set_directionstate_rr(
    Wheel_rr_control_2b9::Directionstate_rrType directionstate_rr) {
  directionstate_rr_ = directionstate_rr;
  return this;
 }

// config detail: {'bit': 40, 'enum': {0: 'DIRECTIONSTATE_RR_REVERSE', 1: 'DIRECTIONSTATE_RR_FORWARD'}, 'is_signed_var': False, 'len': 8, 'name': 'DirectionState_rr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Wheelrrcontrol2b9::set_p_directionstate_rr(uint8_t* data,
    Wheel_rr_control_2b9::Directionstate_rrType directionstate_rr) {
  int x = directionstate_rr;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 8);
}


int Wheelrrcontrol2b9::speed_rr_ctrl(const std::uint8_t* bytes, int32_t length) const {
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

Wheel_rr_control_2b9::Runningstate_rrType Wheelrrcontrol2b9::runningstate_rr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Wheel_rr_control_2b9::Runningstate_rrType ret =  static_cast<Wheel_rr_control_2b9::Runningstate_rrType>(x);
  return ret;
}

Wheel_rr_control_2b9::Directionstate_rrType Wheelrrcontrol2b9::directionstate_rr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Wheel_rr_control_2b9::Directionstate_rrType ret =  static_cast<Wheel_rr_control_2b9::Directionstate_rrType>(x);
  return ret;
}
}  // namespace bxk
}  // namespace canbus
}  // namespace apollo
