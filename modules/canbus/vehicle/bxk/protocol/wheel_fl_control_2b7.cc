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

#include "modules/canbus/vehicle/bxk/protocol/wheel_fl_control_2b7.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace bxk {

using ::apollo::drivers::canbus::Byte;

const int32_t Wheelflcontrol2b7::ID = 0x2B7;

// public
Wheelflcontrol2b7::Wheelflcontrol2b7() { Reset(); }

uint32_t Wheelflcontrol2b7::GetPeriod() const {
  static const uint32_t PERIOD = 0 * 1000;
  return PERIOD;
}

void Wheelflcontrol2b7::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_bxk()->mutable_wheel_fl_control_2b7()->set_speed_fl_ctrl(speed_fl_ctrl(bytes, length));
  chassis->mutable_bxk()->mutable_wheel_fl_control_2b7()->set_runningstate_fl(runningstate_fl(bytes, length));
  chassis->mutable_bxk()->mutable_wheel_fl_control_2b7()->set_directionstate_fl(directionstate_fl(bytes, length));
}

void Wheelflcontrol2b7::UpdateData(uint8_t* data) {
  set_p_speed_fl_ctrl(data, speed_fl_ctrl_);
  set_p_runningstate_fl(data, runningstate_fl_);
  set_p_directionstate_fl(data, directionstate_fl_);
}

void Wheelflcontrol2b7::Reset() {
  // TODO(All) :  you should check this manually
  speed_fl_ctrl_ = 0;
  runningstate_fl_ = Wheel_fl_control_2b7::RUNNINGSTATE_FL_STOPPED;
  directionstate_fl_ = Wheel_fl_control_2b7::DIRECTIONSTATE_FL_REVERSE;
}

Wheelflcontrol2b7* Wheelflcontrol2b7::set_speed_fl_ctrl(
    int speed_fl_ctrl) {
  speed_fl_ctrl_ = speed_fl_ctrl;
  return this;
 }

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 32, 'name': 'Speed_fl_Ctrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1000]', 'physical_unit': 'RPM', 'precision': 1.0, 'type': 'int'}
void Wheelflcontrol2b7::set_p_speed_fl_ctrl(uint8_t* data,
    int speed_fl_ctrl) {
  speed_fl_ctrl = ProtocolData::BoundedValue(0, 1000, speed_fl_ctrl);
  int x = speed_fl_ctrl;
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


Wheelflcontrol2b7* Wheelflcontrol2b7::set_runningstate_fl(
    Wheel_fl_control_2b7::Runningstate_flType runningstate_fl) {
  runningstate_fl_ = runningstate_fl;
  return this;
 }

// config detail: {'bit': 32, 'enum': {0: 'RUNNINGSTATE_FL_STOPPED', 2: 'RUNNINGSTATE_FL_RUNNING'}, 'is_signed_var': False, 'len': 8, 'name': 'RunningState_fl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Wheelflcontrol2b7::set_p_runningstate_fl(uint8_t* data,
    Wheel_fl_control_2b7::Runningstate_flType runningstate_fl) {
  int x = runningstate_fl;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 8);
}


Wheelflcontrol2b7* Wheelflcontrol2b7::set_directionstate_fl(
    Wheel_fl_control_2b7::Directionstate_flType directionstate_fl) {
  directionstate_fl_ = directionstate_fl;
  return this;
 }

// config detail: {'bit': 40, 'enum': {0: 'DIRECTIONSTATE_FL_REVERSE', 1: 'DIRECTIONSTATE_FL_FORWARD'}, 'is_signed_var': False, 'len': 8, 'name': 'DirectionState_fl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Wheelflcontrol2b7::set_p_directionstate_fl(uint8_t* data,
    Wheel_fl_control_2b7::Directionstate_flType directionstate_fl) {
  int x = directionstate_fl;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 8);
}


int Wheelflcontrol2b7::speed_fl_ctrl(const std::uint8_t* bytes, int32_t length) const {
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

Wheel_fl_control_2b7::Runningstate_flType Wheelflcontrol2b7::runningstate_fl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Wheel_fl_control_2b7::Runningstate_flType ret =  static_cast<Wheel_fl_control_2b7::Runningstate_flType>(x);
  return ret;
}

Wheel_fl_control_2b7::Directionstate_flType Wheelflcontrol2b7::directionstate_fl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Wheel_fl_control_2b7::Directionstate_flType ret =  static_cast<Wheel_fl_control_2b7::Directionstate_flType>(x);
  return ret;
}
}  // namespace bxk
}  // namespace canbus
}  // namespace apollo
