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

#pragma once

#include "modules/common_msgs/chassis_msgs/chassis_detail.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace bxk {

class Wheelflcontrol2b7 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Wheelflcontrol2b7();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 32, 'name': 'Speed_fl_Ctrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1000]', 'physical_unit': 'RPM', 'precision': 1.0, 'type': 'int'}
  Wheelflcontrol2b7* set_speed_fl_ctrl(int speed_fl_ctrl);

  // config detail: {'bit': 32, 'enum': {0: 'RUNNINGSTATE_FL_STOPPED', 2: 'RUNNINGSTATE_FL_RUNNING'}, 'is_signed_var': False, 'len': 8, 'name': 'RunningState_fl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Wheelflcontrol2b7* set_runningstate_fl(Wheel_fl_control_2b7::Runningstate_flType runningstate_fl);

  // config detail: {'bit': 40, 'enum': {0: 'DIRECTIONSTATE_FL_REVERSE', 1: 'DIRECTIONSTATE_FL_FORWARD'}, 'is_signed_var': False, 'len': 8, 'name': 'DirectionState_fl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Wheelflcontrol2b7* set_directionstate_fl(Wheel_fl_control_2b7::Directionstate_flType directionstate_fl);

 private:

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 32, 'name': 'Speed_fl_Ctrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1000]', 'physical_unit': 'RPM', 'precision': 1.0, 'type': 'int'}
  void set_p_speed_fl_ctrl(uint8_t* data, int speed_fl_ctrl);

  // config detail: {'bit': 32, 'enum': {0: 'RUNNINGSTATE_FL_STOPPED', 2: 'RUNNINGSTATE_FL_RUNNING'}, 'is_signed_var': False, 'len': 8, 'name': 'RunningState_fl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_runningstate_fl(uint8_t* data, Wheel_fl_control_2b7::Runningstate_flType runningstate_fl);

  // config detail: {'bit': 40, 'enum': {0: 'DIRECTIONSTATE_FL_REVERSE', 1: 'DIRECTIONSTATE_FL_FORWARD'}, 'is_signed_var': False, 'len': 8, 'name': 'DirectionState_fl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_directionstate_fl(uint8_t* data, Wheel_fl_control_2b7::Directionstate_flType directionstate_fl);

  int speed_fl_ctrl(const std::uint8_t* bytes, const int32_t length) const;

  Wheel_fl_control_2b7::Runningstate_flType runningstate_fl(const std::uint8_t* bytes, const int32_t length) const;

  Wheel_fl_control_2b7::Directionstate_flType directionstate_fl(const std::uint8_t* bytes, const int32_t length) const;

 private:
  int speed_fl_ctrl_;
  Wheel_fl_control_2b7::Runningstate_flType runningstate_fl_;
  Wheel_fl_control_2b7::Directionstate_flType directionstate_fl_;
};

}  // namespace bxk
}  // namespace canbus
}  // namespace apollo


