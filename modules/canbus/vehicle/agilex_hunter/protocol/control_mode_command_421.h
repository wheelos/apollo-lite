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
namespace agilex_hunter {

class Controlmodecommand421 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Controlmodecommand421();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 7, 'enum': {0: 'CONTROL_MODE_CMD_STANDBY', 1: 'CONTROL_MODE_CMD_CAN'}, 'is_signed_var': False, 'len': 8, 'name': 'Control_Mode_Cmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Controlmodecommand421* set_control_mode_cmd(Control_mode_command_421::Control_mode_cmdType control_mode_cmd);

 private:

  // config detail: {'bit': 7, 'enum': {0: 'CONTROL_MODE_CMD_STANDBY', 1: 'CONTROL_MODE_CMD_CAN'}, 'is_signed_var': False, 'len': 8, 'name': 'Control_Mode_Cmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_control_mode_cmd(uint8_t* data, Control_mode_command_421::Control_mode_cmdType control_mode_cmd);

  Control_mode_command_421::Control_mode_cmdType control_mode_cmd(const std::uint8_t* bytes, const int32_t length) const;

 private:
  Control_mode_command_421::Control_mode_cmdType control_mode_cmd_;
};

}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo


