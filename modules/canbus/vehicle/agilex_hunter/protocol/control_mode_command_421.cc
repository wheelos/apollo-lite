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

#include "modules/canbus/vehicle/agilex_hunter/protocol/control_mode_command_421.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace agilex_hunter {

using ::apollo::drivers::canbus::Byte;

const int32_t Controlmodecommand421::ID = 0x421;

// public
Controlmodecommand421::Controlmodecommand421() { Reset(); }

uint32_t Controlmodecommand421::GetPeriod() const {
  static const uint32_t PERIOD = 100 * 1000;
  return PERIOD;
}

void Controlmodecommand421::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_agilex_hunter()->mutable_control_mode_command_421()->set_control_mode_cmd(control_mode_cmd(bytes, length));
}

void Controlmodecommand421::UpdateData(uint8_t* data) {
  set_p_control_mode_cmd(data, control_mode_cmd_);
}

void Controlmodecommand421::Reset() {
  // TODO(All) :  you should check this manually
  control_mode_cmd_ = Control_mode_command_421::CONTROL_MODE_CMD_STANDBY;
}

Controlmodecommand421* Controlmodecommand421::set_control_mode_cmd(
    Control_mode_command_421::Control_mode_cmdType control_mode_cmd) {
  control_mode_cmd_ = control_mode_cmd;
  return this;
 }

// config detail: {'bit': 7, 'enum': {0: 'CONTROL_MODE_CMD_STANDBY', 1: 'CONTROL_MODE_CMD_CAN'}, 'is_signed_var': False, 'len': 8, 'name': 'Control_Mode_Cmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Controlmodecommand421::set_p_control_mode_cmd(uint8_t* data,
    Control_mode_command_421::Control_mode_cmdType control_mode_cmd) {
  int x = control_mode_cmd;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


Control_mode_command_421::Control_mode_cmdType Controlmodecommand421::control_mode_cmd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Control_mode_command_421::Control_mode_cmdType ret =  static_cast<Control_mode_command_421::Control_mode_cmdType>(x);
  return ret;
}
}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo
