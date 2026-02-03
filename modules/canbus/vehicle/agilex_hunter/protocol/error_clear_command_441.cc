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

#include "modules/canbus/vehicle/agilex_hunter/protocol/error_clear_command_441.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace agilex_hunter {

using ::apollo::drivers::canbus::Byte;

const int32_t Errorclearcommand441::ID = 0x441;

// public
Errorclearcommand441::Errorclearcommand441() { Reset(); }

uint32_t Errorclearcommand441::GetPeriod() const {
  static const uint32_t PERIOD = 0 * 1000;
  return PERIOD;
}

void Errorclearcommand441::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_agilex_hunter()->mutable_error_clear_command_441()->set_error_clear_cmd(error_clear_cmd(bytes, length));
}

void Errorclearcommand441::UpdateData(uint8_t* data) {
  set_p_error_clear_cmd(data, error_clear_cmd_);
}

void Errorclearcommand441::Reset() {
  // TODO(All) :  you should check this manually
  error_clear_cmd_ = 0;
}

Errorclearcommand441* Errorclearcommand441::set_error_clear_cmd(
    int error_clear_cmd) {
  error_clear_cmd_ = error_clear_cmd;
  return this;
 }

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'Error_Clear_Cmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Errorclearcommand441::set_p_error_clear_cmd(uint8_t* data,
    int error_clear_cmd) {
  error_clear_cmd = ProtocolData::BoundedValue(0, 255, error_clear_cmd);
  int x = error_clear_cmd;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


int Errorclearcommand441::error_clear_cmd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo
