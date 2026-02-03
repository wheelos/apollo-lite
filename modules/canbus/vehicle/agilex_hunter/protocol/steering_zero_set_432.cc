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

#include "modules/canbus/vehicle/agilex_hunter/protocol/steering_zero_set_432.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace agilex_hunter {

using ::apollo::drivers::canbus::Byte;

const int32_t Steeringzeroset432::ID = 0x432;

// public
Steeringzeroset432::Steeringzeroset432() { Reset(); }

uint32_t Steeringzeroset432::GetPeriod() const {
  static const uint32_t PERIOD = 100 * 1000;
  return PERIOD;
}

void Steeringzeroset432::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_agilex_hunter()->mutable_steering_zero_set_432()->set_zero_offset_h(zero_offset_h(bytes, length));
}

void Steeringzeroset432::UpdateData(uint8_t* data) {
  set_p_zero_offset_h(data, zero_offset_h_);
}

void Steeringzeroset432::Reset() {
  // TODO(All) :  you should check this manually
  zero_offset_h_ = 0;
}

Steeringzeroset432* Steeringzeroset432::set_zero_offset_h(
    int zero_offset_h) {
  zero_offset_h_ = zero_offset_h;
  return this;
 }

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'Zero_Offset_H', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Steeringzeroset432::set_p_zero_offset_h(uint8_t* data,
    int zero_offset_h) {
  zero_offset_h = ProtocolData::BoundedValue(0, 255, zero_offset_h);
  int x = zero_offset_h;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


int Steeringzeroset432::zero_offset_h(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo
