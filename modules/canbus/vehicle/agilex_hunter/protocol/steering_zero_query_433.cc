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

#include "modules/canbus/vehicle/agilex_hunter/protocol/steering_zero_query_433.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace agilex_hunter {

using ::apollo::drivers::canbus::Byte;

const int32_t Steeringzeroquery433::ID = 0x433;

// public
Steeringzeroquery433::Steeringzeroquery433() { Reset(); }

uint32_t Steeringzeroquery433::GetPeriod() const {
  static const uint32_t PERIOD = 100 * 1000;
  return PERIOD;
}

void Steeringzeroquery433::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_agilex_hunter()->mutable_steering_zero_query_433()->set_zero_query(zero_query(bytes, length));
}

void Steeringzeroquery433::UpdateData(uint8_t* data) {
  set_p_zero_query(data, zero_query_);
}

void Steeringzeroquery433::Reset() {
  // TODO(All) :  you should check this manually
  zero_query_ = 0;
}

Steeringzeroquery433* Steeringzeroquery433::set_zero_query(
    int zero_query) {
  zero_query_ = zero_query;
  return this;
 }

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'Zero_Query', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[170|170]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Steeringzeroquery433::set_p_zero_query(uint8_t* data,
    int zero_query) {
  zero_query = ProtocolData::BoundedValue(170, 170, zero_query);
  int x = zero_query;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


int Steeringzeroquery433::zero_query(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo
