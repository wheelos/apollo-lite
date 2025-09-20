/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/apple/protocol/light_control_7ff.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace apple {

using ::apollo::drivers::canbus::Byte;

Lightcontrol7ff::Lightcontrol7ff() {}
const int32_t Lightcontrol7ff::ID = 0x7FF;

void Lightcontrol7ff::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_apple()->mutable_light_control_7ff()->set_light_type(light_type(bytes, length));
  chassis->mutable_apple()->mutable_light_control_7ff()->set_light_switch(light_switch(bytes, length));
}

// config detail: {'bit': 0, 'enum': {2: 'LIGHT_TYPE_BRAKELIGHT', 7: 'LIGHT_TYPE_RIGHTTURNLIGHT', 8: 'LIGHT_TYPE_LEFTTURNLIGHT'}, 'is_signed_var': False, 'len': 8, 'name': 'light_type', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|8]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Light_control_7ff::Light_typeType Lightcontrol7ff::light_type(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Light_control_7ff::Light_typeType ret =  static_cast<Light_control_7ff::Light_typeType>(x);
  return ret;
}

// config detail: {'bit': 8, 'enum': {0: 'LIGHT_SWITCH_OFF', 1: 'LIGHT_SWITCH_ON'}, 'is_signed_var': False, 'len': 8, 'name': 'light_switch', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|8]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Light_control_7ff::Light_switchType Lightcontrol7ff::light_switch(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Light_control_7ff::Light_switchType ret =  static_cast<Light_control_7ff::Light_switchType>(x);
  return ret;
}
}  // namespace apple
}  // namespace canbus
}  // namespace apollo
