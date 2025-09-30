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

#include "modules/canbus/vehicle/bxk/protocol/light_control_7ff.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace bxk {

using ::apollo::drivers::canbus::Byte;

const int32_t Lightcontrol7ff::ID = 0x7FF;

// public
Lightcontrol7ff::Lightcontrol7ff() { Reset(); }

uint32_t Lightcontrol7ff::GetPeriod() const {
  static const uint32_t PERIOD = 0 * 1000;
  return PERIOD;
}

void Lightcontrol7ff::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_bxk()->mutable_light_control_7ff()->set_light_type(light_type(bytes, length));
  chassis->mutable_bxk()->mutable_light_control_7ff()->set_light_switch(light_switch(bytes, length));
}

void Lightcontrol7ff::UpdateData(uint8_t* data) {
  set_p_light_type(data, light_type_);
  set_p_light_switch(data, light_switch_);
}

void Lightcontrol7ff::Reset() {
  // TODO(All) :  you should check this manually
  light_type_ = Light_control_7ff::LIGHT_TYPE_BRAKELIGHT;
  light_switch_ = Light_control_7ff::LIGHT_SWITCH_OFF;
}

Lightcontrol7ff* Lightcontrol7ff::set_light_type(
    Light_control_7ff::Light_typeType light_type) {
  light_type_ = light_type;
  return this;
 }

// config detail: {'bit': 0, 'enum': {2: 'LIGHT_TYPE_BRAKELIGHT', 7: 'LIGHT_TYPE_RIGHTTURNLIGHT', 8: 'LIGHT_TYPE_LEFTTURNLIGHT'}, 'is_signed_var': False, 'len': 8, 'name': 'Light_Type', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|8]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Lightcontrol7ff::set_p_light_type(uint8_t* data,
    Light_control_7ff::Light_typeType light_type) {
  int x = light_type;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


Lightcontrol7ff* Lightcontrol7ff::set_light_switch(
    Light_control_7ff::Light_switchType light_switch) {
  light_switch_ = light_switch;
  return this;
 }

// config detail: {'bit': 8, 'enum': {0: 'LIGHT_SWITCH_OFF', 1: 'LIGHT_SWITCH_ON'}, 'is_signed_var': False, 'len': 8, 'name': 'Light_Switch', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|8]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Lightcontrol7ff::set_p_light_switch(uint8_t* data,
    Light_control_7ff::Light_switchType light_switch) {
  int x = light_switch;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}


Light_control_7ff::Light_typeType Lightcontrol7ff::light_type(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Light_control_7ff::Light_typeType ret =  static_cast<Light_control_7ff::Light_typeType>(x);
  return ret;
}

Light_control_7ff::Light_switchType Lightcontrol7ff::light_switch(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Light_control_7ff::Light_switchType ret =  static_cast<Light_control_7ff::Light_switchType>(x);
  return ret;
}
}  // namespace bxk
}  // namespace canbus
}  // namespace apollo
