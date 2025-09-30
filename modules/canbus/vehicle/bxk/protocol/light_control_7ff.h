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

class Lightcontrol7ff : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Lightcontrol7ff();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 0, 'enum': {2: 'LIGHT_TYPE_BRAKELIGHT', 7: 'LIGHT_TYPE_RIGHTTURNLIGHT', 8: 'LIGHT_TYPE_LEFTTURNLIGHT'}, 'is_signed_var': False, 'len': 8, 'name': 'Light_Type', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|8]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Lightcontrol7ff* set_light_type(Light_control_7ff::Light_typeType light_type);

  // config detail: {'bit': 8, 'enum': {0: 'LIGHT_SWITCH_OFF', 1: 'LIGHT_SWITCH_ON'}, 'is_signed_var': False, 'len': 8, 'name': 'Light_Switch', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|8]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Lightcontrol7ff* set_light_switch(Light_control_7ff::Light_switchType light_switch);

 private:

  // config detail: {'bit': 0, 'enum': {2: 'LIGHT_TYPE_BRAKELIGHT', 7: 'LIGHT_TYPE_RIGHTTURNLIGHT', 8: 'LIGHT_TYPE_LEFTTURNLIGHT'}, 'is_signed_var': False, 'len': 8, 'name': 'Light_Type', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|8]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_light_type(uint8_t* data, Light_control_7ff::Light_typeType light_type);

  // config detail: {'bit': 8, 'enum': {0: 'LIGHT_SWITCH_OFF', 1: 'LIGHT_SWITCH_ON'}, 'is_signed_var': False, 'len': 8, 'name': 'Light_Switch', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|8]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_light_switch(uint8_t* data, Light_control_7ff::Light_switchType light_switch);

  Light_control_7ff::Light_typeType light_type(const std::uint8_t* bytes, const int32_t length) const;

  Light_control_7ff::Light_switchType light_switch(const std::uint8_t* bytes, const int32_t length) const;

 private:
  Light_control_7ff::Light_typeType light_type_;
  Light_control_7ff::Light_switchType light_switch_;
};

}  // namespace bxk
}  // namespace canbus
}  // namespace apollo


