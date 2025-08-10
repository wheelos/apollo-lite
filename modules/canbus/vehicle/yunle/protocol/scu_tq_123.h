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
namespace yunle {

class Scutq123 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Scutq123();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 48, 'description': 'Torque available calculated by inverterTorque available calculated by inverter', 'is_signed_var': True, 'len': 16, 'name': 'Torque_cmd_Rear_R', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
  Scutq123* set_torque_cmd_rear_r(double torque_cmd_rear_r);

  // config detail: {'bit': 32, 'is_signed_var': True, 'len': 16, 'name': 'Torque_cmd_Rear_L', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
  Scutq123* set_torque_cmd_rear_l(double torque_cmd_rear_l);

  // config detail: {'bit': 16, 'description': 'Torque available calculated by inverterTorque available calculated by inverter', 'is_signed_var': True, 'len': 16, 'name': 'Torque_cmd_Forward_R', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
  Scutq123* set_torque_cmd_forward_r(double torque_cmd_forward_r);

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'Torque_cmd_Forward_L', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
  Scutq123* set_torque_cmd_forward_l(double torque_cmd_forward_l);

 private:

  // config detail: {'bit': 48, 'description': 'Torque available calculated by inverterTorque available calculated by inverter', 'is_signed_var': True, 'len': 16, 'name': 'Torque_cmd_Rear_R', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
  void set_p_torque_cmd_rear_r(uint8_t* data, double torque_cmd_rear_r);

  // config detail: {'bit': 32, 'is_signed_var': True, 'len': 16, 'name': 'Torque_cmd_Rear_L', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
  void set_p_torque_cmd_rear_l(uint8_t* data, double torque_cmd_rear_l);

  // config detail: {'bit': 16, 'description': 'Torque available calculated by inverterTorque available calculated by inverter', 'is_signed_var': True, 'len': 16, 'name': 'Torque_cmd_Forward_R', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
  void set_p_torque_cmd_forward_r(uint8_t* data, double torque_cmd_forward_r);

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'Torque_cmd_Forward_L', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
  void set_p_torque_cmd_forward_l(uint8_t* data, double torque_cmd_forward_l);

  double torque_cmd_rear_r(const std::uint8_t* bytes, const int32_t length) const;

  double torque_cmd_rear_l(const std::uint8_t* bytes, const int32_t length) const;

  double torque_cmd_forward_r(const std::uint8_t* bytes, const int32_t length) const;

  double torque_cmd_forward_l(const std::uint8_t* bytes, const int32_t length) const;

 private:
  double torque_cmd_rear_r_;
  double torque_cmd_rear_l_;
  double torque_cmd_forward_r_;
  double torque_cmd_forward_l_;
};

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo


