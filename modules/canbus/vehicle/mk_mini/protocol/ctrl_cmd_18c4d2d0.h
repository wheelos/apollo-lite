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
namespace mk_mini {

class Ctrlcmd18c4d2d0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Ctrlcmd18c4d2d0();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t *data) override;

  void Reset() override;

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'ctrl_cmd_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Ctrlcmd18c4d2d0* set_ctrl_cmd_check_bcc(int ctrl_cmd_check_bcc);

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name': 'ctrl_cmd_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Ctrlcmd18c4d2d0* set_ctrl_cmd_alive_cnt(int ctrl_cmd_alive_cnt);

  // config detail: {'bit': 36, 'is_signed_var': False, 'len': 8, 'name': 'ctrl_cmd_Brake', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|127]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Ctrlcmd18c4d2d0* set_ctrl_cmd_brake(int ctrl_cmd_brake);

  // config detail: {'bit': 20, 'is_signed_var': True, 'len': 16, 'name': 'ctrl_cmd_steering', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.01, 'type': 'double'}
  Ctrlcmd18c4d2d0* set_ctrl_cmd_steering(double ctrl_cmd_steering);

  // config detail: {'bit': 4, 'is_signed_var': False, 'len': 16, 'name': 'ctrl_cmd_velocity', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.001, 'type': 'double'}
  Ctrlcmd18c4d2d0* set_ctrl_cmd_velocity(double ctrl_cmd_velocity);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 4, 'name': 'ctrl_cmd_gear', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Ctrlcmd18c4d2d0* set_ctrl_cmd_gear(int ctrl_cmd_gear);

 private:

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'ctrl_cmd_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_ctrl_cmd_check_bcc(uint8_t* data, int ctrl_cmd_check_bcc);

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name': 'ctrl_cmd_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_ctrl_cmd_alive_cnt(uint8_t* data, int ctrl_cmd_alive_cnt);

  // config detail: {'bit': 36, 'is_signed_var': False, 'len': 8, 'name': 'ctrl_cmd_Brake', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|127]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_ctrl_cmd_brake(uint8_t* data, int ctrl_cmd_brake);

  // config detail: {'bit': 20, 'is_signed_var': True, 'len': 16, 'name': 'ctrl_cmd_steering', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.01, 'type': 'double'}
  void set_p_ctrl_cmd_steering(uint8_t* data, double ctrl_cmd_steering);

  // config detail: {'bit': 4, 'is_signed_var': False, 'len': 16, 'name': 'ctrl_cmd_velocity', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.001, 'type': 'double'}
  void set_p_ctrl_cmd_velocity(uint8_t* data, double ctrl_cmd_velocity);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 4, 'name': 'ctrl_cmd_gear', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_ctrl_cmd_gear(uint8_t* data, int ctrl_cmd_gear);

  int ctrl_cmd_check_bcc(const std::uint8_t* bytes, const int32_t length) const;

  int ctrl_cmd_alive_cnt(const std::uint8_t* bytes, const int32_t length) const;

  int ctrl_cmd_brake(const std::uint8_t* bytes, const int32_t length) const;

  double ctrl_cmd_steering(const std::uint8_t* bytes, const int32_t length) const;

  double ctrl_cmd_velocity(const std::uint8_t* bytes, const int32_t length) const;

  int ctrl_cmd_gear(const std::uint8_t* bytes, const int32_t length) const;

 private:
  int ctrl_cmd_check_bcc_;
  int ctrl_cmd_alive_cnt_;
  int ctrl_cmd_brake_;
  double ctrl_cmd_steering_;
  double ctrl_cmd_velocity_;
  int ctrl_cmd_gear_;
};

}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo


