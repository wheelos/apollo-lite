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

#pragma once

#include "modules/common_msgs/chassis_msgs/chassis_detail.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace yunle {

class Ccustatus51 : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Ccustatus51();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 34, 'description': '自动驾驶模式刹车信号',
  // 'is_signed_var': False, 'len': 1, 'name': 'SCU_Brake_Singal', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'bool'}
  bool scu_brake_singal(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 33, 'description': '紧急制动按钮信号',
  // 'is_signed_var': False, 'len': 1, 'name': 'Emergency_Brake', 'offset': 0.0,
  // 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'bool'}
  bool emergency_brake(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'description': '遥控器刹车信号',
  // 'is_signed_var': False, 'len': 1, 'name': 'Remote_Brake', 'offset': 0.0,
  // 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'bool'}
  bool remote_brake(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 5, 'description': '驾驶模式切换按钮',
  // 'is_signed_var': False, 'len': 1, 'name': 'CCU_Drive_Mode_Shift', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'bool'}
  bool ccu_drive_mode_shift(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 59, 'description': '刹车灯状态', 'is_signed_var':
  // False, 'len': 1, 'name': 'Position_Light_Sts', 'offset': 0.0, 'order':
  // 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
  // 'type': 'bool'}
  bool position_light_sts(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 7, 'description': '前轮转向角度方向，左正右负',
  // 'is_signed_var': False, 'len': 1, 'name': 'Steering_Wheel_Direction',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool steering_wheel_direction(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'bit': 57, 'description': '右转向灯状态', 'is_signed_var':
  // False, 'len': 1, 'name': 'Right_Turn_Light_Sts', 'offset': 0.0, 'order':
  // 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
  // 'type': 'bool'}
  bool right_turn_light_sts(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 60, 'description': '近光灯状态', 'is_signed_var':
  // False, 'len': 1, 'name': 'LowBeam_Sts', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool lowbeam_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 56, 'description': '左转向灯状态', 'is_signed_var':
  // False, 'len': 1, 'name': 'Left_Turn_Light_Sts', 'offset': 0.0, 'order':
  // 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
  // 'type': 'bool'}
  bool left_turn_light_sts(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 20, 'description': '底盘速度', 'is_signed_var':
  // False, 'len': 9, 'name': 'CCU_Vehicle_Speed', 'offset': 0.0, 'order':
  // 'intel', 'physical_range': '[0|51]', 'physical_unit': 'km/h', 'precision':
  // 0.1, 'type': 'double'}
  double ccu_vehicle_speed(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 8, 'description':
  // '前轮转向角度，120对应实际转角27°', 'is_signed_var': False, 'len': 12,
  // 'name': 'CCU_Steering_Wheel_Angle', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|120]', 'physical_unit': '', 'precision': 0.1, 'type':
  // 'double'}
  double ccu_steering_wheel_angle(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'bit': 0, 'description': '底盘档位状态', 'is_signed_var':
  // False, 'len': 2, 'name': 'CCU_ShiftLevel_Sts', 'offset': 0.0, 'order':
  // 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0,
  // 'type': 'int'}
  int ccu_shiftlevel_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 2, 'description': '底盘刹车状态', 'is_signed_var':
  // False, 'len': 1, 'name': 'CCU_P_Sts', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool ccu_p_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 3, 'description': 'VCU点火信号状态',
  // 'is_signed_var': False, 'len': 2, 'name': 'CCU_Ignition_Sts', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'int'}
  int ccu_ignition_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 29, 'description': '底盘驾驶模式', 'is_signed_var':
  // False, 'len': 3, 'name': 'CCU_Drive_Mode', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int ccu_drive_mode(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
