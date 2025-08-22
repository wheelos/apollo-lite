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

class Waringlevel77 : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Waringlevel77();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 27, 'is_signed_var': True, 'len': 3, 'name':
  // 'MCU_Vol_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int mcu_vol_warning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 36, 'is_signed_var': True, 'len': 3, 'name':
  // 'Turn_Unstoppable_warning', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int turn_unstoppable_warning(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'bit': 33, 'is_signed_var': True, 'len': 3, 'name':
  // 'Turn_Lock_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int turn_lock_warning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 30, 'is_signed_var': True, 'len': 3, 'name':
  // 'Turn_Disconnect_warning', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int turn_disconnect_warning(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 24, 'is_signed_var': True, 'len': 3, 'name':
  // 'MCU_Temperature_warning', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int mcu_temperature_warning(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 21, 'is_signed_var': True, 'len': 3, 'name':
  // 'MCU_speed_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int mcu_speed_warning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 18, 'is_signed_var': True, 'len': 3, 'name':
  // 'MCU_Motor_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int mcu_motor_warning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 15, 'is_signed_var': True, 'len': 3, 'name':
  // 'MCU_Disconnect_warning', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int mcu_disconnect_warning(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 12, 'is_signed_var': True, 'len': 3, 'name':
  // 'MCU_Cur_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int mcu_cur_warning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 9, 'is_signed_var': True, 'len': 3, 'name':
  // 'BMS_Temperature_warning', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int bms_temperature_warning(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 6, 'is_signed_var': True, 'len': 3, 'name':
  // 'BMS_SOC_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bms_soc_warning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 3, 'is_signed_var': True, 'len': 3, 'name':
  // 'BMS_DischargeCur_warning', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int bms_dischargecur_warning(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 3, 'name':
  // 'BMS_chargeCur_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bms_chargecur_warning(const std::uint8_t* bytes,
                            const int32_t length) const;
};

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
