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

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/common_msgs/chassis_msgs/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace yunle {

class Bmsrtsoc17904001 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Bmsrtsoc17904001();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name': 'BMS_current_Cur', 'offset': -3000.0, 'order': 'intel', 'physical_range': '[3000|9553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  double bms_current_cur(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'BMS_current_Vol', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|6553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  double bms_current_vol(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'BMS_Total_VolBat', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|6553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  double bms_total_volbat(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 48, 'description': '荷电状态', 'is_signed_var': False, 'len': 16, 'name': 'BMS_SOC', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|6553.5]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  double bms_soc(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo


