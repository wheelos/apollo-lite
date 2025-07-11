/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
namespace neolix_edu {

class Pas1stdata311 : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Pas1stdata311();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'phy=int*2;0xFF:no obstacle', 'offset': 0.0,
  // 'precision': 2.0, 'len': 8, 'name': 'PASDistance4', 'is_signed_var': False,
  // 'physical_range': '[0|510]', 'bit': 39, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'cm'}
  double pasdistance4(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'phy=int*2;0xFF:no obstacle', 'offset': 0.0,
  // 'precision': 2.0, 'len': 8, 'name': 'PASDistance3', 'is_signed_var': False,
  // 'physical_range': '[0|510]', 'bit': 31, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'cm'}
  double pasdistance3(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'PAS_F1_Status', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 0, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': 'bit'}
  bool pas_f1_status(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'PAS_F2_Status', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 1, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': 'bit'}
  bool pas_f2_status(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'PAS_F3_Status', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 2, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': 'bit'}
  bool pas_f3_status(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': '0x0:Invalid;0x1:Valid', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'PAS_F4_Status', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 3, 'type': 'bool', 'order':
  // 'motorola', 'physical_unit': 'bit'}
  bool pas_f4_status(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'phy=int*2;0xFF:no obstacle', 'offset': 0.0,
  // 'precision': 2.0, 'len': 8, 'name': 'PASDistance2', 'is_signed_var': False,
  // 'physical_range': '[0|510]', 'bit': 23, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'cm'}
  double pasdistance2(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'PASDistance1', 'offset': 0.0, 'precision': 2.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|510]', 'bit': 15,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
  double pasdistance1(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
