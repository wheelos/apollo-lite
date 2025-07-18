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
namespace ge3 {

class Scu3303 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Scu3303();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'VIN string character 15', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN15', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin15(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 14', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN14', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin14(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 13', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN13', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 47, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin13(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 12', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN12', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 39, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin12(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 11', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN11', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 31, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin11(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 10', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN10', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin10(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 09', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN09', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin09(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 08', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN08', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin08(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
