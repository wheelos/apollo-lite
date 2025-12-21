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

#include "modules/canbus/vehicle/yunle/protocol/bms_rt_soc_17904001.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace yunle {

using ::apollo::drivers::canbus::Byte;

Bmsrtsoc17904001::Bmsrtsoc17904001() {}
const int32_t Bmsrtsoc17904001::ID = 0x97904001;

void Bmsrtsoc17904001::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_yunle()->mutable_bms_rt_soc_17904001()->set_bms_current_cur(bms_current_cur(bytes, length));
  chassis->mutable_yunle()->mutable_bms_rt_soc_17904001()->set_bms_current_vol(bms_current_vol(bytes, length));
  chassis->mutable_yunle()->mutable_bms_rt_soc_17904001()->set_bms_total_volbat(bms_total_volbat(bytes, length));
  chassis->mutable_yunle()->mutable_bms_rt_soc_17904001()->set_bms_soc(bms_soc(bytes, length));
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name': 'bms_current_cur', 'offset': -3000.0, 'order': 'intel', 'physical_range': '[3000|9553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double Bmsrtsoc17904001::bms_current_cur(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -3000.000000;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'bms_current_vol', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|6553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double Bmsrtsoc17904001::bms_current_vol(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'bms_total_volbat', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|6553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double Bmsrtsoc17904001::bms_total_volbat(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 48, 'description': '荷电状态', 'is_signed_var': False, 'len': 16, 'name': 'bms_soc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|6553.5]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
double Bmsrtsoc17904001::bms_soc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}
}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
