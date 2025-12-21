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

#include "modules/canbus/vehicle/yunle/protocol/sas_angle_feedback_e1.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace yunle {

using ::apollo::drivers::canbus::Byte;

Sasanglefeedbacke1::Sasanglefeedbacke1() {}
const int32_t Sasanglefeedbacke1::ID = 0xE1;

void Sasanglefeedbacke1::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_yunle()->mutable_sas_angle_feedback_e1()->set_sas_angle_r(sas_angle_r(bytes, length));
  chassis->mutable_yunle()->mutable_sas_angle_feedback_e1()->set_sas_angle_f(sas_angle_f(bytes, length));
}

// config detail: {'bit': 24, 'is_signed_var': True, 'len': 16, 'name': 'sas_angle_r', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-3276.8|3276.7]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double Sasanglefeedbacke1::sas_angle_r(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'sas_angle_f', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-3276.8|3276.7]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double Sasanglefeedbacke1::sas_angle_f(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}
}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
