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

#include "modules/canbus/vehicle/yunle/protocol/target_speed_feedback_7f1.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace yunle {

using ::apollo::drivers::canbus::Byte;

Targetspeedfeedback7f1::Targetspeedfeedback7f1() {}
const int32_t Targetspeedfeedback7f1::ID = 0x7F1;

void Targetspeedfeedback7f1::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_yunle()->mutable_target_speed_feedback_7f1()->set_target_speed_rpm(target_speed_rpm(bytes, length));
  chassis->mutable_yunle()->mutable_target_speed_feedback_7f1()->set_target_speed(target_speed(bytes, length));
  chassis->mutable_yunle()->mutable_target_speed_feedback_7f1()->set_scu_speed(scu_speed(bytes, length));
  chassis->mutable_yunle()->mutable_target_speed_feedback_7f1()->set_hdw_speed(hdw_speed(bytes, length));
}

// config detail: {'bit': 48, 'description': '实际平均轮速', 'is_signed_var': True, 'len': 16, 'name': 'target_speed_rpm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'rpm', 'precision': 0.1, 'type': 'double'}
double Targetspeedfeedback7f1::target_speed_rpm(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 32, 'description': '目标速度', 'is_signed_var': True, 'len': 16, 'name': 'target_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'km/h', 'precision': 0.1, 'type': 'double'}
double Targetspeedfeedback7f1::target_speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 16, 'description': '自动驾驶模式下发目标速度', 'is_signed_var': True, 'len': 16, 'name': 'scu_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'km/h', 'precision': 0.1, 'type': 'double'}
double Targetspeedfeedback7f1::scu_speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'description': '硬件目标速度', 'is_signed_var': True, 'len': 16, 'name': 'hdw_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'km/h', 'precision': 0.1, 'type': 'double'}
double Targetspeedfeedback7f1::hdw_speed(const std::uint8_t* bytes, int32_t length) const {
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
