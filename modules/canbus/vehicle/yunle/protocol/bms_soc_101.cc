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

#include "modules/canbus/vehicle/yunle/protocol/bms_soc_101.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace yunle {

using ::apollo::drivers::canbus::Byte;

Bmssoc101::Bmssoc101() {}
const int32_t Bmssoc101::ID = 0x101;

void Bmssoc101::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_yunle()->mutable_bms_soc_101()->set_rsoc(rsoc(bytes, length));
}

// config detail: {'bit': 39, 'is_signed_var': True, 'len': 16, 'name': 'rsoc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
int Bmssoc101::rsoc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}
}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
