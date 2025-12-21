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

// #include "math.h"
#include "modules/drivers/radar/nano_radar/protocol/collision_detection_state_408.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"
#include "modules/drivers/radar/nano_radar/protocol/const_vars.h"

namespace apollo {
namespace drivers {
namespace nano_radar {

using apollo::drivers::canbus::Byte;

CollisionDetectionState408::CollisionDetectionState408() {}
const uint32_t CollisionDetectionState408::ID = 0x408;

void CollisionDetectionState408::Parse(const std::uint8_t* bytes,
                                       int32_t length,
                                       NanoRadar* nano_radar) const {
  auto state = nano_radar->mutable_collision_detection_state();
  state->set_activation(activation(bytes, length));
  state->set_number_of_regions(number_of_regions(bytes, length));
  state->set_min_detect_time(min_detect_time(bytes, length));
  state->set_mesa_counter(mesa_counter(bytes, length));
}

bool CollisionDetectionState408::activation(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes);
  uint32_t x = t0.get_byte(1, 1);

  bool ret = (x == 0x1);
  return ret;
}

int CollisionDetectionState408::number_of_regions(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 0);
  uint32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

double CollisionDetectionState408::min_detect_time(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * COLLISION_DETECTION_TIME_RES;
  return ret;
}

int CollisionDetectionState408::mesa_counter(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);

  x <<= 8;
  x |= t;

  double ret = x;
  return ret;
}

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
