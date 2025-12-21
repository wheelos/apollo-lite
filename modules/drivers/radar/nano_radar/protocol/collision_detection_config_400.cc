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

#include "modules/drivers/radar/nano_radar/protocol/collision_detection_config_400.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/radar/nano_radar/protocol/const_vars.h"

namespace apollo {
namespace drivers {
namespace nano_radar {

using apollo::drivers::NanoRadar;
using apollo::drivers::canbus::Byte;

const uint32_t CollisionDetectionConfig400::ID = 0x400;

CollisionDetectionConfig400::CollisionDetectionConfig400() {}
CollisionDetectionConfig400::~CollisionDetectionConfig400() {}

uint32_t CollisionDetectionConfig400::GetPeriod() const {
  static const uint32_t PERIOD = 100 * 1000;
  return PERIOD;
}

/**
 * @brief update the data
 * @param data a pointer to the data to be updated
 */
void CollisionDetectionConfig400::UpdateData(uint8_t* data) {
  set_warning_reset_p(data, radar_conf_.coll_det_cfg_warning_reset());
  set_activation_p(data, radar_conf_.coll_det_cfg_activation());
  set_min_time_valid_p(data, radar_conf_.coll_det_cfg_min_time_valid());
  set_clear_regions_p(data, radar_conf_.coll_det_cfg_clear_regions());
  set_min_detect_time_p(
      data, static_cast<double>(radar_conf_.coll_det_cfg_min_detect_time()));
}

/**
 * @brief reset the private variables
 */
void CollisionDetectionConfig400::Reset() {
  radar_conf_.set_coll_det_cfg_warning_reset(false);
  radar_conf_.set_coll_det_cfg_activation(false);
  radar_conf_.set_coll_det_cfg_min_time_valid(false);
  radar_conf_.set_coll_det_cfg_clear_regions(false);
  radar_conf_.set_coll_det_cfg_min_detect_time(50);
}

RadarConf CollisionDetectionConfig400::radar_conf() { return radar_conf_; }

CollisionDetectionConfig400* CollisionDetectionConfig400::set_radar_conf(
    RadarConf radar_conf) {
  radar_conf_.CopyFrom(radar_conf);
  return this;
}

CollisionDetectionConfig400* CollisionDetectionConfig400::set_warning_reset(
    bool value) {
  radar_conf_.set_coll_det_cfg_warning_reset(value);
  return this;
}

CollisionDetectionConfig400* CollisionDetectionConfig400::set_activation(
    bool value) {
  radar_conf_.set_coll_det_cfg_activation(value);
  return this;
}

CollisionDetectionConfig400* CollisionDetectionConfig400::set_min_time_valid(
    bool value) {
  radar_conf_.set_coll_det_cfg_min_time_valid(value);
  return this;
}

CollisionDetectionConfig400* CollisionDetectionConfig400::set_clear_regions(
    bool value) {
  radar_conf_.set_coll_det_cfg_clear_regions(value);
  return this;
}

CollisionDetectionConfig400* CollisionDetectionConfig400::set_min_detect_time(
    double value) {
  radar_conf_.set_coll_det_cfg_min_detect_time(value);
  return this;
}

void CollisionDetectionConfig400::set_warning_reset_p(uint8_t* data,
                                                      bool value) {
  Byte frame(data);
  if (value) {
    frame.set_value(1, 0, 1);
  } else {
    frame.set_value(0, 0, 1);
  }
}

void CollisionDetectionConfig400::set_activation_p(uint8_t* data, bool value) {
  Byte frame(data);
  if (value) {
    frame.set_value(1, 1, 1);
  } else {
    frame.set_value(0, 1, 1);
  }
}

void CollisionDetectionConfig400::set_min_time_valid_p(uint8_t* data,
                                                       bool value) {
  Byte frame(data);
  if (value) {
    frame.set_value(1, 3, 1);
  } else {
    frame.set_value(0, 3, 1);
  }
}

void CollisionDetectionConfig400::set_clear_regions_p(uint8_t* data,
                                                      bool value) {
  Byte frame(data);
  if (value) {
    frame.set_value(1, 7, 1);
  } else {
    frame.set_value(0, 7, 1);
  }
}

void CollisionDetectionConfig400::set_min_detect_time_p(uint8_t* data,
                                                        double value) {
  uint8_t x = value / COLLISION_DETECTION_TIME_RES;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 1);
  to_set0.set_value(t, 0, 8);
}

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
