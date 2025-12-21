/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <vector>

#include "modules/common_msgs/sensor_msgs/gnss_raw_observation.pb.h"

#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/rtcm_decode.h"

namespace apollo {
namespace drivers {
namespace gnss {

class Rtcm3Parser : public Parser {
 public:
  explicit Rtcm3Parser(const config::Config &config);
  explicit Rtcm3Parser(bool is_base_satation);

  virtual std::vector<ParsedMessage> ParseAllMessages() override;

  bool ProcessHeader() override { return true; }
  std::optional<std::vector<ParsedMessage>> ProcessPayload() {
    return std::nullopt;
  }

 private:
  void SetObservationTime();
  bool SetStationPosition();
  void FillKepplerOrbit(const eph_t &eph,
                        apollo::drivers::gnss::KepplerOrbit *keppler_orbit);
  void FillGlonassOrbit(const geph_t &eph,
                        apollo::drivers::gnss::GlonassOrbit *keppler_orbit);
  bool ProcessObservation(
      apollo::drivers::gnss::EpochObservation *observation_msg);
  bool ProcessEphemerides(apollo::drivers::gnss::GnssEphemeris *ephemeris_msg);
  bool ProcessStationParameters();

 private:
  bool is_base_station_ = false;
  bool init_flag_;

  rtcm_t rtcm_;

  struct Point3D {
    double x;
    double y;
    double z;
  };
  std::unordered_map<int, Point3D> station_location_;

  apollo::drivers::gnss::GnssEphemeris ephemeris_;
  apollo::drivers::gnss::EpochObservation observation_;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
