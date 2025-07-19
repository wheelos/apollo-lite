// Copyright 2025 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-4-15
//  Author: daohu527

#pragma once

#include <math.h>

#include <cstdint>
#include <functional>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "google/protobuf/message.h"

#include "modules/common_msgs/basic_msgs/geometry.pb.h"

// Anonymous namespace that contains helper constants and functions.
namespace {

constexpr int kSecondsPerWeek = 60 * 60 * 24 * 7;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kAccelerationGravity = 9.80665;
constexpr float kFloatNaN = std::numeric_limits<float>::quiet_NaN();

// The NovAtel's orientation covariance matrix is pitch, roll, and yaw. We use
// the index array below to convert it to the orientation covariance matrix with
// order roll, pitch, and yaw.
constexpr int INDEX[] = {4, 3, 5, 1, 0, 2, 7, 6, 8};
static_assert(sizeof(INDEX) == 9 * sizeof(int), "Incorrect size of INDEX");

template <typename T>
constexpr bool is_zero(T value) {
  static_assert(
      std::is_floating_point_v<T> || std::is_integral_v<T>,
      "is_zero can only be used with floating point or integral types");

  if constexpr (std::is_floating_point_v<T>) {
    return std::abs(value) < std::numeric_limits<T>::epsilon();
  } else {
    return value == T{0};
  }
}

// CRC algorithm from the NovAtel document.
inline uint32_t crc32_word(uint32_t word) {
  for (int j = 0; j < 8; ++j) {
    if (word & 1) {
      word = (word >> 1) ^ 0xEDB88320;
    } else {
      word >>= 1;
    }
  }
  return word;
}

inline uint32_t crc32_block(const uint8_t *buffer, size_t length) {
  uint32_t word = 0;
  while (length--) {
    uint32_t t1 = (word >> 8) & 0xFFFFFF;
    uint32_t t2 = crc32_word((word ^ *buffer++) & 0xFF);
    word = t1 ^ t2;
  }
  return word;
}

// Converts NovAtel's azimuth (north = 0, east = 90) to FLU yaw (east = 0, north
// = pi/2).
constexpr double azimuth_deg_to_yaw_rad(double azimuth) {
  return (90.0 - azimuth) * kDegToRad;
}

// A helper that fills an Point3D object (which uses the FLU frame) using RFU
// measurements.
inline void rfu_to_flu(double r, double f, double u,
                       ::apollo::common::Point3D *flu) {
  flu->set_x(f);
  flu->set_y(-r);
  flu->set_z(u);
}

}  // namespace

namespace apollo {
namespace drivers {
namespace gnss {

constexpr char NMEA_START_FLAG = '$';          // Start flag
constexpr char NMEA_FIELD_SEPARATOR = ',';     // Field separator
constexpr char NMEA_CHECKSUM_SEPARATOR = '*';  // Checksum separator
constexpr char NMEA_END_OF_LINE_CR = '\r';     // End of line: Carriage return
constexpr char NMEA_END_OF_LINE_LF = '\n';     // End of line: Line feed

// A helper function that returns a pointer to a protobuf message of type T.
template <class T>
inline T *As(::google::protobuf::Message *message_ptr) {
  static_assert(std::is_base_of_v<::google::protobuf::Message, T>,
                "T must be derived from ::google::protobuf::Message");
  return dynamic_cast<T *>(message_ptr);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
