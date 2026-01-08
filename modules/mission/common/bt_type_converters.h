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

//  Created Date: 2025-12-13
//  Author: daohu527

#pragma once

#include <sstream>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"

#include "modules/common_msgs/basic_msgs/geometry.pb.h"

namespace BT {

inline std::vector<StringView> splitStringView(StringView str, char sep) {
  std::vector<StringView> output;
  const char* data = str.data();
  size_t len = str.length();
  size_t prev = 0;
  for (size_t i = 0; i < len; ++i) {
    if (data[i] == sep) {
      output.push_back(str.substr(prev, i - prev));
      prev = i + 1;
    }
  }
  output.push_back(str.substr(prev, len - prev));
  return output;
}

template <>
inline apollo::common::PointENU convertFromString(StringView str) {
  // Split the string, assuming the format is "x;y;z" or "x,y,z"
  auto parts = splitStringView(str, ',');

  if (parts.size() < 2) {
    // Allow only x, y, z defaults to 0
    throw RuntimeError(
        "Invalid PointENU format. Expected 'x,y' or 'x,y,z'. Got: " +
        std::string(str));
  }

  apollo::common::PointENU p;

  // Recursively call convertFromString<double>, which is built into BT.CPP.
  p.set_x(convertFromString<double>(parts[0]));
  p.set_y(convertFromString<double>(parts[1]));

  if (parts.size() >= 3) {
    p.set_z(convertFromString<double>(parts[2]));
  } else {
    p.set_z(0.0);
  }

  return p;
}

}  // namespace BT
