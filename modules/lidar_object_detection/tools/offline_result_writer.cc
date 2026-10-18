// Copyright 2026 WheelOS. All Rights Reserved.
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

#include "modules/lidar_object_detection/tools/offline_result_writer.h"

#include <fstream>
#include <string>

namespace apollo {
namespace lidar_object_detection {

bool WriteFrameResult(const CenterPointFrameResult& result,
                      const std::string& output_path, std::string* error) {
  if (output_path.empty()) {
    if (error != nullptr) {
      *error = "CenterPoint result output path is empty";
    }
    return false;
  }
  std::ofstream output(output_path, std::ios::binary | std::ios::trunc);
  if (!output.is_open()) {
    if (error != nullptr) {
      *error = "Unable to open CenterPoint result output: " + output_path;
    }
    return false;
  }
  if (!result.SerializeToOstream(&output)) {
    if (error != nullptr) {
      *error = "Unable to serialize CenterPoint result: " + output_path;
    }
    return false;
  }
  output.flush();
  if (!output.good()) {
    if (error != nullptr) {
      *error = "Unable to flush CenterPoint result: " + output_path;
    }
    return false;
  }
  return true;
}

}  // namespace lidar_object_detection
}  // namespace apollo
