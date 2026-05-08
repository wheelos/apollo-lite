/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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

#include <string>

namespace apollo {
namespace data {

struct RoadlogLayout {
  std::string root_dir;
  std::string ring_dir;
  std::string events_dir;
  std::string meta_dir;
  std::string ring_file_prefix;
  std::string trigger_log_path;
};

RoadlogLayout BuildRoadlogLayout(const std::string& root_dir);

}  // namespace data
}  // namespace apollo
