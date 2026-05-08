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

#include <set>
#include <string>
#include <vector>

#include "modules/tools/roadlog/common/trigger_arbiter.h"

namespace apollo {
namespace data {

struct RoadlogSegmentInfo {
  std::string path;
  uint64_t begin_time = 0;
  uint64_t end_time = 0;
  uint64_t bytes = 0;
  size_t pin_count = 0;
};

struct RoadlogEventState {
  ArbitratedEvent metadata;
  bool exporting = false;
  std::set<std::string> segment_paths;
};

struct RoadlogEventExportPlan {
  ArbitratedEvent metadata;
  bool partial = false;
  std::string output_dir;
  std::vector<RoadlogSegmentInfo> segments;
};

}  // namespace data
}  // namespace apollo
