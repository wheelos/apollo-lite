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

#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/tools/roadlog/common/roadlog_layout.h"
#include "modules/tools/roadlog/processor/roadlog_runtime_types.h"

namespace apollo {
namespace data {

class RoadlogSegmentManager {
 public:
  explicit RoadlogSegmentManager(const RoadlogLayout& layout);

  void ConfigureRetention(uint32_t ring_segment_count);
  void RefreshSegments();
  void PinWindow(uint64_t begin_time, uint64_t end_time,
                 std::set<std::string>* segment_paths);
  bool IsWindowCovered(const std::set<std::string>& segment_paths,
                       uint64_t window_end_time) const;
  std::vector<RoadlogSegmentInfo> CollectSegments(
      uint64_t window_begin_time, uint64_t window_end_time,
      const std::set<std::string>& segment_paths, bool* partial) const;
  void Release(const std::set<std::string>& segment_paths);
  bool CleanupExpired();

 private:
  void RegisterSegment(const std::string& record_path);
  bool SegmentIntersectsWindow(const RoadlogSegmentInfo& segment,
                               uint64_t begin_time, uint64_t end_time) const;

  RoadlogLayout layout_;
  uint32_t ring_segment_count_ = 1;
  std::string ring_file_basename_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, RoadlogSegmentInfo> segments_;
  std::vector<std::string> segment_order_;
};

}  // namespace data
}  // namespace apollo
