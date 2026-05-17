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

#include "modules/tools/roadlog/processor/roadlog_segment_manager.h"

#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <cctype>
#include <cstdio>

#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/record/file/record_file_reader.h"

namespace apollo {
namespace data {

namespace {

using cyber::common::GetFileName;
using cyber::common::ListSubPaths;
using cyber::common::PathExists;
using cyber::record::RecordFileReader;

bool IsRecordValid(const std::string& record_path) {
  if (!PathExists(record_path)) {
    return false;
  }
  RecordFileReader file_reader;
  if (!file_reader.Open(record_path)) {
    AERROR << "failed to open record file for checking header: " << record_path;
    return false;
  }
  const bool is_complete = file_reader.GetHeader().is_complete();
  file_reader.Close();
  return is_complete;
}

bool IsSegmentFile(const std::string& file_name, const std::string& basename) {
  const std::string expected_prefix = basename + ".";
  if (file_name.rfind(expected_prefix, 0) != 0) {
    return false;
  }
  const std::string suffix = file_name.substr(expected_prefix.size());
  return suffix.size() == 5 &&
         std::all_of(suffix.begin(), suffix.end(), [](const unsigned char ch) {
           return std::isdigit(ch) != 0;
         });
}

uint64_t FileSize(const std::string& path) {
  struct stat file_stat{};
  if (stat(path.c_str(), &file_stat) != 0) {
    return 0;
  }
  return static_cast<uint64_t>(file_stat.st_size);
}

}  // namespace

RoadlogSegmentManager::RoadlogSegmentManager(const RoadlogLayout& layout)
    : layout_(layout),
      ring_file_basename_(GetFileName(layout_.ring_file_prefix, false)) {}

void RoadlogSegmentManager::ConfigureRetention(
    const uint32_t ring_segment_count) {
  ring_segment_count_ = std::max(1U, ring_segment_count);
}

void RoadlogSegmentManager::RefreshSegments() {
  std::vector<std::string> files = ListSubPaths(layout_.ring_dir, DT_REG);
  std::sort(files.begin(), files.end());
  for (const auto& file_name : files) {
    if (!IsSegmentFile(file_name, ring_file_basename_)) {
      continue;
    }
    const std::string record_path =
        absl::StrCat(layout_.ring_dir, "/", file_name);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (segments_.find(record_path) != segments_.end()) {
        continue;
      }
    }
    if (!IsRecordValid(record_path)) {
      continue;
    }
    RegisterSegment(record_path);
  }
}

void RoadlogSegmentManager::PinWindow(const uint64_t begin_time,
                                      const uint64_t end_time,
                                      std::set<std::string>* segment_paths) {
  if (segment_paths == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& segment_path : segment_order_) {
    auto segment_iter = segments_.find(segment_path);
    if (segment_iter == segments_.end()) {
      continue;
    }
    if (!SegmentIntersectsWindow(segment_iter->second, begin_time, end_time)) {
      continue;
    }
    if (segment_paths->insert(segment_path).second) {
      segment_iter->second.pin_count += 1;
    }
  }
}

bool RoadlogSegmentManager::IsWindowCovered(
    const std::set<std::string>& segment_paths,
    const uint64_t window_end_time) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& segment_path : segment_paths) {
    auto segment_iter = segments_.find(segment_path);
    if (segment_iter == segments_.end()) {
      continue;
    }
    const auto& segment = segment_iter->second;
    if (segment.begin_time <= window_end_time &&
        segment.end_time >= window_end_time) {
      return true;
    }
  }
  return false;
}

std::vector<RoadlogSegmentInfo> RoadlogSegmentManager::CollectSegments(
    const uint64_t window_begin_time, const uint64_t window_end_time,
    const std::set<std::string>& segment_paths, bool* partial) const {
  std::vector<RoadlogSegmentInfo> segments;
  bool is_partial = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& segment_path : segment_paths) {
      auto segment_iter = segments_.find(segment_path);
      if (segment_iter == segments_.end()) {
        is_partial = true;
        continue;
      }
      segments.push_back(segment_iter->second);
    }
  }
  std::sort(segments.begin(), segments.end(),
            [](const RoadlogSegmentInfo& lhs, const RoadlogSegmentInfo& rhs) {
              if (lhs.begin_time != rhs.begin_time) {
                return lhs.begin_time < rhs.begin_time;
              }
              return lhs.path < rhs.path;
            });
  if (segments.empty()) {
    is_partial = true;
  } else {
    if (segments.front().begin_time > window_begin_time) {
      is_partial = true;
    }
    uint64_t covered_until = segments.front().end_time;
    for (size_t i = 1; i < segments.size(); ++i) {
      if (segments[i].begin_time > covered_until) {
        is_partial = true;
      }
      covered_until = std::max(covered_until, segments[i].end_time);
    }
    if (covered_until < window_end_time) {
      is_partial = true;
    }
  }
  if (partial != nullptr) {
    *partial = is_partial;
  }
  return segments;
}

void RoadlogSegmentManager::Release(
    const std::set<std::string>& segment_paths) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& segment_path : segment_paths) {
    auto segment_iter = segments_.find(segment_path);
    if (segment_iter != segments_.end() && segment_iter->second.pin_count > 0) {
      segment_iter->second.pin_count -= 1;
    }
  }
}

bool RoadlogSegmentManager::CleanupExpired() {
  std::vector<std::string> removed_paths;
  bool out_of_space = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t retained_segments = segment_order_.size();
    for (auto iter = segment_order_.begin();
         iter != segment_order_.end() &&
         retained_segments > ring_segment_count_;) {
      auto segment_iter = segments_.find(*iter);
      if (segment_iter == segments_.end()) {
        iter = segment_order_.erase(iter);
        continue;
      }
      if (segment_iter->second.pin_count > 0) {
        ++iter;
        continue;
      }
      removed_paths.push_back(segment_iter->second.path);
      segments_.erase(segment_iter);
      iter = segment_order_.erase(iter);
      retained_segments -= 1;
    }
    out_of_space = retained_segments > ring_segment_count_;
  }

  for (const auto& removed_path : removed_paths) {
    if (std::remove(removed_path.c_str()) != 0) {
      AWARN << "failed to delete expired segment: " << removed_path;
    }
  }
  return out_of_space;
}

void RoadlogSegmentManager::RegisterSegment(const std::string& record_path) {
  RecordFileReader file_reader;
  if (!file_reader.Open(record_path)) {
    AERROR << "failed to open completed segment: " << record_path;
    return;
  }
  RoadlogSegmentInfo segment;
  segment.path = record_path;
  segment.begin_time = file_reader.GetHeader().begin_time();
  segment.end_time = file_reader.GetHeader().end_time();
  segment.bytes = FileSize(record_path);
  file_reader.Close();

  std::lock_guard<std::mutex> lock(mutex_);
  if (segments_.find(record_path) != segments_.end()) {
    return;
  }
  segments_.emplace(record_path, segment);
  segment_order_.push_back(record_path);
  std::sort(segment_order_.begin(), segment_order_.end(),
            [this](const std::string& lhs, const std::string& rhs) {
              const auto& left = segments_.at(lhs);
              const auto& right = segments_.at(rhs);
              if (left.begin_time != right.begin_time) {
                return left.begin_time < right.begin_time;
              }
              return lhs < rhs;
            });
}

bool RoadlogSegmentManager::SegmentIntersectsWindow(
    const RoadlogSegmentInfo& segment, const uint64_t begin_time,
    const uint64_t end_time) const {
  return segment.begin_time <= end_time && segment.end_time >= begin_time;
}

}  // namespace data
}  // namespace apollo
