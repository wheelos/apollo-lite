// Copyright 2026 WheelOS All Rights Reserved.
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

#include "modules/transform/calibration_registry.h"

#include <cstdlib>
#include <filesystem>  // NOLINT(build/c++17)
#include <string>
#include <utility>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace transform {

namespace fs = std::filesystem;

std::string CalibrationRegistry::MakeKey(const std::string& frame_id,
                                         const std::string& child_frame_id) {
  return frame_id + "\n" + child_frame_id;
}

void CalibrationRegistry::Clear() {
  entries_.clear();
  entry_map_.clear();
  child_frame_map_.clear();
}

std::string CalibrationRegistry::ResolveFilePath(const std::string& raw_path) {
  if (raw_path.empty()) {
    return "";
  }

  std::string relative_path = raw_path;
  if (relative_path.rfind("/apollo/", 0) == 0) {
    relative_path = relative_path.substr(8);
  } else if (relative_path.rfind("/apollo", 0) == 0) {
    relative_path = relative_path.substr(7);
  } else if (!relative_path.empty() && relative_path.front() == '/') {
    relative_path = relative_path.substr(1);
  }

  const char* overlay_root = std::getenv("APOLLO_CONFIG_OVERLAY_ROOT");
  if (overlay_root != nullptr && *overlay_root != '\0') {
    fs::path candidate = fs::path(overlay_root) / relative_path;
    if (fs::exists(candidate)) {
      return candidate.lexically_normal().string();
    }
  }

  const char* apollo_root = std::getenv("APOLLO_ROOT_DIR");
  if (apollo_root != nullptr && *apollo_root != '\0') {
    fs::path candidate = fs::path(apollo_root) / relative_path;
    if (fs::exists(candidate)) {
      return candidate.lexically_normal().string();
    }
  }

  if (fs::exists(raw_path)) {
    return fs::path(raw_path).lexically_normal().string();
  }

  if (apollo_root != nullptr && *apollo_root != '\0') {
    return (fs::path(apollo_root) / relative_path).lexically_normal().string();
  }

  return raw_path;
}

bool CalibrationRegistry::Load(const apollo::static_transform::Conf& conf) {
  Clear();
  for (const auto& extrinsic_file : conf.extrinsic_file()) {
    if (!extrinsic_file.has_frame_id() || extrinsic_file.frame_id().empty()) {
      AERROR << "Extrinsic entry missing frame_id.";
      return false;
    }
    if (!extrinsic_file.has_child_frame_id() ||
        extrinsic_file.child_frame_id().empty()) {
      AERROR << "Extrinsic entry missing child_frame_id.";
      return false;
    }
    if (!extrinsic_file.has_file_path() ||
        extrinsic_file.file_path().empty()) {
      AERROR << "Extrinsic entry missing file_path for "
             << extrinsic_file.frame_id() << " -> "
             << extrinsic_file.child_frame_id();
      return false;
    }

    const std::string key =
        MakeKey(extrinsic_file.frame_id(), extrinsic_file.child_frame_id());
    if (entry_map_.find(key) != entry_map_.end()) {
      AERROR << "Duplicate frame pair in calibration conf: "
             << extrinsic_file.frame_id() << " -> "
             << extrinsic_file.child_frame_id();
      return false;
    }

    ExtrinsicEntry entry;
    entry.frame_id = extrinsic_file.frame_id();
    entry.child_frame_id = extrinsic_file.child_frame_id();
    entry.file_path = extrinsic_file.file_path();
    entry.resolved_file_path = ResolveFilePath(extrinsic_file.file_path());
    entry.enable =
        extrinsic_file.has_enable() ? extrinsic_file.enable() : false;

    const size_t index = entries_.size();
    child_frame_map_[entry.child_frame_id] = index;
    entries_.push_back(std::move(entry));
    entry_map_[key] = index;
  }

  return true;
}

bool CalibrationRegistry::LoadFromFile(const std::string& conf_file_path) {
  apollo::static_transform::Conf conf;
  if (!cyber::common::GetProtoFromFile(conf_file_path, &conf)) {
    AERROR << "Failed to parse static transform conf from: " << conf_file_path;
    return false;
  }
  return Load(conf);
}

const ExtrinsicEntry* CalibrationRegistry::Find(
    const std::string& frame_id, const std::string& child_frame_id) const {
  const std::string key = MakeKey(frame_id, child_frame_id);
  const auto it = entry_map_.find(key);
  if (it == entry_map_.end()) {
    return nullptr;
  }
  return &entries_[it->second];
}

const ExtrinsicEntry* CalibrationRegistry::FindByChildFrame(
    const std::string& child_frame_id) const {
  const auto it = child_frame_map_.find(child_frame_id);
  if (it == child_frame_map_.end()) {
    return nullptr;
  }
  return &entries_[it->second];
}

std::vector<ExtrinsicEntry> CalibrationRegistry::EnabledEntries() const {
  std::vector<ExtrinsicEntry> enabled;
  enabled.reserve(entries_.size());
  for (const auto& entry : entries_) {
    if (entry.enable) {
      enabled.push_back(entry);
    }
  }
  return enabled;
}

std::string CalibrationRegistry::ResolveIntrinsicPath(
    const std::string& sensor_name, const std::string& default_dir) {
  if (sensor_name.empty()) {
    return "";
  }
  std::string raw_path;
  if (!default_dir.empty()) {
    raw_path = default_dir + "/" + sensor_name + "_intrinsics.yaml";
  } else {
    raw_path = "modules/perception/data/params/" + sensor_name +
               "_intrinsics.yaml";
  }
  return ResolveFilePath(raw_path);
}

}  // namespace transform
}  // namespace apollo
