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

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "modules/transform/proto/static_transform_conf.pb.h"

namespace apollo {
namespace transform {

struct ExtrinsicEntry {
  std::string frame_id;
  std::string child_frame_id;
  std::string file_path;
  std::string resolved_file_path;
  bool enable = false;
  std::string sensor_name;
  std::string intrinsic_path;
  std::string resolved_intrinsic_path;
};

class CalibrationRegistry {
 public:
  CalibrationRegistry() = default;
  ~CalibrationRegistry() = default;

  bool Load(const apollo::static_transform::Conf& conf);
  bool LoadFromFile(const std::string& conf_file_path);

  const ExtrinsicEntry* Find(const std::string& frame_id,
                             const std::string& child_frame_id) const;
  const ExtrinsicEntry* FindByChildFrame(
      const std::string& child_frame_id) const;

  std::vector<ExtrinsicEntry> EnabledEntries() const;
  const std::vector<ExtrinsicEntry>& AllEntries() const { return entries_; }

  void Clear();

  static std::string ResolveFilePath(const std::string& raw_path);
  static std::string ResolveIntrinsicPath(const std::string& sensor_name,
                                          const std::string& default_dir = "");

 private:
  static std::string MakeKey(const std::string& frame_id,
                             const std::string& child_frame_id);

  std::vector<ExtrinsicEntry> entries_;
  std::unordered_map<std::string, size_t> entry_map_;
  std::unordered_map<std::string, size_t> child_frame_map_;
};

}  // namespace transform
}  // namespace apollo
