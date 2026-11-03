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

#include "modules/dreamview/backend/hmi/mode_registry.h"

#include <string>
#include <unordered_set>

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace dreamview {

namespace {

std::string GetParentDir(const std::string& path) {
  const size_t slash_pos = path.rfind('/');
  if (slash_pos == std::string::npos) {
    return ".";
  }
  return path.substr(0, slash_pos);
}

void MergeModule(const Module& child, Module* merged) {
  CHECK_NOTNULL(merged);
  if (child.has_start_command()) {
    merged->set_start_command(child.start_command());
  }
  if (child.has_stop_command()) {
    merged->set_stop_command(child.stop_command());
  }
  if (child.has_process_monitor_config()) {
    merged->mutable_process_monitor_config()->CopyFrom(
        child.process_monitor_config());
  }
  if (child.has_required_for_safety()) {
    merged->set_required_for_safety(child.required_for_safety());
  }
  if (child.depends_on_size() > 0) {
    merged->mutable_depends_on()->CopyFrom(child.depends_on());
  }
  if (child.has_auto_start()) {
    merged->set_auto_start(child.auto_start());
  }
  if (child.has_exclusive_group()) {
    merged->set_exclusive_group(child.exclusive_group());
  }
}

void MergeCyberModule(const CyberModule& child, CyberModule* merged) {
  CHECK_NOTNULL(merged);
  if (child.dag_files_size() > 0) {
    merged->mutable_dag_files()->CopyFrom(child.dag_files());
  }
  if (child.has_required_for_safety()) {
    merged->set_required_for_safety(child.required_for_safety());
  }
  if (child.has_process_group()) {
    merged->set_process_group(child.process_group());
  }
}

void MergeMode(const HMIMode& base, const HMIMode& child, HMIMode* merged) {
  CHECK_NOTNULL(merged);
  merged->CopyFrom(base);

  for (const auto& iter : child.cyber_modules()) {
    auto* target = &(*merged->mutable_cyber_modules())[iter.first];
    MergeCyberModule(iter.second, target);
  }
  for (const auto& iter : child.modules()) {
    auto* target = &(*merged->mutable_modules())[iter.first];
    MergeModule(iter.second, target);
  }
  for (const auto& iter : child.monitored_components()) {
    auto* target = &(*merged->mutable_monitored_components())[iter.first];
    target->MergeFrom(iter.second);
  }
  for (const auto& iter : child.other_components()) {
    (*merged->mutable_other_components())[iter.first] = iter.second;
  }
  if (child.has_base_mode()) {
    merged->set_base_mode(child.base_mode());
  }
}

}  // namespace

HMIMode ModeRegistry::LoadMode(const std::string& mode_config_path) const {
  std::unordered_set<std::string> loading_paths;
  return LoadMode(mode_config_path, &loading_paths);
}

HMIMode ModeRegistry::LoadMode(
    const std::string& mode_config_path,
    std::unordered_set<std::string>* loading_paths) const {
  ACHECK(loading_paths != nullptr);
  ACHECK(loading_paths->insert(mode_config_path).second)
      << "Circular base_mode chain detected at " << mode_config_path;

  HMIMode mode;
  ACHECK(cyber::common::GetProtoFromFile(mode_config_path, &mode))
      << "Unable to parse HMIMode from file " << mode_config_path;

  HMIMode merged_mode;
  if (mode.has_base_mode() && !mode.base_mode().empty()) {
    const std::string base_mode_path =
        ResolveBaseModePath(mode.base_mode(), mode_config_path);
    ACHECK(!base_mode_path.empty())
        << "Unable to resolve base_mode \"" << mode.base_mode()
        << "\" in " << mode_config_path;
    HMIMode base_mode = LoadMode(base_mode_path, loading_paths);
    MergeMode(base_mode, mode, &merged_mode);
  } else {
    merged_mode.CopyFrom(mode);
  }

  TranslateCyberModules(mode_config_path, &merged_mode);
  merged_mode.clear_base_mode();
  loading_paths->erase(mode_config_path);
  AINFO << "Loaded HMI mode: " << merged_mode.DebugString();
  return merged_mode;
}

std::string ModeRegistry::ResolveBaseModePath(const std::string& base_mode,
                                              const std::string& mode_config_path) {
  if (base_mode.empty()) {
    return "";
  }
  if (cyber::common::PathExists(base_mode)) {
    return base_mode;
  }
  const std::string mode_dir = GetParentDir(mode_config_path);
  const std::string relative_path = absl::StrCat(mode_dir, "/", base_mode);
  if (cyber::common::PathExists(relative_path)) {
    return relative_path;
  }
  if (!absl::EndsWith(base_mode, ".pb.txt")) {
    const std::string relative_pbtxt = absl::StrCat(relative_path, ".pb.txt");
    if (cyber::common::PathExists(relative_pbtxt)) {
      return relative_pbtxt;
    }
  }
  return "";
}

void ModeRegistry::TranslateCyberModules(const std::string& mode_config_path,
                                         HMIMode* mode) {
  CHECK_NOTNULL(mode);
  for (const auto& iter : mode->cyber_modules()) {
    const std::string& module_name = iter.first;
    const CyberModule& cyber_module = iter.second;
    ACHECK(!cyber_module.dag_files().empty())
        << "None dag file is provided for " << module_name << " module in "
        << mode_config_path;

    Module& module = (*mode->mutable_modules())[module_name];
    module.set_required_for_safety(cyber_module.required_for_safety());

    module.set_start_command("mainboard");
    const auto& process_group = cyber_module.process_group();
    if (!process_group.empty()) {
      absl::StrAppend(module.mutable_start_command(), " -p ", process_group);
    }
    for (const std::string& dag : cyber_module.dag_files()) {
      absl::StrAppend(module.mutable_start_command(), " -d ", dag);
    }

    module.clear_stop_command();
    const std::string& first_dag = cyber_module.dag_files(0);
    module.mutable_process_monitor_config()->clear_command_keywords();
    module.mutable_process_monitor_config()->add_command_keywords("mainboard");
    module.mutable_process_monitor_config()->add_command_keywords(first_dag);
  }
  mode->clear_cyber_modules();
}

}  // namespace dreamview
}  // namespace apollo
