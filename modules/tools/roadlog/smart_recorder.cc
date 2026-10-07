/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/common/resource_manager.h"
#include "modules/tools/roadlog/processor/roadlog_runtime.h"
#include "modules/tools/roadlog/smart_recorder_gflags.h"

using apollo::cyber::common::GetProtoFromFile;
using apollo::data::RoadlogRuntime;
using apollo::data::SmartRecordTrigger;

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_roadlog_root_dir.empty()) {
    AERROR << "roadlog_root_dir must be provided";
    return -1;
  }
  AINFO << "roadlog root dir: " << FLAGS_roadlog_root_dir
        << ". config file: " << FLAGS_smart_recorder_config_filename
        << ". program name: " << argv[0];

  RoadlogRuntime runtime(FLAGS_roadlog_root_dir);

  SmartRecordTrigger trigger_conf;
  std::string selected_config_path;
  ACHECK(apollo::cyber::common::ResourceManager::InitializeConfigRoot() &&
         apollo::cyber::common::ResourceManager::ResolveConfigPath(
             FLAGS_smart_recorder_config_filename, &selected_config_path))
      << "Failed to load triggers config file "
      << FLAGS_smart_recorder_config_filename;
  ACHECK(GetProtoFromFile(selected_config_path, &trigger_conf))
      << "Failed to load triggers config file " << selected_config_path;
  AINFO << "[CONFIG] roadlog: " << selected_config_path;

  if (!runtime.Init(trigger_conf)) {
    AERROR << "failed to init roadlog runtime";
    return -1;
  }

  if (!runtime.Run()) {
    AERROR << "failed to run roadlog runtime";
    return -1;
  }

  return 0;
}
