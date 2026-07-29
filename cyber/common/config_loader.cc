/******************************************************************************
 * Copyright 2025 The Apollo Authors. All Rights Reserved.
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

#include "cyber/common/config_loader.h"

#include <memory>
#include <string>

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace common {

bool GetProtoFromFileWithOverride(const std::string& default_path,
                                  google::protobuf::Message* message) {
  // Step 1: Load the default (in-source) configuration.
  if (!GetProtoFromFile(default_path, message)) {
    AERROR << "Failed to load default config: " << default_path;
    return false;
  }

  // Step 2: Optionally apply an external persistent override.
  const std::string filename = GetFileName(default_path);
  const std::string override_path =
      std::string(kConfigOverrideDir) + "/" + filename;

  if (PathExists(override_path)) {
    std::unique_ptr<google::protobuf::Message> override_msg(message->New());
    if (GetProtoFromFile(override_path, override_msg.get())) {
      message->MergeFrom(*override_msg);
      AINFO << "Applied config override from: " << override_path;
    } else {
      AWARN << "Config override file found but could not be parsed: "
            << override_path;
    }
  }

  return true;
}

}  // namespace common
}  // namespace cyber
}  // namespace apollo
