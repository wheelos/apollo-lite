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
#include <unordered_set>

#include "modules/dreamview/proto/hmi_mode.pb.h"

namespace apollo {
namespace dreamview {

class ModeRegistry {
 public:
  HMIMode LoadMode(const std::string& mode_config_path) const;

 private:
  HMIMode LoadMode(const std::string& mode_config_path,
                   std::unordered_set<std::string>* loading_paths) const;
  static std::string ResolveBaseModePath(const std::string& base_mode,
                                         const std::string& mode_config_path);
  static void TranslateCyberModules(const std::string& mode_config_path,
                                    HMIMode* mode);
};

}  // namespace dreamview
}  // namespace apollo
