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
#include <vector>

#include "modules/dreamview/backend/hmi/process_types.h"

namespace apollo {
namespace dreamview {

class LocalRunner {
 public:
  bool Start(const std::string& raw_command, ProcessHandle* handle) const;
  bool Stop(const ProcessHandle& handle,
            const std::vector<std::string>& keywords) const;

 private:
  static std::string NormalizeStartCommand(const std::string& raw_command);
};

}  // namespace dreamview
}  // namespace apollo
