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

// Created Date: 2026-04-21
// Author: daohu527

#include "modules/common/util/proj_helper.h"

#include <cstdlib>
#include <string>

namespace apollo {
namespace common {
namespace util {

namespace {

void SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

bool HasProjData(std::string* error) {
  const char* proj_data = std::getenv("PROJ_DATA");
  if (proj_data == nullptr || proj_data[0] == '\0') {
    SetError("PROJ_DATA is not set. Source cyber/setup.bash after building the "
             "targets so bazel-bin/external/proj~/data is available.",
             error);
    return false;
  }
  return true;
}

}  // namespace

PJ* ProjHelper::CreateNormalizedCrsToCrs(PJ_CONTEXT* proj_context,
                                         const std::string& source_crs,
                                         const std::string& target_crs,
                                         std::string* error) {
  if (!HasProjData(error)) {
    return nullptr;
  }

  PJ* transform = proj_create_crs_to_crs(proj_context, source_crs.c_str(),
                                         target_crs.c_str(), nullptr);
  if (transform == nullptr) {
    SetError("Failed to create PROJ transformation from " + source_crs +
                 " to " + target_crs,
             error);
    return nullptr;
  }

  PJ* normalized_transform =
      proj_normalize_for_visualization(proj_context, transform);
  proj_destroy(transform);

  if (normalized_transform == nullptr) {
    SetError("Failed to normalize PROJ transformation from " + source_crs +
                 " to " + target_crs,
             error);
    return nullptr;
  }

  return normalized_transform;
}

}  // namespace util
}  // namespace common
}  // namespace apollo
