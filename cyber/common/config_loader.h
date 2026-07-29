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

/**
 * @file
 * @brief Layered configuration loader with override support.
 *
 * Provides a unified entry point for loading protobuf configs using a
 * three-tier override priority:
 *   1. Defaults — loaded from the in-source path (lowest priority)
 *   2. External persistent overrides — loaded from /data/conf/<filename>
 *      (higher priority, merged via MergeFrom)
 *
 * Usage:
 *   MyConfig cfg;
 *   CHECK(cyber::common::GetProtoFromFileWithOverride(FLAGS_my_conf, &cfg));
 *
 * See docs/configuration_override.md for the full design.
 */

#ifndef CYBER_COMMON_CONFIG_LOADER_H_
#define CYBER_COMMON_CONFIG_LOADER_H_

#include <string>

#include "google/protobuf/message.h"

namespace apollo {
namespace cyber {
namespace common {

/**
 * @brief The directory that holds external persistent config overrides.
 *        Files placed here shadow the in-source defaults for the same filename.
 */
constexpr char kConfigOverrideDir[] = "/data/conf";

/**
 * @brief Loads a protobuf config using layered override logic.
 *
 * Steps performed:
 *   1. Load the default config from @p default_path.
 *   2. If a file named after the basename of @p default_path exists inside
 *      kConfigOverrideDir, load it and deep-merge it into @p message via
 *      google::protobuf::Message::MergeFrom().  Only the fields explicitly
 *      set in the override file will overwrite the defaults; all other fields
 *      retain their default values.
 *
 * @param default_path  Absolute path to the default (in-source) config file.
 * @param message       Output proto message.  Must not be nullptr.
 * @return true  if the default config was loaded successfully (the override
 *               is optional and does not affect the return value).
 * @return false if the default config could not be loaded.
 */
bool GetProtoFromFileWithOverride(const std::string& default_path,
                                  google::protobuf::Message* message);

}  // namespace common
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_COMMON_CONFIG_LOADER_H_
