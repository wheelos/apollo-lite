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

#include "modules/tools/roadlog/common/roadlog_layout.h"

#include "absl/strings/str_cat.h"

namespace apollo {
namespace data {

RoadlogLayout BuildRoadlogLayout(const std::string& root_dir) {
  RoadlogLayout layout;
  layout.root_dir = root_dir;
  layout.ring_dir = absl::StrCat(root_dir, "/ring");
  layout.events_dir = absl::StrCat(root_dir, "/events");
  layout.meta_dir = absl::StrCat(root_dir, "/meta");
  layout.ring_file_prefix = absl::StrCat(layout.ring_dir, "/roadlog.record");
  layout.trigger_log_path = absl::StrCat(layout.meta_dir, "/trigger_events.log");
  return layout;
}

}  // namespace data
}  // namespace apollo
