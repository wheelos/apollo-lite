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

#include "gtest/gtest.h"

namespace apollo {
namespace data {
namespace {

TEST(RoadlogLayoutTest, BuildsDerivedDirectoriesFromRoot) {
  const auto layout = BuildRoadlogLayout("/tmp/roadlog/task-001");

  EXPECT_EQ("/tmp/roadlog/task-001", layout.root_dir);
  EXPECT_EQ("/tmp/roadlog/task-001/ring", layout.ring_dir);
  EXPECT_EQ("/tmp/roadlog/task-001/events", layout.events_dir);
  EXPECT_EQ("/tmp/roadlog/task-001/meta", layout.meta_dir);
  EXPECT_EQ("/tmp/roadlog/task-001/ring/roadlog.record", layout.ring_file_prefix);
  EXPECT_EQ("/tmp/roadlog/task-001/meta/trigger_events.log",
            layout.trigger_log_path);
}

}  // namespace
}  // namespace data
}  // namespace apollo
