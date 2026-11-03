/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/dreamview/backend/hmi/hmi_worker.h"

#include <cstdlib>
#include <string>

#include "modules/common/util/map_util.h"
#include "modules/dreamview/backend/hmi/process_manager.h"
#include "gtest/gtest.h"

DECLARE_string(hmi_modes_config_path);

namespace apollo {
namespace dreamview {

class HMIWorkerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    const char* test_srcdir = std::getenv("TEST_SRCDIR");
    const char* test_workspace = std::getenv("TEST_WORKSPACE");
    ASSERT_NE(test_srcdir, nullptr);
    ASSERT_NE(test_workspace, nullptr);
    FLAGS_hmi_modes_config_path =
        std::string(test_srcdir) + "/" + test_workspace +
        "/modules/dreamview/conf/hmi_modes";
  }
};

TEST_F(HMIWorkerTest, LoadConfigAndMode) {
  const HMIConfig config = HMIWorker::LoadConfig();
  for (const auto& iter : config.modes()) {
    const std::string& mode_conf_file = iter.second;
    const HMIMode& mode = HMIWorker::LoadMode(mode_conf_file);
    EXPECT_FALSE(mode.modules().empty())
        << "No HMI module loaded from " << mode_conf_file;
  }
}

TEST_F(HMIWorkerTest, LoadsOnlyFunctionalModes) {
  const HMIConfig config = HMIWorker::LoadConfig();
  EXPECT_EQ(config.modes_size(), 4);
  EXPECT_NE(apollo::common::util::FindOrNull(config.modes(), "Runtime"),
            nullptr);
  EXPECT_NE(
      apollo::common::util::FindOrNull(config.modes(), "Sensor Calibration"),
      nullptr);
  EXPECT_NE(apollo::common::util::FindOrNull(config.modes(), "Map Collection"),
            nullptr);
  EXPECT_NE(apollo::common::util::FindOrNull(config.modes(), "Testing"),
            nullptr);
}

TEST_F(HMIWorkerTest, KeepsDebugToolsManual) {
  const HMIConfig config = HMIWorker::LoadConfig();
  const std::string* testing_path =
      apollo::common::util::FindOrNull(config.modes(), "Testing");
  ASSERT_NE(testing_path, nullptr);

  const HMIMode mode = HMIWorker::LoadMode(*testing_path);
  for (const auto& module : mode.modules()) {
    EXPECT_FALSE(module.second.auto_start()) << module.first;
  }
}

TEST(ProcessManagerTest, RejectsInvalidDependencies) {
  HMIMode mode;
  (*mode.mutable_modules())["A"].add_depends_on("Missing");
  ProcessManager manager;
  EXPECT_FALSE(manager.ValidateMode(mode));

  mode.Clear();
  (*mode.mutable_modules())["A"].add_depends_on("B");
  (*mode.mutable_modules())["B"].add_depends_on("A");
  EXPECT_FALSE(manager.ValidateMode(mode));
}

TEST(ProcessManagerTest, StartsDependenciesAndEnforcesExclusivity) {
  HMIMode mode;
  Module* dependency = &(*mode.mutable_modules())["Dependency"];
  dependency->set_start_command("exec sleep 30");
  dependency->mutable_process_monitor_config()->add_command_keywords("sleep");

  Module* primary = &(*mode.mutable_modules())["Primary"];
  primary->set_start_command("exec sleep 30");
  primary->mutable_process_monitor_config()->add_command_keywords("sleep");
  primary->add_depends_on("Dependency");
  primary->set_exclusive_group("tool");

  Module* alternative = &(*mode.mutable_modules())["Alternative"];
  alternative->set_start_command("exec sleep 30");
  alternative->mutable_process_monitor_config()->add_command_keywords("sleep");
  alternative->set_exclusive_group("tool");

  ProcessManager manager;
  ASSERT_TRUE(manager.SetMode(mode));
  ASSERT_TRUE(manager.StartModule("Primary"));
  EXPECT_FALSE(manager.StartModule("Alternative"));
  EXPECT_FALSE(manager.StopModule("Dependency"));
  EXPECT_TRUE(manager.StopModule("Primary"));
  EXPECT_TRUE(manager.StopModule("Dependency"));
  EXPECT_TRUE(manager.StartModule("Alternative"));
  EXPECT_TRUE(manager.StopModule("Alternative"));
}

}  // namespace dreamview
}  // namespace apollo
