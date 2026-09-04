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

#include "modules/transform/calibration_registry.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace transform {
namespace {

namespace fs = std::filesystem;

class CalibrationRegistryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    temp_dir_ = fs::temp_directory_path() / "calibration_registry_test";
    fs::create_directories(temp_dir_ / "overlay");
    fs::create_directories(temp_dir_ / "base");

    setenv("APOLLO_ROOT_DIR", (temp_dir_ / "base").c_str(), 1);
    setenv("APOLLO_CONFIG_OVERLAY_ROOT", (temp_dir_ / "overlay").c_str(), 1);
  }

  void TearDown() override {
    unsetenv("APOLLO_ROOT_DIR");
    unsetenv("APOLLO_CONFIG_OVERLAY_ROOT");
    std::error_code ec;
    fs::remove_all(temp_dir_, ec);
  }

  fs::path temp_dir_;
};

TEST_F(CalibrationRegistryTest, RejectsDuplicateFramePairs) {
  apollo::static_transform::Conf conf;
  auto* entry_a = conf.add_extrinsic_file();
  entry_a->set_frame_id("base_link");
  entry_a->set_child_frame_id("imu");
  entry_a->set_file_path("/apollo/modules/transform/conf/a.yaml");
  entry_a->set_enable(true);

  auto* entry_b = conf.add_extrinsic_file();
  entry_b->set_frame_id("base_link");
  entry_b->set_child_frame_id("imu");
  entry_b->set_file_path("/apollo/modules/transform/conf/b.yaml");
  entry_b->set_enable(true);

  CalibrationRegistry registry;
  EXPECT_FALSE(registry.Load(conf));
}

TEST_F(CalibrationRegistryTest, RejectsMissingFields) {
  CalibrationRegistry registry;

  {
    apollo::static_transform::Conf conf;
    auto* entry = conf.add_extrinsic_file();
    entry->set_child_frame_id("imu");
    entry->set_file_path("/apollo/a.yaml");
    EXPECT_FALSE(registry.Load(conf));
  }

  {
    apollo::static_transform::Conf conf;
    auto* entry = conf.add_extrinsic_file();
    entry->set_frame_id("base_link");
    entry->set_file_path("/apollo/a.yaml");
    EXPECT_FALSE(registry.Load(conf));
  }

  {
    apollo::static_transform::Conf conf;
    auto* entry = conf.add_extrinsic_file();
    entry->set_frame_id("base_link");
    entry->set_child_frame_id("imu");
    EXPECT_FALSE(registry.Load(conf));
  }
}

TEST_F(CalibrationRegistryTest, ResolvesOverlayAndBasePaths) {
  const std::string rel_path = "modules/transform/conf/cam.yaml";
  const fs::path overlay_file = temp_dir_ / "overlay" / rel_path;
  fs::create_directories(overlay_file.parent_path());
  {
    std::ofstream ofs(overlay_file);
    ofs << "test: overlay\n";
  }

  apollo::static_transform::Conf conf;
  auto* entry = conf.add_extrinsic_file();
  entry->set_frame_id("base_link");
  entry->set_child_frame_id("cam");
  entry->set_file_path("/apollo/" + rel_path);
  entry->set_enable(true);

  CalibrationRegistry registry;
  ASSERT_TRUE(registry.Load(conf));

  const auto* found = registry.Find("base_link", "cam");
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->resolved_file_path, overlay_file.string());

  const auto enabled = registry.EnabledEntries();
  ASSERT_EQ(enabled.size(), 1u);
  EXPECT_EQ(enabled[0].child_frame_id, "cam");
  EXPECT_TRUE(enabled[0].enable);
}

TEST_F(CalibrationRegistryTest, FiltersDisabledEntries) {
  apollo::static_transform::Conf conf;
  auto* entry1 = conf.add_extrinsic_file();
  entry1->set_frame_id("base_link");
  entry1->set_child_frame_id("lidar");
  entry1->set_file_path("/apollo/lidar.yaml");
  entry1->set_enable(false);

  auto* entry2 = conf.add_extrinsic_file();
  entry2->set_frame_id("base_link");
  entry2->set_child_frame_id("radar");
  entry2->set_file_path("/apollo/radar.yaml");
  entry2->set_enable(true);

  CalibrationRegistry registry;
  ASSERT_TRUE(registry.Load(conf));

  EXPECT_EQ(registry.AllEntries().size(), 2u);
  const auto enabled = registry.EnabledEntries();
  ASSERT_EQ(enabled.size(), 1u);
  EXPECT_EQ(enabled[0].child_frame_id, "radar");

  const auto* by_child = registry.FindByChildFrame("radar");
  ASSERT_NE(by_child, nullptr);
  EXPECT_EQ(by_child->frame_id, "base_link");
  EXPECT_EQ(registry.FindByChildFrame("unknown"), nullptr);
}

TEST_F(CalibrationRegistryTest, ResolvesIntrinsicPathsAcrossOverlay) {
  const std::string rel_path =
      "modules/perception/data/params/front_6mm_intrinsics.yaml";
  const fs::path overlay_intrinsic = temp_dir_ / "overlay" / rel_path;
  fs::create_directories(overlay_intrinsic.parent_path());
  {
    std::ofstream ofs(overlay_intrinsic);
    ofs << "K: [1,0,0,0,1,0,0,0,1]\n";
  }

  const std::string resolved =
      CalibrationRegistry::ResolveIntrinsicPath("front_6mm");
  EXPECT_EQ(resolved, overlay_intrinsic.string());
}

}  // namespace
}  // namespace transform
}  // namespace apollo

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  std::fflush(nullptr);
  std::_Exit(result);
}
