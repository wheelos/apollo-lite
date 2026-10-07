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
#include "modules/perception/lib/config_manager/config_manager.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "gtest/gtest.h"

#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace lib {

class ConfigManagerTest : public testing::Test {
 protected:
  ConfigManagerTest() : config_manager_(NULL) {}
  virtual ~ConfigManagerTest() {}
  virtual void SetUp() {
    namespace fs = std::filesystem;
    const char* test_tmpdir = std::getenv("TEST_TMPDIR");
    ASSERT_NE(test_tmpdir, nullptr);
    test_root_ = fs::path(test_tmpdir) / "config_manager_overlay_test";
    software_root_ = test_root_ / "software";
    config_root_ = test_root_ / "assets/vehicles/cargo/config";
    fs::remove_all(test_root_);
    fs::create_directories(config_root_);

    const std::string model_dir = "modules/perception/testdata/lib/conf";
    WriteFile(software_root_ / model_dir / "config_manager.config",
              "model_config_path: \"" +
                  (software_root_ / model_dir / "model.config").string() +
                  "\"\n");
    WriteFile(software_root_ / model_dir / "model.config",
              "model_configs {\n"
              "  name: \"FrameClassifier\"\n"
              "  integer_params { name: \"threshold1\" value: 1 }\n"
              "  integer_params { name: \"threshold2\" value: 2 }\n"
              "  string_params { name: \"threshold3\" value: \"str3\" }\n"
              "  double_params { name: \"threshold4\" value: 4.0 }\n"
              "  float_params { name: \"threshold5\" value: 5.0 }\n"
              "  bool_params { name: \"bool_value_true\" value: true }\n"
              "  bool_params { name: \"bool_value_false\" value: false }\n"
              "  array_integer_params { name: \"array_p1\" values: 1 "
              "values: 2 values: 3 }\n"
              "  array_string_params { name: \"array_p2\" values: \"str1\" "
              "values: \"str2\" values: \"str3\" values: \"str4\" }\n"
              "  array_double_params { name: \"array_p4\" values: 1.1 "
              "values: 1.2 values: 1.3 values: 1.4 }\n"
              "  array_bool_params { name: \"array_bool\" values: true "
              "values: false values: true values: false }\n"
              "}\n"
              "model_configs { name: \"FrameClassifier2\" }\n");

    ASSERT_EQ(0, setenv("APOLLO_ROOT_DIR", software_root_.c_str(), 1));
    ASSERT_EQ(0, setenv("WHEELOS_CONFIG_ROOT", config_root_.c_str(), 1));
    FLAGS_work_root = software_root_.string();
    FLAGS_config_manager_path =
        (software_root_ / model_dir).string();
    unsetenv("CYBER_PATH");
    unsetenv("MODULE_PATH");
    config_manager_ = ConfigManager::Instance();
    ASSERT_TRUE(config_manager_ != nullptr);
    config_manager_->set_work_root(software_root_.string());
  }

  static void WriteFile(const std::filesystem::path& path,
                        const std::string& contents) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path);
    ASSERT_TRUE(output.is_open()) << path;
    output << contents;
    ASSERT_TRUE(output.good()) << path;
  }

 protected:
  ConfigManager* config_manager_;
  std::filesystem::path test_root_;
  std::filesystem::path software_root_;
  std::filesystem::path config_root_;
};

TEST_F(ConfigManagerTest, ConfigOverlayResolvesIndexAndNestedFiles) {
  namespace fs = std::filesystem;
  const char* test_tmpdir = std::getenv("TEST_TMPDIR");
  ASSERT_NE(test_tmpdir, nullptr);
  const fs::path test_root =
      fs::path(test_tmpdir) / "config_manager_overlay_test";
  const fs::path software_root = test_root / "software";
  const fs::path config_root = test_root / "assets/vehicles/cargo/config";

  const std::string override_dir = "modules/perception/overlay";
  const std::string fallback_dir = "modules/perception/fallback";
  const std::string invalid_dir = "modules/perception/invalid";
  const auto write_config_set = [&](const std::string& relative_dir,
                                    const std::string& model_name) {
    WriteFile(software_root / relative_dir / "config_manager.config",
              "model_config_path: \"" + relative_dir + "/model.config\"\n");
    WriteFile(software_root / relative_dir / "model.config",
              "model_configs { name: \"" + model_name + "\" }\n");
  };

  fs::remove_all(test_root);
  fs::create_directories(config_root);
  write_config_set(override_dir, "DefaultModel");
  write_config_set(fallback_dir, "FallbackModel");
  write_config_set(invalid_dir, "DefaultInvalidModel");
  WriteFile(config_root / override_dir / "config_manager.config",
            "model_config_path: \"modules/perception/overlay/model.config\"\n");
  WriteFile(config_root / override_dir / "model.config",
            "model_configs { name: \"OverlayModel\" }\n");
  WriteFile(config_root / invalid_dir / "config_manager.config",
            "invalid: [\n");

  setenv("APOLLO_ROOT_DIR", software_root.c_str(), 1);
  setenv("WHEELOS_CONFIG_ROOT", config_root.c_str(), 1);
  const std::string original_config_manager_path =
      FLAGS_config_manager_path;

  ConfigManager overlay_manager;
  overlay_manager.set_work_root(software_root.string());
  FLAGS_config_manager_path = override_dir;
  ASSERT_TRUE(overlay_manager.Init());
  const ModelConfig* model_config = nullptr;
  ASSERT_TRUE(overlay_manager.GetModelConfig("OverlayModel", &model_config));
  EXPECT_FALSE(
      overlay_manager.GetModelConfig("DefaultModel", &model_config));

  ConfigManager fallback_manager;
  fallback_manager.set_work_root(software_root.string());
  FLAGS_config_manager_path = fallback_dir;
  ASSERT_TRUE(fallback_manager.Init());
  EXPECT_TRUE(
      fallback_manager.GetModelConfig("FallbackModel", &model_config));

  ConfigManager invalid_manager;
  invalid_manager.set_work_root(software_root.string());
  FLAGS_config_manager_path = invalid_dir;
  EXPECT_FALSE(invalid_manager.Init());

  FLAGS_config_manager_path = original_config_manager_path;
}

TEST_F(ConfigManagerTest, TestInit) {
  config_manager_->inited_ = true;
  EXPECT_TRUE(config_manager_->Init());
  EXPECT_TRUE(config_manager_->Reset());
  config_manager_->set_work_root("");
  EXPECT_TRUE(config_manager_->Init());
  EXPECT_TRUE(config_manager_->Reset());
  EXPECT_EQ(config_manager_->NumModels(), 2u);
  ConfigManager config_manager;
  config_manager.set_work_root("");
  EXPECT_EQ(config_manager.work_root(), "");
}

TEST_F(ConfigManagerTest, TestGetModelConfig) {
  std::string model_name = "FrameClassifier";
  const ModelConfig* model_config = nullptr;

  EXPECT_TRUE(config_manager_->GetModelConfig(model_name, &model_config));
  ASSERT_TRUE(model_config != nullptr);
  EXPECT_EQ(model_config->name(), model_name);

  // not exist model.
  model_config = nullptr;
  EXPECT_FALSE(config_manager_->GetModelConfig("noexist", &model_config));
  EXPECT_EQ(model_config, nullptr);
}

TEST_F(ConfigManagerTest, TestModelConfig) {
  std::string model_name = "FrameClassifier";
  const ModelConfig* model_config = nullptr;
  ASSERT_TRUE(config_manager_->Init());
  ASSERT_EQ(config_manager_->NumModels(), 2u);
  ASSERT_FALSE(
      config_manager_->GetModelConfig("FrameClassifier1", &model_config));
  ASSERT_TRUE(config_manager_->GetModelConfig(model_name, &model_config));
  ASSERT_EQ(model_config->name(), model_name);

  // Check FrameClassifier param map.
  int int_value = 0;
  EXPECT_TRUE(model_config->get_value("threshold1", &int_value));
  EXPECT_EQ(int_value, 1);
  EXPECT_TRUE(model_config->get_value("threshold2", &int_value));
  EXPECT_EQ(int_value, 2);

  std::string str_value;
  EXPECT_TRUE(model_config->get_value("threshold3", &str_value));
  EXPECT_EQ(str_value, "str3");

  double double_value;
  EXPECT_TRUE(model_config->get_value("threshold4", &double_value));
  EXPECT_EQ(double_value, 4.0);

  float float_value;
  EXPECT_TRUE(model_config->get_value("threshold5", &float_value));
  EXPECT_EQ(float_value, 5.0);

  bool bool_value = false;
  EXPECT_TRUE(model_config->get_value("bool_value_true", &bool_value));
  EXPECT_TRUE(bool_value);
  EXPECT_TRUE(model_config->get_value("bool_value_false", &bool_value));
  EXPECT_FALSE(bool_value);

  std::vector<int> int_list;
  EXPECT_TRUE(model_config->get_value("array_p1", &int_list));
  EXPECT_EQ(int_list.size(), 3u);
  EXPECT_EQ(int_list[2], 3);

  std::vector<std::string> str_list;
  EXPECT_TRUE(model_config->get_value("array_p2", &str_list));
  EXPECT_EQ(str_list.size(), 4u);
  EXPECT_EQ(str_list[2], "str3");

  std::vector<double> double_list;
  EXPECT_TRUE(model_config->get_value("array_p4", &double_list));
  EXPECT_EQ(double_list.size(), 4u);
  EXPECT_EQ(double_list[2], 1.3);

  std::vector<bool> bool_list;
  EXPECT_TRUE(model_config->get_value("array_bool", &bool_list));
  EXPECT_EQ(bool_list.size(), 4u);
  EXPECT_TRUE(bool_list[2]);

  // not exist
  EXPECT_FALSE(model_config->get_value("array_p3", &double_list));
  EXPECT_FALSE(model_config->get_value("array_p3", &int_list));
  EXPECT_FALSE(model_config->get_value("array_p1", &str_list));
  EXPECT_FALSE(model_config->get_value("array_p3", &double_value));
  EXPECT_FALSE(model_config->get_value("array_p3", &int_value));
  EXPECT_FALSE(model_config->get_value("array_p3", &str_value));
}

TEST_F(ConfigManagerTest, TestConfigManagerError) {
  ConfigManagerError error("config manager error");
  EXPECT_EQ(error.What(), "config manager error");
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
