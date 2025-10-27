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

//  Created Date: 2025-10-25
//  Author: daohu527 <daohu527@gmail.com>

#include "cyber/common/file.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "cyber/proto/unit_test.pb.h"

namespace apollo {
namespace cyber {
namespace common {

namespace fs = std::filesystem;
using ::testing::Contains;
using ::testing::ElementsAre;
using ::testing::IsEmpty;
using ::testing::Not;
using ::testing::UnorderedElementsAre;

class FileTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create unique, isolated sandbox directories for each test
    const auto* test_info =
        ::testing::UnitTest::GetInstance()->current_test_info();
    test_root_ =
        fs::temp_directory_path() / "file_test_root" / test_info->name();
    fs::remove_all(test_root_);
    fs::create_directories(test_root_);
  }

  void TearDown() override {
    std::error_code ec;
    fs::remove_all(test_root_, ec);
    ASSERT_FALSE(ec) << "TearDown failed to clean up: " << test_root_.string();
  }

  // Helper to get full path within the test sandbox
  fs::path GetTestPath(const std::string& relative_path) const {
    return test_root_ / relative_path;
  }

  fs::path test_root_;
};

TEST_F(FileTest, ProtoIOCombined) {
  proto::UnitTest message;
  message.set_class_name("ProtoTest");
  const auto ascii_path = GetTestPath("message.ascii");
  const auto bin_path = GetTestPath("message.bin");
  const auto json_path = GetTestPath("message.json");

  // Testing ASCII reading and writing
  ASSERT_TRUE(SetProtoToASCIIFile(message, ascii_path.string()));
  proto::UnitTest read_ascii;
  ASSERT_TRUE(GetProtoFromASCIIFile(ascii_path.string(), &read_ascii));
  EXPECT_EQ(read_ascii.class_name(), "ProtoTest");

  // Testing Binary reading and writing
  ASSERT_TRUE(SetProtoToBinaryFile(message, bin_path.string()));
  proto::UnitTest read_bin;
  ASSERT_TRUE(GetProtoFromBinaryFile(bin_path.string(), &read_bin));
  EXPECT_EQ(read_bin.class_name(), "ProtoTest");

  // Testing JSON reading and writing
  {
    std::ofstream ofs(json_path);
    ofs << R"({"className": "JsonTest"})";
  }
  proto::UnitTest read_json;
  ASSERT_TRUE(GetProtoFromJsonFile(json_path.string(), &read_json));
  EXPECT_EQ(read_json.class_name(), "JsonTest");

  // Testing malformed JSON handling
  {
    std::ofstream ofs(json_path);
    ofs << R"({"className": )";
  }
  EXPECT_FALSE(GetProtoFromJsonFile(json_path.string(), &read_json));
}

TEST_F(FileTest, ContentAndExistence) {
  const auto dir_path = GetTestPath("a_dir");
  const auto file_path = GetTestPath("a_file.txt");
  fs::create_directory(dir_path);
  const std::string content_to_write = "Hello, Cyber!";
  {
    std::ofstream ofs(file_path);
    ofs << content_to_write;
  }

  // Testing PathExists and DirectoryExists
  EXPECT_TRUE(PathExists(dir_path.string()));
  EXPECT_TRUE(PathExists(file_path.string()));
  EXPECT_FALSE(PathExists(GetTestPath("non_existent").string()));
  EXPECT_TRUE(DirectoryExists(dir_path.string()));
  EXPECT_FALSE(DirectoryExists(file_path.string()));

  // Testing GetContent
  std::string read_content;
  ASSERT_TRUE(GetContent(file_path.string(), &read_content));
  EXPECT_EQ(read_content, content_to_write);
  EXPECT_FALSE(GetContent("non_existent_file", &read_content));
}

TEST_F(FileTest, DirectoryModification) {
  // Testing CreateDirectory
  const auto single_dir = GetTestPath("single_dir");
  EXPECT_TRUE(CreateDirectory(single_dir.string()));
  EXPECT_TRUE(fs::is_directory(single_dir));
  EXPECT_TRUE(CreateDirectory(single_dir.string()));  // 幂等性

  // Testing CreateDirectories
  const auto nested_dir = GetTestPath("a/b/c");
  EXPECT_TRUE(CreateDirectories(nested_dir.string()));
  EXPECT_TRUE(fs::is_directory(nested_dir));

  // Testing Remove
  EXPECT_FALSE(Remove(GetTestPath("a").string()));
  EXPECT_TRUE(fs::exists(GetTestPath("a")));
}

TEST_F(FileTest, RemoveAll_Functionality) {
  const auto non_empty_dir = GetTestPath("non_empty_dir");
  fs::create_directories(non_empty_dir / "sub/folder");
  {
    std::ofstream ofs(non_empty_dir / "sub/file.txt");
    ofs << "data";
  }

  // Successfully deleted a non-empty directory
  EXPECT_TRUE(RemoveAll(non_empty_dir.string()));
  EXPECT_FALSE(fs::exists(non_empty_dir));

  // Idempotence: Deleting a non-existent directory also returns true
  EXPECT_TRUE(RemoveAll(non_empty_dir.string()));
}

TEST_F(FileTest, RemoveAll_SafetyChecks) {
  // Basic illegal path
  EXPECT_FALSE(RemoveAll(""));
  EXPECT_FALSE(RemoveAll("/"));

  // Switch to the sandbox directory to test relative path protection
  const auto original_cwd = fs::current_path();
  fs::current_path(test_root_);

  // Verify protection against the current directory
  EXPECT_FALSE(RemoveAll("."));
  EXPECT_TRUE(fs::exists("."));  // Verify the directory was not deleted

  // Verify protection against the parent directory
  EXPECT_FALSE(RemoveAll(".."));
  EXPECT_TRUE(fs::exists(".."));  // Verify the directory was not deleted

  // Switch back to the original directory
  fs::current_path(original_cwd);
}

TEST_F(FileTest, CopyFileAndDir) {
  const auto from_file = GetTestPath("from.txt");
  const auto to_file = GetTestPath("to.txt");

  {
    std::ofstream ofs(from_file);
    ofs << "data";
  }

  ASSERT_TRUE(CopyFile(from_file.string(), to_file.string()));
  ASSERT_TRUE(fs::exists(to_file));
  std::string content;
  ASSERT_TRUE(GetContent(to_file.string(), &content));
  EXPECT_EQ(content, "data");

  const auto from_dir = GetTestPath("from_dir");
  const auto to_dir = GetTestPath("to_dir");
  fs::create_directories(from_dir / "sub");
  {
    std::ofstream ofs(from_dir / "f.txt");
    ofs << "sub-data";
  }

  ASSERT_TRUE(CopyDir(from_dir.string(), to_dir.string()));
  EXPECT_TRUE(fs::is_directory(to_dir / "sub"));
  EXPECT_TRUE(fs::exists(to_dir / "f.txt"));
}

TEST_F(FileTest, Enumeration) {
  const auto dir1 = GetTestPath("dir1");
  const auto file1 = GetTestPath("file1.txt");
  const auto file2 = GetTestPath("file2.log");
  fs::create_directory(dir1);
  { std::ofstream ofs(file1); }
  { std::ofstream ofs(file2); }

  EXPECT_THAT(
      ListSubPaths(test_root_.string(), FileTypeFilter::All),
      UnorderedElementsAre(dir1.string(), file1.string(), file2.string()));
  EXPECT_THAT(ListSubPaths(test_root_.string(), FileTypeFilter::Directories),
              ElementsAre(dir1.string()));
  EXPECT_THAT(ListSubPaths(test_root_.string(), FileTypeFilter::Files),
              UnorderedElementsAre(file1.string(), file2.string()));

  // Glob
  EXPECT_THAT(Glob((test_root_ / "*.txt").string()),
              ElementsAre(file1.string()));
  EXPECT_THAT(Glob((test_root_ / "*.*").string()),
              UnorderedElementsAre(file1.string(), file2.string()));
  EXPECT_THAT(Glob((test_root_ / "file?.log").string()),
              ElementsAre(file2.string()));
}

TEST_F(FileTest, GetAbsolutePath) {
  // The relative path is already an absolute path
  EXPECT_EQ("/var/log", GetAbsolutePath("/home/work", "/var/log"));
  EXPECT_EQ("/var/log", GetAbsolutePath("/home/work", "/var/lib/../log"));

  // Basic concatenation
  // Note: weakly_canonical may not change the path without actually creating
  // the directory For test stability, we assume the path format is canonical
  EXPECT_EQ(fs::path("/home/work/data.txt").string(),
            GetAbsolutePath("/home/work", "data.txt"));

  // Empty prefix, based on the current working directory
  const std::string expected_path =
      fs::weakly_canonical(fs::current_path() / "xx.txt").string();
  EXPECT_EQ(expected_path, GetAbsolutePath("", "xx.txt"));
}

}  // namespace common
}  // namespace cyber
}  // namespace apollo
