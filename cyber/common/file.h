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

/**
 * @file
 */

#ifndef CYBER_COMMON_FILE_H_
#define CYBER_COMMON_FILE_H_

#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include <google/protobuf/message.h>

namespace apollo {
namespace cyber {
namespace common {

// Enums for path status and filtering.
enum class PathStatus { NotFound, IsRegularFile, IsDirectory, IsOther, Error };
enum class FileTypeFilter { All, Files, Directories };

// ===================================================================
//                        Path and Name Utilities
// ===================================================================

std::string GetAbsolutePath(const std::string& prefix,
                            const std::string& relative_path);
std::string GetFileName(const std::string& path_str,
                        bool remove_extension = false);
std::string GetCurrentPath();

// ===================================================================
//                 Path Status and Query Utilities
// ===================================================================

bool PathExists(const std::string& path);
bool DirectoryExists(const std::string& directory_path);
PathStatus GetPathStatus(const std::filesystem::path& path,
                         std::error_code& ec);
bool EnsureDirectory(const std::string& directory_path);

// ===================================================================
//                   File Content I/O Utilities
// ===================================================================

bool GetContent(const std::string& file_name, std::string* content);
bool SetProtoToASCIIFile(const google::protobuf::Message& message,
                         const std::string& file_name);
bool GetProtoFromASCIIFile(const std::string& file_name,
                           google::protobuf::Message* message);
bool SetProtoToBinaryFile(const google::protobuf::Message& message,
                          const std::string& file_name);
bool GetProtoFromBinaryFile(const std::string& file_name,
                            google::protobuf::Message* message);
bool GetProtoFromJsonFile(const std::string& file_name,
                          google::protobuf::Message* message);
bool GetProtoFromFile(const std::string& file_name,
                      google::protobuf::Message* message);

// ===================================================================
//                 Filesystem Modification Utilities
// ===================================================================

/**
 * @brief Create a single directory. Fails if parent directory does not exist.
 * Mirrors std::filesystem::create_directory.
 * @param path The path of the directory to create.
 * @return True if successful or if the directory already exists and is a
 * directory.
 */
bool CreateDirectory(const std::string& path);

/**
 * @brief Create a directory and all its parent directories if they do not
 * exist. Mirrors std::filesystem::create_directories (like mkdir -p).
 * @param path The path of the directory to create.
 * @return True on success.
 */
bool CreateDirectories(const std::string& path);

/**
 * @brief Copy a file or directory.
 * @param from The source path.
 * @param to The destination path.
 * @param options The copy options from std::filesystem::copy_options.
 * @return True on success.
 */
bool Copy(const std::string& from, const std::string& to,
          std::filesystem::copy_options options =
              std::filesystem::copy_options::recursive);
bool CopyFile(const std::string& from, const std::string& to);
bool CopyDir(const std::string& from, const std::string& to);

/**
 * @brief Remove a file or an empty directory. Fails if directory is not empty.
 * Mirrors std::filesystem::remove.
 * @param path The path to remove.
 * @return True on success or if the path did not exist.
 */
bool Remove(const std::string& path);

/**
 * @brief Remove a file or a directory and all its contents recursively.
 * Mirrors std::filesystem::remove_all.
 * @param path The path to remove.
 * @return True on success or if the path did not exist.
 */
bool RemoveAll(const std::string& path);

// ===================================================================
//                 Filesystem Enumeration Utilities
// ===================================================================

/**
 * @brief Performs a simplified, non-recursive glob search in a directory.
 *
 * This function only evaluates wildcards in the filename portion of the path.
 * It is not a full-featured glob parser. To prevent C++ compilation errors with
 * nested comments, examples containing the asterisk character are omitted.
 *
 * - **Supported Wildcards**:
 *   - The asterisk (star) for matching zero or more characters.
 *   - The question mark (@c ?), matching exactly one character.
 *
 * - **Unsupported Syntax**:
 *   - Character sets (e.g., @c "[a-z]").
 *   - Brace expansion (e.g., @c "{a,b}").
 *   - Recursive directory traversal.
 *
 * @param pattern The input glob pattern. For instance, @c
 * "/path/to/data/file?.bin".
 * @return A std::vector<std::string> containing the full paths of matching
 * entries.
 */
std::vector<std::string> Glob(const std::string& pattern);
std::vector<std::string> ListSubPaths(
    const std::string& directory_path,
    FileTypeFilter filter = FileTypeFilter::All);

}  // namespace common
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_COMMON_FILE_H_
