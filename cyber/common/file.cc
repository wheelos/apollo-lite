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

#include <fstream>
#include <regex>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace common {

namespace fs = std::filesystem;

// ===================================================================
//                        Path and Name Utilities
// ===================================================================

std::string GetAbsolutePath(const std::string& prefix,
                            const std::string& relative_path) {
  // If relative_path is already an absolute path, just normalize and return it.
  if (!relative_path.empty() && relative_path[0] == '/') {
    return fs::weakly_canonical(fs::path(relative_path)).string();
  }

  // Define the base for our combination. If prefix is empty, use the current
  // path.
  fs::path base_path = prefix.empty() ? fs::current_path() : fs::path(prefix);

  // Combine the base and relative path, and let weakly_canonical do the work.
  return fs::weakly_canonical(base_path / relative_path).string();
}

std::string GetFileName(const std::string& path_str, bool remove_extension) {
  fs::path p(path_str);
  return remove_extension ? p.stem().string() : p.filename().string();
}

std::string GetCurrentPath() {
  std::error_code ec;
  fs::path current_path = fs::current_path(ec);
  if (ec) {
    AERROR << "Failed to get current path: " << ec.message();
    return "";
  }
  return current_path.string();
}

// ===================================================================
//                 Path Status and Query Utilities
// ===================================================================

bool PathExists(const std::string& path) {
  std::error_code ec;
  bool exists = fs::exists(path, ec);
  if (ec) {
    AWARN << "Error checking existence of path '" << path
          << "': " << ec.message();
    return false;
  }
  return exists;
}

bool DirectoryExists(const std::string& directory_path) {
  std::error_code ec;
  bool is_dir = fs::is_directory(directory_path, ec);
  if (ec) {
    AWARN << "Error checking if path '" << directory_path
          << "' is a directory: " << ec.message();
    return false;
  }
  return is_dir;
}

PathStatus GetPathStatus(const fs::path& path, std::error_code& ec) {
  fs::file_status status = fs::status(path, ec);
  if (ec) {
    if (ec == std::errc::no_such_file_or_directory) {
      ec.clear();
      return PathStatus::NotFound;
    }
    AERROR << "Failed to get status for path: " << path
           << ", Error: " << ec.message();
    return PathStatus::Error;
  }
  switch (status.type()) {
    case fs::file_type::regular:
      return PathStatus::IsRegularFile;
    case fs::file_type::directory:
      return PathStatus::IsDirectory;
    case fs::file_type::not_found:
      return PathStatus::NotFound;
    default:
      return PathStatus::IsOther;
  }
}

bool EnsureDirectory(const std::string& directory_path) {
  return CreateDirectories(directory_path);
}

// ===================================================================
//                   File Content I/O Utilities
// ===================================================================

bool GetContent(const std::string& file_name, std::string* content) {
  if (!content) {
    AERROR << "Input content string pointer is null.";
    return false;
  }
  std::ifstream file(file_name, std::ios::binary);
  if (!file) {
    AWARN << "Failed to open file for reading: " << file_name;
    return false;
  }
  content->assign((std::istreambuf_iterator<char>(file)),
                  std::istreambuf_iterator<char>());
  return true;
}

bool SetProtoToASCIIFile(const google::protobuf::Message& message,
                         const std::string& file_name) {
  std::ofstream fs(file_name, std::ios::out | std::ios::trunc);
  if (!fs) {
    AERROR << "Failed to open file for writing: " << file_name;
    return false;
  }
  google::protobuf::io::OstreamOutputStream zcs(&fs);
  return google::protobuf::TextFormat::Print(message, &zcs);
}

bool GetProtoFromASCIIFile(const std::string& file_name,
                           google::protobuf::Message* message) {
  std::ifstream fs(file_name, std::ios::in);
  if (!fs) {
    AWARN << "Failed to open ASCII file for reading: " << file_name;
    return false;
  }
  google::protobuf::io::IstreamInputStream zcs(&fs);
  return google::protobuf::TextFormat::Parse(&zcs, message);
}

bool SetProtoToBinaryFile(const google::protobuf::Message& message,
                          const std::string& file_name) {
  std::ofstream fs(file_name,
                   std::ios::out | std::ios::trunc | std::ios::binary);
  if (!fs) {
    AERROR << "Failed to open file for writing: " << file_name;
    return false;
  }
  return message.SerializeToOstream(&fs);
}

bool GetProtoFromBinaryFile(const std::string& file_name,
                            google::protobuf::Message* message) {
  std::ifstream fs(file_name, std::ios::in | std::ios::binary);
  if (!fs) {
    AWARN << "Failed to open binary file for reading: " << file_name;
    return false;
  }
  return message->ParseFromIstream(&fs);
}

bool GetProtoFromJsonFile(const std::string& file_name,
                          google::protobuf::Message* message) {
  std::string json_content;
  if (!GetContent(file_name, &json_content)) {
    return false;
  }
  google::protobuf::util::JsonParseOptions options;
  options.ignore_unknown_fields = true;
  auto status = google::protobuf::util::JsonStringToMessage(json_content,
                                                            message, options);
  if (!status.ok()) {
    AERROR << "Failed to parse JSON from file '" << file_name
           << "': " << status.ToString();
    return false;
  }
  return true;
}

bool GetProtoFromFile(const std::string& file_name,
                      google::protobuf::Message* message) {
  if (!PathExists(file_name)) {
    AERROR << "File does not exist: " << file_name;
    return false;
  }

  if (GetProtoFromASCIIFile(file_name, message)) {
    return true;
  }
  AWARN << "Failed to parse file [" << file_name
        << "] as ASCII format, trying binary format now.";

  if (GetProtoFromBinaryFile(file_name, message)) {
    return true;
  }

  AERROR << "Failed to parse file [" << file_name
         << "] as both ASCII and binary format.";
  return false;
}

// ===================================================================
//                 Filesystem Modification Utilities
// ===================================================================

bool CreateDirectory(const std::string& path) {
  if (path.empty()) {
    return false;
  }
  std::error_code ec;
  fs::create_directory(path, ec);
  if (ec && ec != std::errc::file_exists) {
    AERROR << "Failed to create directory: " << path
           << ", Error: " << ec.message();
    return false;
  }
  return true;
}

bool CreateDirectories(const std::string& path) {
  if (path.empty()) {
    return false;
  }
  std::error_code ec;
  fs::create_directories(path, ec);
  if (ec) {
    AERROR << "Failed to create directories: " << path
           << ", Error: " << ec.message();
    return false;
  }
  return true;
}

bool Copy(const std::string& from, const std::string& to,
          fs::copy_options options) {
  std::error_code ec;
  fs::copy(from, to, options, ec);
  if (ec) {
    AERROR << "Failed to copy from '" << from << "' to '" << to
           << "', Error: " << ec.message();
    return false;
  }
  return true;
}

bool CopyFile(const std::string& from, const std::string& to) {
  return Copy(from, to, fs::copy_options::overwrite_existing);
}

bool CopyDir(const std::string& from, const std::string& to) {
  return Copy(
      from, to,
      fs::copy_options::recursive | fs::copy_options::overwrite_existing);
}

bool Remove(const std::string& path) {
  std::error_code ec;
  if (!fs::remove(path, ec)) {
    if (ec && ec != std::errc::no_such_file_or_directory) {
      AERROR << "Failed to remove path: " << path
             << ", Error: " << ec.message();
      return false;
    }
  }
  return true;
}

bool RemoveAll(const std::string& path) {
  if (path.empty()) {
    AWARN << "Attempting to remove an empty path.";
    return false;
  }

  std::error_code ec;
  const fs::path p(path);
  fs::path normalized_path = fs::canonical(p, ec);
  if (ec) {
    if (ec == std::errc::no_such_file_or_directory) {
      return true;
    }
    AERROR << "Failed to normalize path for removal: " << path
           << ", Error: " << ec.message();
    return false;
  }

  // Prohibit deletion of root directory
  if (normalized_path == "/") {
    AERROR << "Critical error: Attempting to remove root directory. Aborted.";
    return false;
  }

  // Check if it is the current working directory
  if (normalized_path == fs::current_path(ec)) {
    AWARN << "Attempting to remove current working directory. Aborted.";
    return false;
  }

  std::uintmax_t removed = fs::remove_all(p, ec);
  if (ec) {
    AERROR << "Failed to remove path recursively: " << path
           << ", Error: " << ec.message();
    return false;
  }
  (void)removed;
  return true;
}

// ===================================================================
//                 Filesystem Enumeration Utilities
// ===================================================================

namespace {
std::string WildcardToRegex(const std::string& wildcard) {
  std::string r;
  r.reserve(wildcard.size() * 2);
  for (char c : wildcard) {
    switch (c) {
      case '*':
        r += "[^/]*";
        break;
      case '?':
        r += ".";
        break;
      // Escape all special regex characters.
      case '.':
      case '+':
      case '(':
      case ')':
      case '{':
      case '}':
      case '[':
      case ']':
      case '\\':
      case '|':
      case '^':
      case '$':
        r += '\\';
        r += c;
        break;
      default:
        r += c;
        break;
    }
  }
  return r;
}
}  // namespace

std::vector<std::string> Glob(const std::string& pattern) {
  std::vector<std::string> results;
  const fs::path p(pattern);
  const fs::path dir =
      p.has_parent_path() ? p.parent_path() : fs::current_path();

  if (dir.empty() || !DirectoryExists(dir.string())) {
    return results;
  }

  const std::string fname_pattern = p.filename().string();

  try {
    const std::regex matcher(WildcardToRegex(fname_pattern));
    for (const auto& entry : fs::directory_iterator(dir)) {
      if (std::regex_match(entry.path().filename().string(), matcher)) {
        results.push_back(entry.path().string());
      }
    }
  } catch (const std::regex_error& e) {
    AERROR << "Invalid glob pattern: " << pattern
           << ", regex error: " << e.what();
  }
  return results;
}

std::vector<std::string> ListSubPaths(const std::string& directory_path,
                                      FileTypeFilter filter) {
  std::vector<std::string> result;
  std::error_code ec;

  if (!DirectoryExists(directory_path)) {
    AWARN << "Cannot open non-existent directory: " << directory_path;
    return result;
  }

  auto it = fs::directory_iterator(directory_path, ec);
  if (ec) {
    AERROR << "Cannot create directory iterator for: " << directory_path
           << ", Error: " << ec.message();
    return result;
  }

  for (const auto& entry : it) {
    bool match = false;
    std::error_code type_ec;
    switch (filter) {
      case FileTypeFilter::All:
        match = true;
        break;
      case FileTypeFilter::Files:
        match = entry.is_regular_file(type_ec);
        break;
      case FileTypeFilter::Directories:
        match = entry.is_directory(type_ec);
        break;
    }
    if (type_ec) {
      AWARN << "Failed to check type of path " << entry.path().string() << ": "
            << type_ec.message();
      continue;
    }
    if (match) {
      result.push_back(entry.path().string());
    }
  }
  return result;
}

}  // namespace common
}  // namespace cyber
}  // namespace apollo
