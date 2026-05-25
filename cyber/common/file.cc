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

#include <algorithm>
#include <fstream>
#include <memory>
#include <regex>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace common {

namespace fs = std::filesystem;

namespace {

bool IsDirectory(const fs::path& path) {
  std::error_code ec;
  const bool is_directory = fs::is_directory(path, ec);
  return !ec && is_directory;
}

ProtoFileFormat DetectProtoFileFormat(const fs::path& path) {
  const std::string path_string = path.string();
  const std::string extension = path.extension().string();
  if (extension == ".json") {
    return ProtoFileFormat::Json;
  }
  if (extension == ".bin" || extension == ".pb") {
    return ProtoFileFormat::Binary;
  }
  if (extension == ".txt" || extension == ".textproto" ||
      extension == ".pbtxt" || extension == ".prototxt") {
    return ProtoFileFormat::Text;
  }
  if (path_string.size() >= std::string(".pb.txt").size() &&
      path_string.compare(path_string.size() - std::string(".pb.txt").size(),
                          std::string(".pb.txt").size(), ".pb.txt") == 0) {
    return ProtoFileFormat::Text;
  }
  return ProtoFileFormat::Auto;
}

bool ParseProtoFromFile(const std::string& file_name,
                        google::protobuf::Message* message,
                        ProtoFileFormat format) {
  if (message == nullptr) {
    AERROR << "Output protobuf message pointer is null.";
    return false;
  }
  std::unique_ptr<google::protobuf::Message> parsed_message(message->New());
  if (parsed_message == nullptr) {
    AERROR << "Failed to allocate temporary protobuf message for: " << file_name;
    return false;
  }
  switch (format) {
    case ProtoFileFormat::Text:
      if (!GetProtoFromASCIIFile(file_name, parsed_message.get())) {
        return false;
      }
      break;
    case ProtoFileFormat::Binary:
      if (!GetProtoFromBinaryFile(file_name, parsed_message.get())) {
        return false;
      }
      break;
    case ProtoFileFormat::Json:
      if (!GetProtoFromJsonFile(file_name, parsed_message.get())) {
        return false;
      }
      break;
    case ProtoFileFormat::Auto:
      return false;
  }
  message->CopyFrom(*parsed_message);
  return true;
}

bool MatchesFilter(const fs::directory_entry& entry, FileTypeFilter filter,
                   std::error_code* ec) {
  switch (filter) {
    case FileTypeFilter::All:
      return true;
    case FileTypeFilter::Files:
      return entry.is_regular_file(*ec);
    case FileTypeFilter::Directories:
      return entry.is_directory(*ec);
  }
  return false;
}

}  // namespace

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
                      google::protobuf::Message* message,
                      ProtoFileFormat format) {
  if (message == nullptr) {
    AERROR << "Output protobuf message pointer is null.";
    return false;
  }
  if (!PathExists(file_name)) {
    AERROR << "File does not exist: " << file_name;
    return false;
  }

  const ProtoFileFormat effective_format =
      format == ProtoFileFormat::Auto ? DetectProtoFileFormat(file_name)
                                      : format;
  switch (effective_format) {
    case ProtoFileFormat::Text:
      return ParseProtoFromFile(file_name, message, ProtoFileFormat::Text);
    case ProtoFileFormat::Binary:
      return ParseProtoFromFile(file_name, message, ProtoFileFormat::Binary);
    case ProtoFileFormat::Json:
      return ParseProtoFromFile(file_name, message, ProtoFileFormat::Json);
    case ProtoFileFormat::Auto:
      break;
  }

  for (const auto candidate :
       {ProtoFileFormat::Text, ProtoFileFormat::Binary, ProtoFileFormat::Json}) {
    if (ParseProtoFromFile(file_name, message, candidate)) {
      return true;
    }
  }

  AERROR << "Failed to parse file [" << file_name
         << "] as text, binary, or json format.";
  return false;
}

// ===================================================================
//                 Filesystem Modification Utilities
// ===================================================================

bool CreateDirectory(const std::string& path) {
  if (path.empty()) {
    return false;
  }
  if (IsDirectory(path)) {
    return true;
  }
  std::error_code ec;
  fs::create_directory(path, ec);
  if (ec) {
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
  if (IsDirectory(path)) {
    return true;
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

bool ClearDirectory(const std::string& directory_path) {
  std::error_code ec;
  if (!fs::is_directory(directory_path, ec)) {
    if (ec) {
      AERROR << "Cannot open directory " << directory_path
             << ": " << ec.message();
    } else {
      AERROR << "Cannot open directory " << directory_path;
    }
    return false;
  }

  auto it = fs::directory_iterator(directory_path, ec);
  if (ec) {
    AERROR << "Cannot iterate directory " << directory_path
           << ": " << ec.message();
    return false;
  }

  for (const auto& entry : it) {
    std::error_code remove_ec;
    fs::remove_all(entry.path(), remove_ec);
    if (remove_ec) {
      AERROR << "Fail to remove path " << entry.path().string()
             << ": " << remove_ec.message();
      return false;
    }
  }
  return true;
}

// ===================================================================
//                 Filesystem Enumeration Utilities
// ===================================================================

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

std::vector<std::string> Glob(const std::string& pattern) {
  std::vector<std::string> results;
  glob_t globs = {};
  if (glob(pattern.c_str(), GLOB_TILDE, nullptr, &globs) == 0) {
    for (size_t i = 0; i < globs.gl_pathc; ++i) {
      results.emplace_back(globs.gl_pathv[i]);
    }
  }
  globfree(&globs);
  std::sort(results.begin(), results.end());
  return results;
}

std::vector<fs::path> ListSubPaths(const std::string& directory_path,
                                   FileTypeFilter filter) {
  std::vector<fs::path> result;
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
    std::error_code type_ec;
    const bool match = MatchesFilter(entry, filter, &type_ec);
    if (type_ec) {
      AWARN << "Failed to check type of path " << entry.path().string() << ": "
            << type_ec.message();
      continue;
    }
    if (match) {
      result.push_back(entry.path());
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

}  // namespace common
}  // namespace cyber
}  // namespace apollo
