// Copyright 2026 WheelOS. All Rights Reserved.
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

//  Created Date: 2026-02-21
//  Author: daohu527

#include "modules/dreamview/backend/handlers/map_data_handler.h"

#include <limits.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

namespace apollo {
namespace dreamview {

// URL-decode a percent-encoded string. Returns an empty string on malformed
// percent-encoding.
static std::string UrlDecode(const std::string &src) {
  std::string out;
  out.reserve(src.size());
  for (size_t i = 0; i < src.size(); ++i) {
    char c = src[i];
    if (c == '+') {
      out.push_back(' ');
    } else if (c == '%' && i + 2 < src.size()) {
      auto hex = src.substr(i + 1, 2);
      char *end = nullptr;
      long val = strtol(hex.c_str(), &end, 16);
      if (end == hex.c_str() || *end != '\0' || val < 0 || val > 255) {
        return std::string();
      }
      out.push_back(static_cast<char>(val));
      i += 2;
    } else if (c == '%') {
      return std::string();
    } else {
      out.push_back(c);
    }
  }
  return out;
}

// Send a minimal HTTP error response. Returns nothing; caller should return
// true from the handler to indicate the request was handled.
static void SendHttpError(struct mg_connection *conn, int code,
                          const char *reason) {
  const std::string body = reason;
  mg_printf(conn,
            "HTTP/1.1 %d %s\r\nContent-Type: text/plain; charset=utf-8\r\n"
            "Content-Length: %zu\r\n\r\n%s",
            code, reason, body.size(), body.c_str());
}

MapDataHandler::MapDataHandler(const std::string &map_dir) : map_dir_(map_dir) {
  if (!map_dir_.empty() && map_dir_.back() != '/') {
    map_dir_.push_back('/');
  }
}

bool MapDataHandler::handleGet(CivetServer * /*server*/,
                               struct mg_connection *conn) {
  const struct mg_request_info *request_info = mg_get_request_info(conn);
  if (request_info == nullptr || request_info->local_uri == nullptr) {
    SendHttpError(conn, 400, "Bad Request");
    return true;
  }

  const std::string uri =
      request_info->local_uri;  // e.g. /assets/map_data/abc.bin

  // Map requests under /assets/map_data/ to filesystem directory.
  const std::string prefix = "/assets/map_data/";
  if (uri.compare(0, prefix.length(), prefix) != 0) {
    // Not handled by this handler; let other handlers try.
    return false;
  }

  std::string relative_path = uri.substr(prefix.length());
  if (relative_path.empty()) {
    SendHttpError(conn, 400, "Bad Request");
    return true;
  }

  // Decode percent-encoding and validate path segments to prevent traversal.
  std::string decoded = UrlDecode(relative_path);
  if (decoded.empty()) {
    SendHttpError(conn, 400, "Bad Request");
    return true;
  }
  // Reject attempts to traverse up the tree or absolute paths.
  if (decoded.find("..") != std::string::npos ||
      (!decoded.empty() && decoded[0] == '/')) {
    SendHttpError(conn, 403, "Forbidden");
    return true;
  }

  // Build candidate physical path and resolve real paths to avoid symlink
  // escapes.
  std::string candidate = map_dir_ + decoded;

  char resolved_map[PATH_MAX];
  if (realpath(map_dir_.c_str(), resolved_map) == nullptr) {
    SendHttpError(conn, 500, "Server Error");
    return true;
  }

  char resolved_file[PATH_MAX];
  if (realpath(candidate.c_str(), resolved_file) == nullptr) {
    SendHttpError(conn, 404, "Not Found");
    return true;
  }

  std::string resolved_map_s(resolved_map);
  if (resolved_map_s.back() != '/') resolved_map_s.push_back('/');
  std::string resolved_file_s(resolved_file);

  // Ensure the requested file is still inside the map directory.
  if (resolved_file_s.rfind(resolved_map_s, 0) != 0) {
    SendHttpError(conn, 403, "Forbidden");
    return true;
  }

  // Ensure it's a regular file.
  struct stat st;
  if (stat(resolved_file_s.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
    SendHttpError(conn, 404, "Not Found");
    return true;
  }

  // Serve the file.
  mg_send_file(conn, resolved_file_s.c_str());
  return true;
}

}  // namespace dreamview
}  // namespace apollo
