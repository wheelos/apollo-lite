/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/msf/common/util/file_utility.h"

#include <dirent.h>

#include <algorithm>
#include <cerrno>
#include <iostream>
#include <limits>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include "cyber/common/log.h"
#include "modules/localization/msf/common/util/md5.h"

namespace apollo {
namespace localization {
namespace msf {
const size_t kBufferSize = 20480000;
namespace {

bool ComputeFileMd5Internal(const std::string &file_path,
                            unsigned char res[FileUtility::kUcharMd5Length]) {
  FILE *file = fopen(file_path.c_str(), "rb");
  if (file == nullptr) {
    AERROR << "Can't find the file: " << file_path;
    std::memset(res, 0, FileUtility::kUcharMd5Length);
    return false;
  }

  MD5 md5;
  md5.init();

  std::vector<unsigned char> buf(kBufferSize);
  while (true) {
    const size_t read_size =
        fread(buf.data(), sizeof(unsigned char), buf.size(), file);
    if (read_size > 0) {
      md5.update(buf.data(), static_cast<unsigned int>(read_size));
    }
    if (read_size < buf.size()) {
      if (ferror(file)) {
        AERROR << "Read file failed: " << file_path;
        std::memset(res, 0, FileUtility::kUcharMd5Length);
        fclose(file);
        return false;
      }
      break;
    }
  }

  md5.finalize();
  std::memcpy(res, md5.digest, FileUtility::kUcharMd5Length);
  fclose(file);
  return true;
}

}  // namespace

void FileUtility::ComputeFileMd5(const std::string &file_path,
                                 unsigned char res[kUcharMd5Length]) {
  ComputeFileMd5Internal(file_path, res);
}

void FileUtility::ComputeFileMd5(const std::string &file_path,
                                 char res[kCharMd5Lenth]) {
  unsigned char md[kUcharMd5Length] = {0};
  ComputeFileMd5Internal(file_path, md);
  for (size_t i = 0; i < kUcharMd5Length; ++i) {
    std::snprintf(res + i * 2, 3, "%02X", md[i]);
  }
  res[kCharMd5Lenth - 1] = '\0';
}

void FileUtility::ComputeBinaryMd5(const unsigned char *binary, size_t size,
                                   unsigned char res[kUcharMd5Length]) {
  if (binary == nullptr && size > 0) {
    AERROR << "Null binary input for md5.";
    std::memset(res, 0, kUcharMd5Length);
    return;
  }

  MD5 md5;
  md5.init();

  const size_t kMaxChunkSize = std::numeric_limits<unsigned int>::max();
  size_t offset = 0;
  while (offset < size) {
    const size_t chunk_size = std::min(kMaxChunkSize, size - offset);
    md5.update(binary + offset, static_cast<unsigned int>(chunk_size));
    offset += chunk_size;
  }

  md5.finalize();
  for (uint8_t i = 0; i < kUcharMd5Length; ++i) {
    res[i] = md5.digest[i];
  }
}

void FileUtility::ComputeBinaryMd5(const unsigned char *binary, size_t size,
                                   char res[kCharMd5Lenth]) {
  unsigned char md[kUcharMd5Length] = {0};

  ComputeBinaryMd5(binary, size, md);

  for (size_t i = 0; i < kUcharMd5Length; ++i) {
    std::snprintf(res + i * 2, 3, "%02X", md[i]);
  }
  res[kCharMd5Lenth - 1] = '\0';
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
