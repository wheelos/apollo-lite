/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/map/tools/bidirectional_lane_converter.h"

#include <filesystem>
#include <string>

#include "gflags/gflags.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

DEFINE_string(input_map_file, "", "input map file");
DEFINE_string(output_map_file, "", "new binary output map file");
DEFINE_string(output_text_map_file, "",
              "optional ASCII output map file for matching binary output");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_input_map_file.empty() || FLAGS_output_map_file.empty()) {
    AERROR << "Both --input_map_file and --output_map_file are required.";
    return 1;
  }

  std::error_code path_error;
  const auto input_path =
      std::filesystem::weakly_canonical(FLAGS_input_map_file, path_error);
  if (path_error) {
    AERROR << "Unable to resolve input map path: " << path_error.message();
    return 1;
  }
  const auto output_path =
      std::filesystem::weakly_canonical(FLAGS_output_map_file, path_error);
  if (path_error) {
    AERROR << "Unable to resolve output map path: " << path_error.message();
    return 1;
  }
  if (input_path == output_path) {
    AERROR << "Input and output map paths must be different.";
    return 1;
  }
  std::filesystem::path text_output_path;
  if (!FLAGS_output_text_map_file.empty()) {
    text_output_path =
        std::filesystem::weakly_canonical(FLAGS_output_text_map_file,
                                           path_error);
    if (path_error) {
      AERROR << "Unable to resolve text output map path: "
             << path_error.message();
      return 1;
    }
    if (text_output_path == input_path || text_output_path == output_path) {
      AERROR << "Input, binary output, and text output paths must be "
                "different.";
      return 1;
    }
  }
  if (std::filesystem::exists(output_path, path_error) || path_error) {
    AERROR << "Output map already exists or cannot be checked: "
           << output_path.string();
    return 1;
  }
  if (!text_output_path.empty() &&
      (std::filesystem::exists(text_output_path, path_error) || path_error)) {
    AERROR << "Text output map already exists or cannot be checked: "
           << text_output_path.string();
    return 1;
  }

  apollo::hdmap::Map map;
  if (!apollo::cyber::common::GetProtoFromFile(input_path.string(), &map)) {
    AERROR << "Failed to load input map: " << input_path.string();
    return 1;
  }

  std::string error;
  if (!apollo::hdmap::tools::ConvertBidirectionalLanes(&map, &error)) {
    AERROR << "Failed to convert bidirectional lanes: " << error;
    return 1;
  }
  if (!apollo::cyber::common::SetProtoToBinaryFile(map, output_path.string())) {
    AERROR << "Failed to write output map: " << output_path.string();
    return 1;
  }
  if (!text_output_path.empty() &&
      !apollo::cyber::common::SetProtoToASCIIFile(map,
                                                   text_output_path.string())) {
    AERROR << "Failed to write text output map: " << text_output_path.string();
    return 1;
  }

  AINFO << "Wrote converted map to " << output_path.string();
  return 0;
}
