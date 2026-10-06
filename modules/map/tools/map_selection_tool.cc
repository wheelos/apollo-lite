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

#include <iostream>
#include <string>

#include "modules/common/map/map_selection.h"

int main(int argc, char** argv) {
  const std::string command = argc > 1 ? argv[1] : "";
  if (argc != 2 ||
      (command != "--print-map-dir" && command != "--print-map-id")) {
    std::cerr << "Usage: map_selection_tool "
                 "(--print-map-dir|--print-map-id)\n";
    return 2;
  }

  apollo::common::SelectedMap selected_map;
  if (!apollo::common::MapSelection::GetSelectedMap(&selected_map)) {
    return 1;
  }
  std::cout << (command == "--print-map-id" ? selected_map.id
                                             : selected_map.directory)
            << '\n';
  return 0;
}
