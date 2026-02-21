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

#pragma once

#include <string>

#include "CivetServer.h"

namespace apollo {
namespace dreamview {

// MapDataHandler serves files under /assets/map_data/ by mapping to a
// filesystem directory.
class MapDataHandler : public CivetHandler {
 public:
  explicit MapDataHandler(
      const std::string &map_dir = "/apollo/modules/map/data/");
  bool handleGet(CivetServer *server, struct mg_connection *conn) override;

 private:
  std::string map_dir_;
};

}  // namespace dreamview
}  // namespace apollo
