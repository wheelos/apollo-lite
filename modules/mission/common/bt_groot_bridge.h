// Copyright 2025 WheelOS All Rights Reserved.
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

//  Created Date: 2025-12-13
//  Author: daohu527

#pragma once

#include <memory>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "cyber/common/log.h"

namespace apollo {
namespace mission {

class GrootBridge {
 public:
  GrootBridge() = default;
  ~GrootBridge() { Reset(); }

  void Connect(BT::Tree& tree, unsigned port = 1667) {
    try {
      publisher_.reset();

      publisher_ = std::make_unique<BT::Groot2Publisher>(tree, port);
      AINFO << "Groot2 Publisher started on port " << port;
    } catch (const std::exception& e) {
      AERROR << "Failed to start Groot2 Publisher: " << e.what();
    }
  }

  void Reset() { publisher_.reset(); }

 private:
  std::unique_ptr<BT::Groot2Publisher> publisher_;
};

}  // namespace mission
}  // namespace apollo
