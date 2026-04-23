// Copyright 2025 WheelOS. All Rights Reserved.
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

//  Created Date: 2026-01-03
//  Author: daohu527

#pragma once

#include <cstdint>

namespace apollo {
namespace control {

// System Safety State (FSM)
enum class SafetyState {
  kNormal = 0,     // Normal driving
  kWarning = 1,    // Warning (Degradation, e.g., speed limit)
  kSoftStop = 2,   // Soft Stop (Comfortable deceleration, lane keeping)
  kHardEstop = 3,  // Emergency Stop (Max braking, Hazard lights)
  kFatal = 4       // Fatal (Power cut-off, Watchdog reset)
};

// Macro to generate Fault ID: Module(1byte) | Category(1byte) | ID(2bytes)
// Control Module ID assumed to be 0x01
#define CONTROL_FAULT_ID(cat, id) ((0x01 << 24) | ((cat) << 16) | (id))

}  // namespace control
}  // namespace apollo
