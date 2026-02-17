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

//  Created Date: 2026-02-17
//  Author: daohu527

#pragma once

#include <utility>

#include "modules/common_msgs/chassis_msgs/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace vehicle_detail {

template <typename T>
bool Has(const ChassisDetail& chassis) {
  return chassis.has_vehicle_detail() && chassis.vehicle_detail().Is<T>();
}

template <typename T>
const T& Get(const ChassisDetail& chassis) {
  thread_local T cached;
  cached.Clear();
  if (Has<T>(chassis)) {
    chassis.vehicle_detail().UnpackTo(&cached);
  }
  return cached;
}

template <typename T>
bool UnpackTo(const ChassisDetail& chassis, T* detail) {
  if (detail == nullptr) {
    return false;
  }
  detail->Clear();
  if (!Has<T>(chassis)) {
    return false;
  }
  return chassis.vehicle_detail().UnpackTo(detail);
}

template <typename T>
class Editor {
 public:
  explicit Editor(ChassisDetail* chassis) : chassis_(chassis) {
    if (chassis_ != nullptr && Has<T>(*chassis_)) {
      chassis_->vehicle_detail().UnpackTo(&detail_);
    }
  }

  ~Editor() {
    if (chassis_ != nullptr) {
      chassis_->mutable_vehicle_detail()->PackFrom(detail_);
    }
  }

  T* operator->() { return &detail_; }
  T* get() { return &detail_; }

 private:
  ChassisDetail* chassis_ = nullptr;
  T detail_;
};

template <typename T>
Editor<T> Edit(ChassisDetail* chassis) {
  return Editor<T>(chassis);
}

}  // namespace vehicle_detail
}  // namespace canbus
}  // namespace apollo
