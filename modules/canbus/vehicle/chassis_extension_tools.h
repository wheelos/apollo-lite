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

//  Created Date: 2026-03-06
//  Author: daohu527

#pragma once

#include <utility>

namespace apollo {
namespace canbus {

template <typename Extension, typename ChassisDetailLike>
bool HasChassisExtension(const ChassisDetailLike& chassis_detail) {
  if (!chassis_detail.has_chassis_extension()) {
    return false;
  }
  Extension extension;
  return chassis_detail.chassis_extension().UnpackTo(&extension);
}

template <typename Extension, typename ChassisDetailLike>
Extension GetChassisExtensionOrDefault(
    const ChassisDetailLike& chassis_detail) {
  Extension extension;
  if (chassis_detail.has_chassis_extension()) {
    chassis_detail.chassis_extension().UnpackTo(&extension);
  }
  return extension;
}

template <typename Extension, typename ChassisDetailLike>
class ChassisExtensionWriter {
 public:
  explicit ChassisExtensionWriter(ChassisDetailLike* chassis_detail)
      : chassis_detail_(chassis_detail) {
    if (chassis_detail_ != nullptr &&
        chassis_detail_->has_chassis_extension()) {
      chassis_detail_->chassis_extension().UnpackTo(&extension_);
    }
  }

  ChassisExtensionWriter(const ChassisExtensionWriter&) = delete;
  ChassisExtensionWriter& operator=(const ChassisExtensionWriter&) = delete;

  ChassisExtensionWriter(ChassisExtensionWriter&& other) noexcept
      : chassis_detail_(other.chassis_detail_),
        extension_(std::move(other.extension_)),
        dirty_(other.dirty_) {
    other.chassis_detail_ = nullptr;
    other.dirty_ = false;
  }

  ChassisExtensionWriter& operator=(ChassisExtensionWriter&& other) noexcept {
    if (this == &other) {
      return *this;
    }
    Flush();
    chassis_detail_ = other.chassis_detail_;
    extension_ = std::move(other.extension_);
    dirty_ = other.dirty_;
    other.chassis_detail_ = nullptr;
    other.dirty_ = false;
    return *this;
  }

  ~ChassisExtensionWriter() { Flush(); }

  Extension* operator->() {
    dirty_ = true;
    return &extension_;
  }

  Extension& Mutable() {
    dirty_ = true;
    return extension_;
  }

 private:
  void Flush() {
    if (dirty_ && chassis_detail_ != nullptr) {
      chassis_detail_->mutable_chassis_extension()->PackFrom(extension_);
      dirty_ = false;
    }
  }

 private:
  ChassisDetailLike* chassis_detail_ = nullptr;
  Extension extension_;
  bool dirty_ = false;
};

template <typename Extension, typename ChassisDetailLike>
ChassisExtensionWriter<Extension, ChassisDetailLike> MutableChassisExtension(
    ChassisDetailLike* chassis_detail) {
  return ChassisExtensionWriter<Extension, ChassisDetailLike>(chassis_detail);
}

}  // namespace canbus
}  // namespace apollo
