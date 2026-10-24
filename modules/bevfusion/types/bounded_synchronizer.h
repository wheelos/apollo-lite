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

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "modules/bevfusion/types/bevfusion_input.h"

namespace apollo {
namespace bevfusion {

struct CameraPacket {
  std::uint64_t timestamp_ns = 0;
  std::uint64_t arrival_monotonic_ns = 0;
  CameraImageView image;
  std::shared_ptr<const std::vector<std::uint8_t>> storage;
};

struct LidarPacket {
  std::uint64_t timestamp_ns = 0;
  std::uint64_t arrival_monotonic_ns = 0;
  std::size_t point_count = 0;
  std::shared_ptr<const std::vector<float>> packed_points;
};

struct SynchronizedPackets {
  std::shared_ptr<const LidarPacket> lidar;
  std::array<std::shared_ptr<const CameraPacket>, kCameraCount> cameras;
};

enum class SyncResult { kReady, kWaiting, kRejected };

class BoundedSynchronizer {
 public:
  bool Init(std::size_t queue_capacity, std::uint64_t max_skew_ns,
            std::uint64_t max_wait_ns, std::string* error);

  bool PushCamera(std::size_t camera_index,
                  std::shared_ptr<const CameraPacket> packet,
                  std::string* error);
  bool PushLidar(std::shared_ptr<const LidarPacket> packet, std::string* error);

  SyncResult TryPop(std::uint64_t now_monotonic_ns,
                    SynchronizedPackets* packets, std::string* diagnostic);

 private:
  std::size_t queue_capacity_ = 0;
  std::uint64_t max_skew_ns_ = 0;
  std::uint64_t max_wait_ns_ = 0;
  std::array<std::deque<std::shared_ptr<const CameraPacket>>, kCameraCount>
      camera_queues_;
  std::deque<std::shared_ptr<const LidarPacket>> lidar_queue_;
};

}  // namespace bevfusion
}  // namespace apollo
