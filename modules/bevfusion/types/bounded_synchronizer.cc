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

#include "modules/bevfusion/types/bounded_synchronizer.h"

#include <algorithm>
#include <sstream>

namespace apollo {
namespace bevfusion {

namespace {

void SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

std::uint64_t Distance(std::uint64_t a, std::uint64_t b) {
  return a >= b ? a - b : b - a;
}

}  // namespace

bool BoundedSynchronizer::Init(std::size_t queue_capacity,
                               std::uint64_t max_skew_ns,
                               std::uint64_t max_wait_ns, std::string* error) {
  if (queue_capacity == 0 || max_skew_ns == 0 || max_wait_ns == 0) {
    SetError(
        "queue capacity, timestamp tolerance, and wait bound must be "
        "configured and nonzero",
        error);
    return false;
  }
  queue_capacity_ = queue_capacity;
  max_skew_ns_ = max_skew_ns;
  max_wait_ns_ = max_wait_ns;
  for (auto& queue : camera_queues_) {
    queue.clear();
  }
  lidar_queue_.clear();
  return true;
}

bool BoundedSynchronizer::PushCamera(std::size_t camera_index,
                                     std::shared_ptr<const CameraPacket> packet,
                                     std::string* error) {
  if (queue_capacity_ == 0) {
    SetError("synchronizer is not initialized", error);
    return false;
  }
  if (camera_index >= kCameraCount || packet == nullptr ||
      packet->timestamp_ns == 0 || packet->arrival_monotonic_ns == 0 ||
      packet->storage == nullptr || packet->image.camera_name == nullptr ||
      packet->image.timestamp_ns != packet->timestamp_ns ||
      packet->image.data != packet->storage->data() ||
      packet->image.byte_count != packet->storage->size()) {
    SetError("invalid camera packet or camera index", error);
    return false;
  }
  if (std::string(packet->image.camera_name) != kCameraOrder[camera_index]) {
    SetError("camera packet index does not match required camera order", error);
    return false;
  }
  auto& queue = camera_queues_[camera_index];
  if (queue.size() >= queue_capacity_) {
    SetError(std::string("camera synchronization queue is full: ") +
                 kCameraOrder[camera_index],
             error);
    return false;
  }
  if (!queue.empty() && packet->timestamp_ns <= queue.back()->timestamp_ns) {
    SetError(std::string("out-of-order camera timestamp: ") +
                 kCameraOrder[camera_index],
             error);
    return false;
  }
  queue.push_back(std::move(packet));
  return true;
}

bool BoundedSynchronizer::PushLidar(std::shared_ptr<const LidarPacket> packet,
                                    std::string* error) {
  if (queue_capacity_ == 0) {
    SetError("synchronizer is not initialized", error);
    return false;
  }
  if (packet == nullptr || packet->timestamp_ns == 0 ||
      packet->arrival_monotonic_ns == 0 || packet->point_count == 0 ||
      packet->point_count > kMaxPointCount ||
      packet->packed_points == nullptr ||
      packet->packed_points->size() !=
          packet->point_count * kPointFeatureCount) {
    SetError("invalid LiDAR packet or packed five-feature payload", error);
    return false;
  }
  if (lidar_queue_.size() >= queue_capacity_) {
    SetError("LiDAR synchronization queue is full", error);
    return false;
  }
  if (!lidar_queue_.empty() &&
      packet->timestamp_ns <= lidar_queue_.back()->timestamp_ns) {
    SetError("out-of-order LiDAR timestamp", error);
    return false;
  }
  lidar_queue_.push_back(std::move(packet));
  return true;
}

SyncResult BoundedSynchronizer::TryPop(std::uint64_t now_monotonic_ns,
                                       SynchronizedPackets* packets,
                                       std::string* diagnostic) {
  if (packets == nullptr || queue_capacity_ == 0 || lidar_queue_.empty()) {
    return SyncResult::kWaiting;
  }

  const auto& lidar = lidar_queue_.front();
  std::array<std::size_t, kCameraCount> nearest_indices{};
  bool all_cameras_found = true;
  for (std::size_t camera = 0; camera < kCameraCount; ++camera) {
    const auto& queue = camera_queues_[camera];
    std::uint64_t nearest = UINT64_MAX;
    std::size_t nearest_index = 0;
    bool found = false;
    for (std::size_t index = 0; index < queue.size(); ++index) {
      const auto distance =
          Distance(queue[index]->timestamp_ns, lidar->timestamp_ns);
      if (distance < nearest) {
        nearest = distance;
        nearest_index = index;
        found = true;
      }
    }
    if (!found || nearest > max_skew_ns_) {
      all_cameras_found = false;
      break;
    }
    nearest_indices[camera] = nearest_index;
  }

  const std::uint64_t arrival = lidar->arrival_monotonic_ns;
  const bool timed_out =
      now_monotonic_ns >= arrival && now_monotonic_ns - arrival >= max_wait_ns_;
  if (all_cameras_found && !timed_out) {
    packets->lidar = lidar;
    for (std::size_t camera = 0; camera < kCameraCount; ++camera) {
      auto& queue = camera_queues_[camera];
      const std::size_t selected = nearest_indices[camera];
      packets->cameras[camera] = queue[selected];
      queue.erase(queue.begin() + selected);
    }
    lidar_queue_.pop_front();
    return SyncResult::kReady;
  }

  if (!timed_out) {
    return SyncResult::kWaiting;
  }

  std::ostringstream message;
  bool missing = false;
  if (all_cameras_found) {
    message << "synchronization wait bound " << max_wait_ns_
            << " ns expired before the complete bundle was processed";
  }
  for (std::size_t camera = 0; camera < kCameraCount; ++camera) {
    if (!all_cameras_found && camera_queues_[camera].empty()) {
      if (!all_cameras_found && !missing) {
        message << "timed out waiting for camera ";
      } else {
        message << ", ";
      }
      message << kCameraOrder[camera];
      missing = true;
    }
  }
  if (!missing) {
    message << "camera samples exceeded synchronization tolerance "
            << max_skew_ns_ << " ns for LiDAR timestamp "
            << lidar->timestamp_ns;
  }
  lidar_queue_.pop_front();
  SetError(message.str(), diagnostic);
  return SyncResult::kRejected;
}

}  // namespace bevfusion
}  // namespace apollo
