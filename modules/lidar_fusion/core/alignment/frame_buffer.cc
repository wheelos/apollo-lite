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

#include "modules/lidar_fusion/core/alignment/frame_buffer.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <utility>

#include "modules/lidar_fusion/core/alignment/sweep_identity.h"

namespace apollo {
namespace lidar_fusion {
namespace {

template <typename Frame>
auto Match(const drivers::PointCloud& cloud,
           const std::deque<std::shared_ptr<const Frame>>& queue) {
  return std::find_if(queue.begin(), queue.end(), [&](const auto& frame) {
    return SameSweep(cloud, frame->header());
  });
}

template <typename Frame>
void RemoveMatch(const drivers::PointCloud& cloud,
                 std::deque<std::shared_ptr<const Frame>>* queue) {
  queue->erase(std::remove_if(queue->begin(), queue->end(),
                              [&](const auto& frame) {
                                return SameSweep(cloud, frame->header());
                              }),
               queue->end());
}

}  // namespace

bool FrameBuffer::PushCloud(std::shared_ptr<const drivers::PointCloud> cloud,
                            double received_sec) {
  if (cloud == nullptr || !std::isfinite(received_sec) ||
      cloud->point_size() == 0 ||
      cloud->point_size() > static_cast<int>(config_.max_points()) ||
      !cloud->has_measurement_time() ||
      !std::isfinite(cloud->measurement_time()) ||
      cloud->measurement_time() <= last_cloud_stamp_ ||
      cloud->frame_id() != config_.sensor_frame() ||
      (cloud->header().has_frame_id() &&
       cloud->header().frame_id() != config_.sensor_frame()) ||
      !SameSweep(*cloud, cloud->header()) ||
      std::any_of(clouds_.begin(), clouds_.end(), [&](const auto& pending) {
        return SameSweep(*pending.cloud, cloud->header());
      })) {
    ++dropped_clouds_;
    return false;
  }
  if (clouds_.size() == config_.queue_capacity()) {
    RemoveMatch(*clouds_.front().cloud, &detections_);
    RemoveMatch(*clouds_.front().cloud, &semantics_);
    clouds_.pop_front();
    ++dropped_clouds_;
    ++queue_overflows_;
  }
  clouds_.push_back({std::move(cloud), received_sec});
  return true;
}

bool FrameBuffer::PushDetection(
    std::shared_ptr<const DetectionFrame> detection) {
  if (detection == nullptr || !detection->has_header() ||
      detection->sensor_frame_id() != config_.sensor_frame() ||
      detection->detection_size() >
          static_cast<int>(config_.max_detections())) {
    ++dropped_results_;
    return false;
  }
  if (detections_.size() == config_.queue_capacity()) {
    detections_.pop_front();
    ++dropped_results_;
    ++queue_overflows_;
  }
  detections_.push_back(std::move(detection));
  return true;
}

bool FrameBuffer::PushSemantic(
    std::shared_ptr<const SemanticFrame> segmentation) {
  if (segmentation == nullptr || !segmentation->has_header() ||
      segmentation->source_frame_id() != config_.sensor_frame() ||
      segmentation->source_topic() != config_.pointcloud_channel() ||
      segmentation->point_label_size() >
          static_cast<int>(config_.max_points())) {
    ++dropped_results_;
    return false;
  }
  if (semantics_.size() == config_.queue_capacity()) {
    semantics_.pop_front();
    ++dropped_results_;
    ++queue_overflows_;
  }
  semantics_.push_back(std::move(segmentation));
  return true;
}

bool FrameBuffer::Pop(double now_sec, ReadyFrame* ready) {
  if (ready == nullptr || !std::isfinite(now_sec)) {
    return false;
  }
  while (!clouds_.empty()) {
    const auto& pending = clouds_.front();
    if (pending.cloud->measurement_time() <= last_cloud_stamp_) {
      RemoveMatch(*pending.cloud, &detections_);
      RemoveMatch(*pending.cloud, &semantics_);
      clouds_.pop_front();
      ++dropped_clouds_;
      continue;
    }
    const auto det = Match(*pending.cloud, detections_);
    const auto seg = Match(*pending.cloud, semantics_);
    if ((det == detections_.end() || seg == semantics_.end()) &&
        now_sec - pending.received_sec < config_.wait_ms() * 0.001) {
      return false;
    }
    ready->cloud = pending.cloud;
    ready->detection = det != detections_.end() ? *det : nullptr;
    ready->segmentation = seg != semantics_.end() ? *seg : nullptr;
    last_cloud_stamp_ = pending.cloud->measurement_time();
    RemoveMatch(*pending.cloud, &detections_);
    RemoveMatch(*pending.cloud, &semantics_);
    clouds_.pop_front();
    return true;
  }
  return false;
}

}  // namespace lidar_fusion
}  // namespace apollo
