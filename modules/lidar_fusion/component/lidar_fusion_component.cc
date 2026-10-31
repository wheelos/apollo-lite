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

#include "modules/lidar_fusion/component/lidar_fusion_component.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "google/protobuf/arena.h"

#include "cyber/common/log.h"
#include "modules/lidar_fusion/publication/functional_scene.h"

namespace apollo {
namespace lidar_fusion {
namespace {

double SteadySeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::shared_ptr<FusionOutput> MakeOutput() {
  auto arena = std::make_shared<google::protobuf::Arena>();
  auto* output = google::protobuf::Arena::Create<FusionOutput>(arena.get());
  return std::shared_ptr<FusionOutput>(arena, output);
}

}  // namespace

bool LidarFusionComponent::Init() {
  std::string error;
  if (!GetProtoConfig(&config_) || !fusion_.Init(config_, &error)) {
    AERROR << "Invalid LiDAR fusion configuration: " << error;
    return false;
  }
  buffer_ = std::make_unique<FrameBuffer>(config_);
  if (config_.temporal().enabled()) {
    if (config_.temporal().pose_channel().empty()) {
      AERROR
          << "Temporal fusion needs an explicit continuous local ODOM channel";
      return false;
    }
    pose_reader_ = node_->CreateReader<LocalPose>(
        config_.temporal().pose_channel(),
        [this](const std::shared_ptr<LocalPose>& message) {
          std::lock_guard<std::mutex> lock(buffer_mutex_);
          if (poses_.size() == config_.queue_capacity()) {
            poses_.pop_front();
          }
          poses_.push_back(message);
        });
    if (pose_reader_ == nullptr) {
      return false;
    }
  }
  const auto& publication = config_.publication();
  if (publication.transport() == CORE_POD) {
    const auto create = [&](const std::string& channel) {
      auto writer = node_->CreateWriter<cyber::transport::PodMessage>(channel);
      return writer == nullptr
                 ? nullptr
                 : std::make_unique<FusionPodPublisher>(
                       std::move(writer), publication.max_pod_frame_bytes());
    };
    pod_writer_ = create(config_.output_channel());
    if (pod_writer_ == nullptr) {
      return false;
    }
    if (!publication.diagnostic_channel().empty()) {
      pod_diagnostic_writer_ = create(publication.diagnostic_channel());
      if (pod_diagnostic_writer_ == nullptr) {
        return false;
      }
    }
  } else {
    writer_ = node_->CreateWriter<FusionOutput>(config_.output_channel());
    if (writer_ == nullptr) {
      return false;
    }
    if (!publication.diagnostic_channel().empty()) {
      diagnostic_writer_ =
          node_->CreateWriter<FusionOutput>(publication.diagnostic_channel());
      if (diagnostic_writer_ == nullptr) {
        return false;
      }
    }
  }
  cloud_reader_ = node_->CreateReader<drivers::PointCloud>(
      config_.pointcloud_channel(),
      [this](const std::shared_ptr<drivers::PointCloud>& message) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (!buffer_->PushCloud(message, SteadySeconds())) {
          AWARN
              << "LiDAR fusion rejected a cloud (identity/frame/size/ordering)";
        }
      });
  detection_reader_ = node_->CreateReader<DetectionFrame>(
      config_.detection_channel(),
      [this](const std::shared_ptr<DetectionFrame>& message) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (!buffer_->PushDetection(message)) {
          AWARN << "LiDAR fusion rejected a detection result";
        }
      });
  segmentation_reader_ = node_->CreateReader<SemanticFrame>(
      config_.segmentation_channel(),
      [this](const std::shared_ptr<SemanticFrame>& message) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (!buffer_->PushSemantic(message)) {
          AWARN << "LiDAR fusion rejected a semantic result";
        }
      });
  return cloud_reader_ != nullptr && detection_reader_ != nullptr &&
         segmentation_reader_ != nullptr;
}

void LidarFusionComponent::Clear() {
  cyber::TimerComponent::Clear();
  if (cloud_reader_ != nullptr) {
    cloud_reader_->Shutdown();
  }
  if (detection_reader_ != nullptr) {
    detection_reader_->Shutdown();
  }
  if (segmentation_reader_ != nullptr) {
    segmentation_reader_->Shutdown();
  }
  if (pose_reader_ != nullptr) {
    pose_reader_->Shutdown();
  }
}

bool LidarFusionComponent::PublishMain(
    const std::shared_ptr<FusionOutput>& output) {
  if (pod_writer_ == nullptr) {
    return writer_->Write(output);
  }
  std::string error;
  if (pod_writer_->Publish(*output, &error) == PodPublishResult::REJECTED) {
    AERROR << "LiDAR fusion POD publication failed: " << error;
    return false;
  }
  return true;
}

bool LidarFusionComponent::Proc() {
  std::lock_guard<std::mutex> processing(process_mutex_);
  ReadyFrame ready;
  uint64_t dropped_clouds = 0;
  uint64_t dropped_results = 0;
  uint64_t queue_overflows = 0;
  std::shared_ptr<const LocalPose> pose;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (!buffer_->Pop(SteadySeconds(), &ready)) {
      return true;
    }
    dropped_clouds = buffer_->dropped_clouds();
    dropped_results = buffer_->dropped_results();
    queue_overflows = buffer_->queue_overflows();
    double best = config_.temporal().pose_tolerance();
    for (const auto& candidate : poses_) {
      const double dt =
          std::abs(candidate->stamp() - ready.cloud->measurement_time());
      if (std::isfinite(dt) && dt <= best) {
        best = dt;
        pose = candidate;
      }
    }
    while (!poses_.empty() &&
           poses_.front()->stamp() < ready.cloud->measurement_time() -
                                         config_.temporal().pose_tolerance()) {
      poses_.pop_front();
    }
  }
  auto output = MakeOutput();
  std::string error;
  if (!fusion_.Process(*ready.cloud, ready.detection.get(),
                       ready.segmentation.get(), pose.get(), output.get(),
                       &error)) {
    AERROR << "LiDAR fusion rejected frame: " << error;
    return false;
  }
  output->mutable_health()->set_dropped_clouds(dropped_clouds);
  output->mutable_health()->set_dropped_results(dropped_results);
  output->mutable_health()->set_queue_overflows(queue_overflows);
  if (queue_overflows > 0) {
    output->add_risk_event(INPUT_QUEUE_OVERFLOW);
  }
  if (config_.publication().detail() == FUNCTIONAL_SCENE) {
    auto scene = MakeOutput();
    if (!ProjectFunctionalOutput(config_, *ready.cloud, *output, scene.get(),
                                 &error)) {
      AERROR << "LiDAR fusion publication rejected: " << error;
      return false;
    }
    const bool published = PublishMain(scene);
    if (diagnostic_writer_ != nullptr && diagnostic_writer_->HasReader() &&
        !diagnostic_writer_->Write(output)) {
      AWARN << "LiDAR fusion diagnostic publication failed";
    }
    if (pod_diagnostic_writer_ != nullptr &&
        pod_diagnostic_writer_->HasReader() &&
        pod_diagnostic_writer_->Publish(*output, &error) ==
            PodPublishResult::REJECTED) {
      AWARN << "LiDAR fusion POD diagnostic publication failed: " << error;
    }
    return published;
  }
  return PublishMain(output);
}

}  // namespace lidar_fusion
}  // namespace apollo
