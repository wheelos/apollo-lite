#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/traffic_light/ports/provider_ports.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class InMemoryDataProviderPort : public IDataProviderPort,
                                 public IFrameInputPort {
 public:
  bool PushCameraFrame(uint64_t frame_id,
                       const CameraFrameState& frame) override {
    frame_id_ = frame_id;
    timestamp_ns_ = frame.timestamp_ns;
    primary_camera_name_ = frame.camera_name;
    camera_frames_.clear();
    camera_frames_.push_back(frame);
    return true;
  }

  bool PopulateFrameData(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }
    context->frame_id = frame_id_;
    context->timestamp = timestamp_ns_;
    context->primary_camera_name = primary_camera_name_;
    context->image_wide = Image();
    context->image_tele = Image();
    context->camera_frames = camera_frames_;
    if (context->primary_camera_name.empty() &&
        !context->camera_frames.empty()) {
      context->primary_camera_name = context->camera_frames.front().camera_name;
    }
    if (!context->camera_frames.empty()) {
      // Keep legacy single-image fields in sync for transitional stages.
      context->image_tele = context->camera_frames.front().image;
    }
    context->status.image_healthy = !context->camera_frames.empty();
    return context->status.image_healthy;
  }

 private:
  uint64_t frame_id_ = 0;
  uint64_t timestamp_ns_ = 0;
  std::string primary_camera_name_;
  std::vector<CameraFrameState> camera_frames_;
};

// 适用于第一阶段：把 pose 查询从 component/stage 中剥离为独立端口。
class StaticPoseProviderPort : public IPoseProviderPort {
 public:
  void SetEgoState(const VehicleState& ego_state) { ego_state_ = ego_state; }

  void SetCameraPoses(
      std::vector<std::pair<std::string, Pose3d>> camera_poses) {
    camera_poses_ = std::move(camera_poses);
  }

  bool PopulatePose(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }
    context->ego_state = ego_state_;
    for (auto& camera_frame : context->camera_frames) {
      for (const auto& item : camera_poses_) {
        if (item.first == camera_frame.camera_name) {
          camera_frame.camera_pose = item.second;
          break;
        }
      }
    }
    context->status.tf_available = context->ego_state.pose.valid;
    if (!context->status.tf_available &&
        context->status.degrade_reason.empty()) {
      context->status.degrade_reason = "pose unavailable";
    }
    return context->status.tf_available;
  }

 private:
  VehicleState ego_state_;
  std::vector<std::pair<std::string, Pose3d>> camera_poses_;
};

// 第一阶段/第二阶段都可用：提供静态或缓存式 HDMap 候选信号。
class CachedMapProviderPort : public IMapProviderPort {
 public:
  void SetSignals(std::vector<SignalCandidate> signals) {
    signals_ = std::move(signals);
  }

  bool PopulateSignals(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }
    if (!signals_.empty()) {
      context->map_signals = signals_;
      context->status.hdmap_available = true;
      if (context->runtime_state != nullptr) {
        context->runtime_state->cached_signals = signals_;
        context->runtime_state->last_signals_ts_sec =
            static_cast<double>(context->timestamp) * 1e-9;
      }
      return true;
    }
    if (context->runtime_state != nullptr &&
        !context->runtime_state->cached_signals.empty()) {
      context->map_signals = context->runtime_state->cached_signals;
      context->status.hdmap_available = true;
      if (context->status.degrade_reason.empty()) {
        context->status.degrade_reason = "hdmap fallback cache";
      }
      return true;
    }
    context->status.hdmap_available = false;
    if (context->status.degrade_reason.empty()) {
      context->status.degrade_reason = "hdmap unavailable";
    }
    return false;
  }

 private:
  std::vector<SignalCandidate> signals_;
};

// 适用于在线/离线混合场景：异步推送 V2X，按帧快照注入当前上下文。
class BufferedV2XProviderPort : public IV2XProviderPort, public IV2XInputPort {
 public:
  void SetMaxBufferSize(size_t max_buffer_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    max_buffer_size_ = max_buffer_size;
    TrimLocked();
  }

  void PushV2XEvidence(const V2XLightEvidence& evidence) override {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.push_back(evidence);
    TrimLocked();
  }

  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
  }

  bool PopulateV2X(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    context->v2x_lights.assign(buffer_.begin(), buffer_.end());
    if (context->runtime_state != nullptr) {
      for (const auto& evidence : buffer_) {
        context->runtime_state->v2x_buffer.push_back(evidence);
      }
      context->runtime_state->TrimV2XBuffer(max_buffer_size_);
    }
    context->status.v2x_available = !context->v2x_lights.empty();
    if (!context->status.v2x_available &&
        context->status.degrade_reason.empty()) {
      context->status.degrade_reason = "v2x unavailable";
    }
    return context->status.v2x_available;
  }

 private:
  void TrimLocked() {
    while (buffer_.size() > max_buffer_size_) {
      buffer_.pop_front();
    }
  }

  std::mutex mutex_;
  size_t max_buffer_size_ = 64;
  std::deque<V2XLightEvidence> buffer_;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
