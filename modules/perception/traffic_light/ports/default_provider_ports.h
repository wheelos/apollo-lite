#pragma once

#include <algorithm>
#include <deque>
#include <map>
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
  void SetFrameStalenessToleranceSec(double tolerance_sec) {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_staleness_tolerance_sec_ = std::max(0.0, tolerance_sec);
  }

  void SetCameraOrder(const std::vector<std::string>& camera_order) {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_order_ = camera_order;
  }

  bool PushCameraFrame(uint64_t frame_id,
                       const CameraFrameState& frame) override {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_id_ = frame_id;
    timestamp_ns_ = frame.timestamp_ns;
    primary_camera_name_ = frame.camera_name;
    latest_frames_[frame.camera_name] = frame;
    return true;
  }

  bool PopulateFrameData(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    context->frame_id = frame_id_;
    context->timestamp = timestamp_ns_;
    context->primary_camera_name = primary_camera_name_;
    context->camera_frames.clear();

    if (primary_camera_name_.empty()) {
      if (!camera_order_.empty()) {
        context->primary_camera_name = camera_order_.front();
      } else if (!latest_frames_.empty()) {
        context->primary_camera_name = latest_frames_.begin()->first;
      }
    }

    const double latest_ts_sec = static_cast<double>(timestamp_ns_) * 1e-9;
    auto append_if_fresh = [&](const std::string& camera_name) {
      const auto it = latest_frames_.find(camera_name);
      if (it == latest_frames_.end()) {
        return;
      }
      const double frame_ts_sec =
          static_cast<double>(it->second.timestamp_ns) * 1e-9;
      if (latest_ts_sec > 0.0 &&
          latest_ts_sec - frame_ts_sec > frame_staleness_tolerance_sec_) {
        return;
      }
      context->camera_frames.push_back(it->second);
    };

    if (!context->primary_camera_name.empty()) {
      append_if_fresh(context->primary_camera_name);
    }
    for (const auto& camera_name : camera_order_) {
      if (camera_name == context->primary_camera_name) {
        continue;
      }
      append_if_fresh(camera_name);
    }
    for (const auto& item : latest_frames_) {
      if (item.first == context->primary_camera_name ||
          std::find(camera_order_.begin(), camera_order_.end(), item.first) !=
              camera_order_.end()) {
        continue;
      }
      append_if_fresh(item.first);
    }

    if (!context->camera_frames.empty()) {
      context->image_tele = context->camera_frames.front().image;
    } else {
      context->image_tele = Image();
    }
    context->image_wide = Image();
    context->status.image_healthy = !context->camera_frames.empty();
    return context->status.image_healthy;
  }

 private:
  std::mutex mutex_;
  uint64_t frame_id_ = 0;
  uint64_t timestamp_ns_ = 0;
  double frame_staleness_tolerance_sec_ = 0.2;
  std::string primary_camera_name_;
  std::vector<std::string> camera_order_;
  std::map<std::string, CameraFrameState> latest_frames_;
};

class StaticPoseProviderPort : public IPoseProviderPort {
 public:
  void SetEgoState(const VehicleState& ego_state) { ego_state_ = ego_state; }

  void SetCameraPoses(
      const std::vector<std::pair<std::string, Pose3d>>& camera_poses) {
    camera_poses_.clear();
    for (const auto& item : camera_poses) {
      camera_poses_[item.first] = item.second;
    }
  }

  bool PopulatePose(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }
    context->ego_state = ego_state_;
    for (auto& camera_frame : context->camera_frames) {
      const auto it = camera_poses_.find(camera_frame.camera_name);
      if (it != camera_poses_.end()) {
        camera_frame.camera_pose = it->second;
      }
    }
    context->status.tf_available = context->ego_state.pose.valid;
    if (!context->status.tf_available) {
      context->AppendDegradeReason("pose unavailable");
    }
    return context->status.tf_available;
  }

 private:
  VehicleState ego_state_;
  std::map<std::string, Pose3d> camera_poses_;
};

class CachedMapProviderPort : public IMapProviderPort {
 public:
  void SetSignals(std::vector<SignalCandidate> signals) {
    signals_ = std::move(signals);
  }

  void SetValidCacheWindowSec(double valid_cache_window_sec) {
    valid_cache_window_sec_ = std::max(0.0, valid_cache_window_sec);
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
      const double frame_ts_sec =
          static_cast<double>(context->timestamp) * 1e-9;
      const double dt =
          frame_ts_sec - context->runtime_state->last_signals_ts_sec;
      if (dt >= 0.0 && dt <= valid_cache_window_sec_) {
        context->map_signals = context->runtime_state->cached_signals;
        context->status.hdmap_available = true;
        context->AppendDegradeReason("hdmap fallback cache");
        return true;
      }
    }

    context->status.hdmap_available = false;
    context->AppendDegradeReason("hdmap unavailable");
    return false;
  }

 private:
  double valid_cache_window_sec_ = 1.5;
  std::vector<SignalCandidate> signals_;
};

class StaticDetectorProviderPort : public IDetectorProviderPort {
 public:
  void SetDetections(std::vector<YoloLightCandidate> detections) {
    detections_ = std::move(detections);
  }

  bool PopulateDetections(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }
    context->raw_yolo_lights = detections_;
    return !context->raw_yolo_lights.empty();
  }

 private:
  std::vector<YoloLightCandidate> detections_;
};

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

  bool PopulateV2X(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    context->v2x_lights.assign(buffer_.begin(), buffer_.end());
    if (context->runtime_state != nullptr) {
      context->runtime_state->v2x_buffer.assign(buffer_.begin(), buffer_.end());
      context->runtime_state->TrimV2XBuffer(max_buffer_size_);
    }
    context->status.v2x_available = !context->v2x_lights.empty();
    if (!context->status.v2x_available) {
      context->AppendDegradeReason("v2x unavailable");
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
