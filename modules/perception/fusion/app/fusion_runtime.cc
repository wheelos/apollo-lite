#include "modules/perception/fusion/app/fusion_runtime.h"

#include <cmath>
#include <memory>
#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace fusion {

bool FusionRuntime::Init(const std::string& main_sensor, double reorder_window,
                         Processor processor, SourceValidator source_validator) {
  if (main_sensor.empty() || !std::isfinite(reorder_window) ||
      reorder_window < 0 || !processor) {
    AERROR << "Invalid fusion runtime configuration.";
    return false;
  }
  main_sensor_ = main_sensor;
  reorder_window_ = reorder_window;
  processor_ = std::move(processor);
  source_validator_ = std::move(source_validator);
  last_tick_ = -std::numeric_limits<double>::infinity();
  source_errors_.clear();
  processed_timestamps_.clear();
  has_publish_pose_ = false;
  faulted_ = false;
  return true;
}

bool FusionRuntime::Add(const base::FramePtr& frame) {
  if (!processor_ || faulted_ || !frame) {
    AERROR << "Fusion runtime cannot admit an observation.";
    return false;
  }
  FusionFrame input;
  input.frame = frame;
  if (!processor_(&input)) return false;
  return true;
}

bool FusionRuntime::Tick(double now, FusionFrame* output) {
  const double timestamp = now - reorder_window_;
  if (output) {
    *output = FusionFrame();
    output->degraded = true;
  }
  if (!processor_ || !output || !std::isfinite(timestamp) ||
      timestamp <= last_tick_) {
    AERROR << "Invalid or non-monotonic fusion publication clock.";
    return false;
  }
  output->frame = std::make_shared<base::Frame>();
  output->frame->timestamp = timestamp;
  output->publish_tick = true;
  last_tick_ = timestamp;
  if (faulted_) {
    output->degraded = true;
    AERROR << "Fusion runtime faulted; restart required.";
    return false;
  }
  if (!processor_(output)) {
    faulted_ = true;
    output->fused_objects.clear();
    output->degraded = true;
    AERROR << "Fusion cycle failed; withholding partial state.";
    return false;
  }
  if (!output->ready || !output->scene_ptr || !output->frame ||
      output->frame->timestamp != timestamp || !output->publish_tick) {
    faulted_ = true;
    output->fused_objects.clear();
    output->degraded = true;
    AERROR << "Fusion cycle did not produce a completed scene.";
    return false;
  }
  for (const auto& frame : output->sensor_frames) {
    if (!frame) {
      faulted_ = true;
      output->fused_objects.clear();
      output->degraded = true;
      AERROR << "Completed fusion cycle contains a null processed frame.";
      return false;
    }
    processed_timestamps_[frame->GetSensorId()] = frame->GetTimestamp();
    if (frame->GetSensorId() == main_sensor_) {
      if (!frame->GetPose(&publish_pose_)) {
        faulted_ = true;
        output->fused_objects.clear();
        output->degraded = true;
        AERROR << "Processed main frame has no publication pose.";
        return false;
      }
      publish_sensor_info_ = frame->GetHeader()->sensor_info;
      has_publish_pose_ = true;
    }
    const auto it = source_errors_.find(frame->GetSensorId());
    if (it != source_errors_.end() && frame->GetTimestamp() > it->second) {
      source_errors_.erase(it);
    }
  }
  output->has_publish_pose = has_publish_pose_;
  if (has_publish_pose_) {
    output->frame->sensor2world_pose = publish_pose_;
    output->frame->sensor_info = publish_sensor_info_;
  }
  output->degraded = output->degraded || !source_errors_.empty();
  return true;
}

bool FusionRuntime::ReportSourceError(const std::string& sensor,
                                      double timestamp) {
  if (!processor_ || sensor.empty() || !std::isfinite(timestamp) || timestamp < 0) {
    AERROR << "Invalid fusion source error header.";
    return false;
  }
  if (source_validator_ && !source_validator_(sensor)) {
    AERROR << "Unknown or disabled fusion source error: " << sensor;
    return false;
  }
  const auto processed = processed_timestamps_.find(sensor);
  if (processed != processed_timestamps_.end() &&
      processed->second > timestamp) {
    AWARN << "Ignoring source error superseded by a newer processed observation.";
    return true;
  }
  const auto it = source_errors_.find(sensor);
  if (it == source_errors_.end() || timestamp > it->second) {
    source_errors_[sensor] = timestamp;
  }
  AWARN << "Fusion input source failed: " << sensor;
  return true;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
