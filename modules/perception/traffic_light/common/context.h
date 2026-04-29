#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "modules/perception/traffic_light/common/types.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class RuntimeState {
 public:
  std::map<std::string, double> last_camera_timestamps_sec;
  std::vector<TrackedLight> tracked_memory;
  std::deque<V2XLightEvidence> v2x_buffer;
  std::vector<SignalCandidate> cached_signals;

  uint64_t last_frame_id = 0;
  uint64_t dropped_frame_count = 0;
  double last_processed_ts_sec = -1.0;
  double last_signals_ts_sec = -1.0;

  void TrimV2XBuffer(size_t max_size) {
    while (v2x_buffer.size() > max_size) {
      v2x_buffer.pop_front();
    }
  }
};

class PipelineContext {
 public:
  uint64_t frame_id = 0;
  uint64_t timestamp = 0;
  std::string primary_camera_name;
  Image image_wide;
  Image image_tele;
  std::vector<CameraFrameState> camera_frames;
  VehicleState ego_state;
  std::vector<AgentState> surrounding_agents;
  NavTopology nav_topology;
  std::vector<SignalCandidate> map_signals;
  std::vector<V2XLightEvidence> v2x_lights;
  ProcessingStatus status;
  RuntimeState* runtime_state = nullptr;

  std::mutex rw_mutex;

  std::vector<PromptRegion> prompts;
  std::vector<YoloLightCandidate> raw_yolo_lights;
  std::vector<VisualLight> visual_lights;
  std::vector<BoundLight> bound_lights;
  std::vector<TrackedLight> tracked_lights;
  HeuristicState heuristic_state;
  std::vector<TrafficLightResult> final_lights;
  TrafficLightResult primary_decision;
  PipelineRuntimeMetrics metrics;

  void ResetPerFrame() {
    frame_id = 0;
    timestamp = 0;
    primary_camera_name.clear();
    image_wide = Image();
    image_tele = Image();
    camera_frames.clear();
    ego_state = VehicleState();
    surrounding_agents.clear();
    nav_topology = NavTopology();
    map_signals.clear();
    v2x_lights.clear();
    status = ProcessingStatus();
    prompts.clear();
    raw_yolo_lights.clear();
    visual_lights.clear();
    bound_lights.clear();
    tracked_lights.clear();
    heuristic_state = HeuristicState();
    final_lights.clear();
    primary_decision = TrafficLightResult();
    metrics = PipelineRuntimeMetrics();
  }

  void AppendDegradeReason(const std::string& reason) {
    if (reason.empty()) {
      return;
    }
    if (status.degrade_reason.empty()) {
      status.degrade_reason = reason;
      return;
    }
    if (status.degrade_reason.find(reason) != std::string::npos) {
      return;
    }
    status.degrade_reason.append("; ");
    status.degrade_reason.append(reason);
  }
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
