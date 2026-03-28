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

// 运行期跨帧状态，承载缓存、健康状态与异步消息快照。
class RuntimeState {
public:
    std::map<std::string, double> last_camera_timestamps_sec;
    std::vector<TrackedLight> tracked_memory;
    std::deque<V2XLightEvidence> v2x_buffer;

    uint64_t last_frame_id = 0;
    double last_query_tf_ts_sec = -1.0;
    double last_processed_ts_sec = -1.0;
    double last_signals_ts_sec = -1.0;
    std::vector<SignalCandidate> cached_signals;

    void TrimV2XBuffer(size_t max_size) {
        while (v2x_buffer.size() > max_size) {
            v2x_buffer.pop_front();
        }
    }
};

// 黑板模式上下文：线程安全的数据总线，解耦各算法模块的数据流转
class PipelineContext {
public:
    // --- 1. 外部输入 (单帧内不可变) ---
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

    // --- 2. 线程安全锁  ---
    std::mutex rw_mutex;

    // --- 3. 算法中间产物 (黑板) ---
    std::vector<PromptRegion> prompts;                 // From Prompter Stage
    std::vector<VisualLight> visual_lights;            // From Detector Stage
    std::vector<BoundLight> bound_lights;              // From Binder Stage
    std::vector<TrackedLight> tracked_lights;          // From Tracker Stage
    HeuristicState heuristic_state;                    // From Heuristic Stage

    // --- 4. 链路最终决策 ---
    TrafficLightResult final_decision;                 // From Fusion Stage

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
        visual_lights.clear();
        bound_lights.clear();
        tracked_lights.clear();
        heuristic_state = HeuristicState();
        final_decision = TrafficLightResult();
    }
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo
