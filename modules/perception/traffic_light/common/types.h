#pragma once

#include <cstdint>
#include <string>
#include <vector>


namespace apollo {
namespace perception {
namespace traffic_light {

// --- 基础枚举 ---
enum class LightColor { UNKNOWN = 0, RED = 1, GREEN = 2, YELLOW = 3, BLACK = 4 };
enum class LightShape { UNKNOWN = 0, CIRCLE = 1, ARROW_LEFT = 2, ARROW_RIGHT = 3, ARROW_STRAIGHT = 4 };
enum class LaneIntent { UNKNOWN = 0, STRAIGHT = 1, LEFT = 2, RIGHT = 3, U_TURN = 4 };
enum class EvidenceSource {
    UNKNOWN = 0,
    VISION = 1,
    HDMAP = 2,
    V2X = 3,
    HEURISTIC = 4,
};

// --- 几何与图像抽象  ---
struct Rect2f {
    float x = 0.0f;
    float y = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
};

struct Image {
    void* data = nullptr;
    int rows = 0;
    int cols = 0;
    int channels = 0;
    std::string encoding;
};

struct Pose3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    bool valid = false;
};

// --- 外部依赖状态 (Ego, Agent, Topology) ---
struct VehicleState {
    double timestamp = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    Pose3d pose;
};

struct AgentState {
    int id = 0;
    double velocity = 0.0;
    LaneIntent intent = LaneIntent::UNKNOWN;
    bool is_starting = false;
};

struct NavTopology {
    LaneIntent ego_lane_intent = LaneIntent::UNKNOWN;
    double distance_to_intersection = 0.0;
    bool is_in_intersection = false;
};

struct CameraFrameState {
    std::string camera_name;
    uint64_t timestamp_ns = 0;
    Image image;
    Pose3d camera_pose;
    bool is_working = true;
};

struct SignalCandidate {
    std::string signal_id;
    Rect2f projection_roi;
    float confidence = 0.0f;
    LaneIntent intended_movement = LaneIntent::UNKNOWN;
    double stopline_distance_m = -1.0;
};

struct V2XLightEvidence {
    std::string signal_id;
    LightColor color = LightColor::UNKNOWN;
    bool blink = false;
    float confidence = 0.0f;
    double timestamp_sec = 0.0;
};

struct ProcessingStatus {
    bool image_healthy = true;
    bool tf_available = false;
    bool hdmap_available = false;
    bool v2x_available = false;
    std::string degrade_reason;
};

// --- 管道中间状态表达 ---

// 1. Attention ROI
struct PromptRegion {
    Rect2f roi_box;
    float weight = 0.0f;         // 权重：历史追踪 > 导航推断 > 全图兜底
    int source = 0;              // 0: Unknown, 1: History, 2: Nav, 3: Full-Image
    std::string signal_id;
};

// 2. 单帧视觉感知结果
struct VisualLight {
    Rect2f bbox;
    LightColor color = LightColor::UNKNOWN;
    LightShape shape = LightShape::UNKNOWN;
    float confidence = 0.0f;
    std::string signal_id;
    std::string camera_name;
    bool blink = false;
};

// 3. 拓扑与语义绑定结果
struct BoundLight {
    VisualLight visual_light;
    LaneIntent bound_intent = LaneIntent::UNKNOWN;  // 绑定到的车道意图
    float bind_score = 0.0f;
    double stopline_distance_m = -1.0;
};

// 4. 时序追踪实体
struct TrackedLight {
    int track_id = 0;
    BoundLight current_state;
    int lost_frames = 0;          // 抗闪烁与遮挡
};

// 5. 启发式推理辅助状态
struct HeuristicState {
    LightColor inferred_color = LightColor::UNKNOWN;
    float probability = 0.0f;
    std::string inference_reason; // 例如："前方车辆已起步"
};

// --- 最终输出结果 ---
struct TrafficLightResult {
    LightColor color = LightColor::UNKNOWN;
    LightShape shape = LightShape::UNKNOWN;
    float confidence = 0.0f;
    bool is_heuristic_override = false;   // 是否由启发式推理推翻了视觉结果
    bool blink = false;
    std::string signal_id;
    EvidenceSource source = EvidenceSource::UNKNOWN;
    std::string decision_reason;
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo
