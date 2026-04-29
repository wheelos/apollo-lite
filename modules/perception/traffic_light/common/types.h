#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace apollo {
namespace perception {
namespace traffic_light {

enum class LightColor {
  UNKNOWN = 0,
  RED = 1,
  YELLOW = 2,
  GREEN = 3,
  BLACK = 4,
};

enum class LightShape {
  UNKNOWN = 0,
  CIRCLE = 1,
  ARROW_LEFT = 2,
  ARROW_RIGHT = 3,
  ARROW_STRAIGHT = 4,
};

enum class LaneIntent {
  UNKNOWN = 0,
  STRAIGHT = 1,
  LEFT = 2,
  RIGHT = 3,
  U_TURN = 4,
};

enum class EvidenceSource {
  UNKNOWN = 0,
  VISION = 1,
  V2X = 2,
  HEURISTIC = 3,
};

enum class DetectorBackendType {
  HEURISTIC = 0,
  YOLO = 1,
};

enum class PromptSource {
  UNKNOWN = 0,
  HISTORY = 1,
  MAP = 2,
  NAVIGATION = 3,
  FULL_FRAME = 4,
};

constexpr uint32_t kMovementMaskNone = 0u;
constexpr uint32_t kMovementMaskStraight = 1u << 0;
constexpr uint32_t kMovementMaskLeft = 1u << 1;
constexpr uint32_t kMovementMaskRight = 1u << 2;
constexpr uint32_t kMovementMaskUTurn = 1u << 3;

struct Rect2f {
  float x = 0.0f;
  float y = 0.0f;
  float width = 0.0f;
  float height = 0.0f;

  bool IsValid() const { return width > 0.0f && height > 0.0f; }

  float Area() const {
    return std::max(0.0f, width) * std::max(0.0f, height);
  }
};

struct Image {
  const uint8_t* data = nullptr;
  int rows = 0;
  int cols = 0;
  int channels = 0;
  std::string encoding;
  std::shared_ptr<std::vector<uint8_t>> storage;
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

struct VehicleState {
  double timestamp_sec = 0.0;
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
  double distance_to_intersection_m = 0.0;
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
  std::string lane_id;
  std::string signal_group_id;
  std::string camera_name;
  Rect2f projection_roi;
  float confidence = 0.0f;
  float topology_confidence = 0.0f;
  LaneIntent intended_movement = LaneIntent::UNKNOWN;
  uint32_t movement_mask = kMovementMaskNone;
  LightShape shape_hint = LightShape::UNKNOWN;
  double stopline_distance_m = -1.0;
};

struct V2XLightEvidence {
  std::string signal_id;
  LaneIntent movement = LaneIntent::UNKNOWN;
  uint32_t movement_mask = kMovementMaskNone;
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
  bool detector_ran = false;
  bool using_full_frame_fallback = false;
  bool glare_detected = false;
  std::string degrade_reason;
};

struct PromptRegion {
  Rect2f roi_box;
  float weight = 0.0f;
  PromptSource source = PromptSource::UNKNOWN;
  std::string signal_id;
  std::string camera_name;
  LaneIntent intended_movement = LaneIntent::UNKNOWN;
  uint32_t movement_mask = kMovementMaskNone;
};

struct YoloLightCandidate {
  Rect2f bbox;
  float objectness = 0.0f;
  float semantic_confidence = 0.0f;
  int class_id = -1;
  std::string class_name;
  LightColor color = LightColor::UNKNOWN;
  LightShape shape = LightShape::UNKNOWN;
  uint32_t movement_mask = kMovementMaskNone;
  bool blink = false;
  std::string signal_id;
  std::string camera_name;
};

struct VisualLight {
  Rect2f bbox;
  LightColor color = LightColor::UNKNOWN;
  LightShape shape = LightShape::UNKNOWN;
  float confidence = 0.0f;
  float existence_confidence = 0.0f;
  float state_confidence = 0.0f;
  std::string signal_id;
  std::string camera_name;
  bool blink = false;
  bool glare_suspected = false;
  int yolo_class_id = -1;
  std::string yolo_class_name;
  PromptSource prompt_source = PromptSource::UNKNOWN;
  LaneIntent intended_movement = LaneIntent::UNKNOWN;
  uint32_t movement_mask = kMovementMaskNone;
  double stopline_distance_m = -1.0;
};

struct BoundLight {
  VisualLight visual_light;
  LaneIntent bound_intent = LaneIntent::UNKNOWN;
  uint32_t signal_movement_mask = kMovementMaskNone;
  float bind_score = 0.0f;
  float topology_confidence = 0.0f;
  float movement_match_score = 0.0f;
  bool controlling_ego_lane = false;
  std::string lane_id;
  std::string signal_group_id;
  double stopline_distance_m = -1.0;
};

struct TrackedLight {
  int track_id = 0;
  BoundLight current_state;
  LightColor stabilized_color = LightColor::UNKNOWN;
  float stabilized_confidence = 0.0f;
  bool blink = false;
  int age = 0;
  int visible_count = 0;
  int lost_frames = 0;
  bool confirmed = false;
  double last_visible_timestamp_sec = 0.0;
  std::array<float, 5> color_belief = {{1.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
};

struct HeuristicState {
  LightColor inferred_color = LightColor::UNKNOWN;
  float probability = 0.0f;
  int supporting_agents = 0;
  bool advisory_only = false;
  std::string inference_reason;
};

struct TrafficLightResult {
  LightColor color = LightColor::UNKNOWN;
  LightShape shape = LightShape::UNKNOWN;
  float confidence = 0.0f;
  float existence_confidence = 0.0f;
  float state_confidence = 0.0f;
  float topology_confidence = 0.0f;
  bool is_heuristic_override = false;
  bool blink = false;
  std::string signal_id;
  std::string lane_id;
  std::string signal_group_id;
  std::string camera_name;
  LaneIntent bound_intent = LaneIntent::UNKNOWN;
  uint32_t signal_movement_mask = kMovementMaskNone;
  bool controlling_ego_lane = false;
  double stopline_distance_m = -1.0;
  double freshness_sec = 0.0;
  bool is_degraded = false;
  EvidenceSource source = EvidenceSource::UNKNOWN;
  std::string degrade_reason;
  std::string decision_reason;
};

struct StageRuntimeMetrics {
  int input_count = 0;
  int output_count = 0;
  int rejected_count = 0;
  float max_confidence = 0.0f;
  float avg_confidence = 0.0f;
  bool fallback_used = false;
  std::string note;
};

struct PipelineRuntimeMetrics {
  StageRuntimeMetrics prompter;
  StageRuntimeMetrics detector;
  StageRuntimeMetrics binder;
  StageRuntimeMetrics tracker;
  StageRuntimeMetrics heuristic;
  StageRuntimeMetrics fusion;
};

struct CameraSourceOptions {
  std::string camera_name;
  std::string channel_name;
  int image_width = 1920;
  int image_height = 1080;
  bool is_primary = false;
};

struct PrompterOptions {
  float history_expand_ratio = 1.4f;
  float history_weight = 0.9f;
  float memory_expand_ratio = 1.6f;
  float memory_weight = 0.75f;
  float map_expand_ratio = 1.25f;
  float map_weight_floor = 0.7f;
  double cold_start_distance_to_intersection_m = 120.0;
  float full_frame_weight = 0.1f;
  bool always_add_full_frame_fallback = true;
};

struct DetectorOptions {
  DetectorBackendType backend = DetectorBackendType::YOLO;
  float min_prompt_weight = 0.12f;
  float min_bright_pixel_ratio = 0.01f;
  float min_color_ratio = 0.45f;
  float min_roi_area = 36.0f;
  int sample_grid_divisor = 16;
  float map_overlap_bonus = 0.2f;
  float history_bonus = 0.15f;
  bool allow_unknown_output = false;
  float glare_white_ratio_threshold = 0.35f;
  float glare_penalty = 0.35f;
  float min_signal_overlap_for_glare_override = 0.45f;
  float min_objectness = 0.25f;
  float min_semantic_confidence = 0.25f;
  bool prefer_raw_yolo_candidates = true;
};

struct BinderOptions {
  float min_bind_score = 0.15f;
  bool require_intent_match = true;
  float min_signal_overlap = 0.2f;
  float max_center_distance_ratio = 1.2f;
  float min_topology_confidence = 0.35f;
};

struct TrackerOptions {
  int max_lost_frames = 12;
  float min_iou_match = 0.2f;
  float belief_decay = 0.85f;
  float measurement_gain = 0.65f;
  int min_confirmed_visible_count = 2;
};

struct HeuristicOptions {
  double max_distance_to_intersection_m = 100.0;
  int min_starting_agents = 2;
  float green_probability = 0.85f;
  float red_hold_probability = 0.65f;
};

struct FusionOptions {
  float weak_vision_threshold = 0.6f;
  float strong_v2x_threshold = 0.7f;
  float heuristic_trigger_threshold = 0.4f;
  float heuristic_accept_threshold = 0.92f;
  double v2x_sync_window_sec = 0.3;
  bool prefer_ego_intent = true;
  float min_conflict_override_margin = 0.25f;
};

struct ComponentOptions {
  std::vector<CameraSourceOptions> cameras;
  std::string tf2_frame_id = "world";
  std::string tf2_child_frame_id = "novatel";
  double tf2_timeout_second = 0.01;
  double max_process_image_fps = 8.0;
  double query_tf_interval_seconds = 0.3;
  double valid_hdmap_interval_seconds = 1.5;
  double image_sys_ts_diff_threshold = 0.5;
  double frame_cache_tolerance_sec = 0.2;
  std::string output_channel_name = "/apollo/perception/traffic_light";
  std::string debug_output_channel_name = "/apollo/perception/traffic_light/debug";
  std::string debug_image_channel_name =
      "/apollo/perception/traffic_light/debug/image";
  std::string v2x_channel_name = "/apollo/v2x/traffic_light";
  double v2x_sync_interval_seconds = 0.3;
  int max_v2x_msg_buff_size = 64;
  bool enable_debug_recording = true;
  bool enable_debug_image_stream = true;
  bool enable_perf_logging = true;
  bool enable_heuristic_stage = true;
  PrompterOptions prompter;
  DetectorOptions detector;
  BinderOptions binder;
  TrackerOptions tracker;
  HeuristicOptions heuristic;
  FusionOptions fusion;
};

inline uint32_t MovementMaskFromLaneIntent(LaneIntent intent) {
  switch (intent) {
    case LaneIntent::STRAIGHT:
      return kMovementMaskStraight;
    case LaneIntent::LEFT:
      return kMovementMaskLeft;
    case LaneIntent::RIGHT:
      return kMovementMaskRight;
    case LaneIntent::U_TURN:
      return kMovementMaskUTurn;
    default:
      return kMovementMaskNone;
  }
}

inline uint32_t MovementMaskFromShape(LightShape shape) {
  switch (shape) {
    case LightShape::ARROW_LEFT:
      return kMovementMaskLeft;
    case LightShape::ARROW_RIGHT:
      return kMovementMaskRight;
    case LightShape::ARROW_STRAIGHT:
      return kMovementMaskStraight;
    case LightShape::CIRCLE:
      return kMovementMaskStraight | kMovementMaskLeft | kMovementMaskRight |
             kMovementMaskUTurn;
    default:
      return kMovementMaskNone;
  }
}

inline uint32_t NormalizeMovementMask(uint32_t movement_mask,
                                      LaneIntent intended_movement,
                                      LightShape shape) {
  if (movement_mask != kMovementMaskNone) {
    return movement_mask;
  }
  const uint32_t lane_mask = MovementMaskFromLaneIntent(intended_movement);
  if (lane_mask != kMovementMaskNone) {
    return lane_mask;
  }
  return MovementMaskFromShape(shape);
}

inline bool MovementMasksCompatible(uint32_t lhs, uint32_t rhs) {
  if (lhs == kMovementMaskNone || rhs == kMovementMaskNone) {
    return true;
  }
  return (lhs & rhs) != 0u;
}

inline LaneIntent PrimaryLaneIntentFromMask(uint32_t movement_mask) {
  if ((movement_mask & kMovementMaskStraight) != 0u) {
    return LaneIntent::STRAIGHT;
  }
  if ((movement_mask & kMovementMaskLeft) != 0u) {
    return LaneIntent::LEFT;
  }
  if ((movement_mask & kMovementMaskRight) != 0u) {
    return LaneIntent::RIGHT;
  }
  if ((movement_mask & kMovementMaskUTurn) != 0u) {
    return LaneIntent::U_TURN;
  }
  return LaneIntent::UNKNOWN;
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
