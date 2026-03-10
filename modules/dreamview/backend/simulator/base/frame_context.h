#pragma once

#include <memory>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace dreamview {

// 障碍物状态快照
struct ObstacleStatus {
  int id;
  double x;
  double y;
  double theta;
  double v;
  double length;
  double width;
  bool is_static;
};

// Ego 车辆状态快照
struct EgoState {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double heading = 0.0;
  double v = 0.0;
  double a = 0.0;
  double kappa = 0.0;  // 曲率
  double throttle = 0.0;
  double brake = 0.0;
  double steering_percentage = 0.0;
};

// 仿真帧上下文：作为 Evaluator 的输入
struct FrameContext {
  // 时间信息
  double dt = 0.0;
  double sim_time_sec = 0.0;
  size_t frame_index = 0;

  // 状态数据（使用指针避免大对象拷贝，由 Engine 保证生命周期）
  const EgoState* ego_state = nullptr;
  const std::vector<ObstacleStatus>* obstacles = nullptr;

  // 扩展：地图信息（可选，用于检测是否压线、超速等）
  // MapService* map_service = nullptr;
};

}  // namespace dreamview
}  // namespace apollo
