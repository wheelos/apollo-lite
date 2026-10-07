#pragma once

#include <string>
#include <vector>

#include "modules/common/math/vec2d.h"

namespace apollo {
namespace dreamview {

struct EgoConfig {
  std::string type = "PERFECT_CONTROL";  // 模型类型
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double v = 0.0;
  std::vector<apollo::common::math::Vec2d> waypoints;
};

struct ObstacleConfig {
  int id;
  std::string type;  // STATIC, DYNAMIC
  double x, y, heading, v;
  double length, width;
  double trigger_time = 0.0;
};

struct MetricConfig {
  std::string name;
  double value = 0.0;  // 例如最大仿真时间限制
};

// 完整的场景蓝图
struct Scenario {
  std::string name;
  EgoConfig ego_config;
  std::vector<ObstacleConfig> obstacles;
  // 可以继续扩展红绿灯、天气、路面系数等
  std::vector<MetricConfig> metrics;
};

}  // namespace dreamview
}  // namespace apollo
