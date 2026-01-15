#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/perception/pipeline/proto/stage/semantic.pb.h"

#include "modules/perception/camera/lib/interface/base_traffic_light_tracker.h"

namespace apollo {
namespace perception {
namespace camera {

// 定义状态维度：Unknown, Red, Yellow, Green, Black
constexpr int kNumColors = 5;

class LightBayesFilter {
 public:
  LightBayesFilter();

  // 核心：根据当前观测更新后验概率
  // timestamp: 当前时间戳
  // obs_color: 检测器给出的颜色
  // obs_conf: 检测器的置信度
  void Update(double timestamp, base::TLColor obs_color, float obs_conf);

  // 获取概率最大的颜色
  base::TLColor GetMaxProbColor() const;

  // 检查是否在闪烁（简单的后处理）
  bool IsBlinking() const { return is_blinking_; }

  // 获取当前状态向量（用于调试）
  const std::vector<float>& GetProbs() const { return probs_; }

 private:
  void Predict();
  void Correct(base::TLColor obs_color, float obs_conf);
  void Normalize();
  void CheckBlink(double timestamp, base::TLColor cur_max_color);

  // 状态向量 P(X)，顺序对应 base::TLColor 枚举值
  std::vector<float> probs_;

  double last_timestamp_ = 0.0;

  // 闪烁检测辅助变量
  bool is_blinking_ = false;
  double last_bright_ts_ = 0.0;
  double last_dark_ts_ = 0.0;
  base::TLColor last_color_ = base::TLColor::TL_UNKNOWN_COLOR;
};

class SemanticReviser : public BaseTrafficLightTracker {
 public:
  SemanticReviser();
  virtual ~SemanticReviser() = default;

  bool Init(const TrafficLightTrackerInitOptions& options) override;
  bool Init(const StageConfig& stage_config) override;

  bool Track(const TrafficLightTrackerOptions& options,
             CameraFrame* frame) override;

  std::string Name() const override { return "SemanticReviser"; }
  bool Process(DataFrame* data_frame) override;
  bool IsEnabled() const override { return enable_; }

 private:
  // 将检测器输出的多个 Box (针对同一个 ID) 合并为一个观测结果
  std::pair<base::TLColor, float> AggregateObservations(
      const std::vector<base::TrafficLightPtr>& lights,
      const std::vector<int>& light_ids);

  SemanticReviserConfig semantic_param_;

  // 维护每个 Semantic ID 对应的滤波器状态
  // Key: semantic_id (string), Value: Filter Object
  std::unordered_map<std::string, LightBayesFilter> filters_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
