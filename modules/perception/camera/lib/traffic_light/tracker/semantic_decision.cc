#include "modules/perception/camera/lib/traffic_light/tracker/semantic_decision.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

// 物理约束常量
constexpr double kMaxTimeInterval = 2.0;        // 超过2秒无观测，重置滤波器
constexpr double kFilterCleanupInterval = 5.0;  // 超过5秒未更新的ID，从内存清除
constexpr float kMinConfidence = 0.1f;          // 最低置信度钳位

// 辅助函数：将枚举转为整数索引
inline int ColorToInt(base::TLColor color) { return static_cast<int>(color); }

LightBayesFilter::LightBayesFilter() {
  // 初始化：Unknown 概率最大，其余均分
  // 顺序: Unknown, Red, Yellow, Green, Black
  probs_.assign(kNumColors, (1.0f - 0.6f) / (kNumColors - 1));
  probs_[ColorToInt(base::TLColor::TL_UNKNOWN_COLOR)] = 0.6f;
}

void LightBayesFilter::Predict() {
  // 状态转移矩阵 (Transition Matrix) P(X_t | X_t-1)
  // 行: t-1 时刻状态, 列: t 时刻状态
  // 这里的数值基于交通灯的物理规律设定
  static const float trans_mat[kNumColors][kNumColors] = {
      // From UNKNOWN: 倾向于保持 Unknown，或者变成任意颜色
      {0.90f, 0.025f, 0.025f, 0.025f, 0.025f},

      // From RED: 极大概率保持红，或者变绿(某些地区)，或者变黄(红黄亮)
      {0.01f, 0.95f, 0.01f, 0.03f, 0.00f},

      // From YELLOW: 黄灯持续时间短，倾向于变红
      {0.01f, 0.19f, 0.80f, 0.00f, 0.00f},

      // From GREEN: 绿灯倾向于变黄，极小概率直接变红
      {0.01f, 0.00f, 0.09f, 0.90f, 0.00f},

      // From BLACK: 黑灯可能随时亮起
      {0.10f, 0.225f, 0.225f, 0.225f, 0.225f}};

  std::vector<float> next_probs(kNumColors, 0.0f);
  for (int curr = 0; curr < kNumColors; ++curr) {
    for (int prev = 0; prev < kNumColors; ++prev) {
      next_probs[curr] += probs_[prev] * trans_mat[prev][curr];
    }
  }
  probs_ = next_probs;
}

void LightBayesFilter::Correct(base::TLColor obs_color, float obs_conf) {
  // 观测矩阵 (Likelihood) P(Z | X)
  // 逻辑：
  // 1. 如果 obs_conf 高，说明检测器很确信，似然分布应该很尖锐。
  // 2. 如果 obs_conf 低，说明检测器不确信，似然分布应该趋于均匀。

  // 钳位置信度，防止数值问题
  float conf = std::max(std::min(obs_conf, 0.99f), kMinConfidence);
  int obs_idx = ColorToInt(obs_color);
  bool valid_obs = (obs_color != base::TLColor::TL_UNKNOWN_COLOR);

  for (int state = 0; state < kNumColors; ++state) {
    float likelihood = 1.0f;

    if (!valid_obs) {
      // 观测无效(Unknown)，不提供区分信息
      likelihood = 1.0f;
    } else {
      if (state == obs_idx) {
        // 状态匹配观测
        likelihood = conf;
      } else {
        // 状态不匹配观测，将剩余概率均分
        likelihood = (1.0f - conf) / (kNumColors - 1);
      }
    }
    probs_[state] *= likelihood;
  }
}

void LightBayesFilter::Normalize() {
  float sum = std::accumulate(probs_.begin(), probs_.end(), 0.0f);
  if (sum < std::numeric_limits<float>::epsilon()) {
    // 异常兜底：重置为均匀分布
    std::fill(probs_.begin(), probs_.end(), 1.0f / kNumColors);
  } else {
    for (auto& p : probs_) p /= sum;
  }
}

void LightBayesFilter::Update(double timestamp, base::TLColor obs_color,
                              float obs_conf) {
  // 1. 时间连续性检查
  if (timestamp - last_timestamp_ > kMaxTimeInterval) {
    // 丢帧太久，重置先验
    probs_.assign(kNumColors, (1.0f - 0.6f) / (kNumColors - 1));
    probs_[ColorToInt(base::TLColor::TL_UNKNOWN_COLOR)] = 0.6f;
  }

  // 2. 预测
  Predict();

  // 3. 修正
  Correct(obs_color, obs_conf);

  // 4. 归一化
  Normalize();

  // 5. 闪烁检测
  base::TLColor max_color = GetMaxProbColor();
  CheckBlink(timestamp, max_color);

  last_timestamp_ = timestamp;
}

base::TLColor LightBayesFilter::GetMaxProbColor() const {
  auto it = std::max_element(probs_.begin(), probs_.end());
  int idx = std::distance(probs_.begin(), it);
  // 只有当最大概率超过一定阈值 (如 0.4) 才认为有效，否则返回 Unknown
  // 这可以防止在概率分布极度平坦时（如 0.2, 0.2, 0.2 ...）随意输出状态
  if (*it < 0.35f) {
    return base::TLColor::TL_UNKNOWN_COLOR;
  }
  return static_cast<base::TLColor>(idx);
}

void LightBayesFilter::CheckBlink(double timestamp, base::TLColor cur_color) {
  // 简化的闪烁检测：如果在 Green 和 Black/Unknown 之间快速切换
  if (cur_color == base::TLColor::TL_GREEN) {
    last_bright_ts_ = timestamp;
  } else if (cur_color == base::TLColor::TL_BLACK ||
             cur_color == base::TLColor::TL_UNKNOWN_COLOR) {
    last_dark_ts_ = timestamp;
  }

  if (cur_color == base::TLColor::TL_GREEN &&
      std::abs(timestamp - last_dark_ts_) < 0.5) {
    is_blinking_ = true;
  } else {
    // 滞后清除 Blink 状态，避免闪烁
    if (timestamp - last_bright_ts_ > 1.0) {
      is_blinking_ = false;
    }
  }
}

// ================= SemanticReviser Implementation =================

SemanticReviser::SemanticReviser() {}

bool SemanticReviser::Init(const TrafficLightTrackerInitOptions& options) {
  std::string proto_path =
      cyber::common::GetAbsolutePath(options.root_dir, options.conf_file);
  if (!cyber::common::GetProtoFromFile(proto_path, &semantic_param_)) {
    AERROR << "load proto param failed, root dir: " << options.root_dir;
    return false;
  }
  return true;
}

bool SemanticReviser::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }
  semantic_param_ = stage_config.semantic_reviser_config();
  return true;
}

std::pair<base::TLColor, float> SemanticReviser::AggregateObservations(
    const std::vector<base::TrafficLightPtr>& lights,
    const std::vector<int>& light_ids) {
  if (light_ids.empty()) return {base::TLColor::TL_UNKNOWN_COLOR, 0.0f};

  // 如果只有一个检测框，直接返回
  if (light_ids.size() == 1) {
    const auto& light = lights[light_ids[0]];
    if (!light->region.is_detected) {
      return {base::TLColor::TL_UNKNOWN_COLOR, 0.1f};
    }
    return {light->status.color, light->region.detect_score};
  }

  // 多个框对应同一个 ID (可能是一图多灯或者误检)
  // 策略：加权投票
  std::map<base::TLColor, float> color_scores;
  float total_weight = 0.0f;

  for (int id : light_ids) {
    auto& light = lights[id];
    float score = light->region.is_detected ? light->region.detect_score : 0.0f;
    // 忽略置信度过低的框
    if (score < 0.1f) continue;

    color_scores[light->status.color] += score;
    total_weight += score;
  }

  if (total_weight < kMinConfidence) {
    return {base::TLColor::TL_UNKNOWN_COLOR, 0.1f};
  }

  // 找出得分最高的颜色
  base::TLColor best_color = base::TLColor::TL_UNKNOWN_COLOR;
  float max_score = -1.0f;

  for (auto const& [color, score] : color_scores) {
    if (score > max_score) {
      max_score = score;
      best_color = color;
    }
  }

  // 计算归一化置信度：最大类得分 / 总得分
  // 这样如果红灯和绿灯得分差不多，置信度会降低
  float normalized_conf =
      (total_weight > 0) ? (max_score / total_weight) : 0.0f;

  return {best_color, normalized_conf};
}

bool SemanticReviser::Track(const TrafficLightTrackerOptions& options,
                            CameraFrame* frame) {
  if (!frame) return false;
  double timestamp = options.time_stamp;  // 使用 Options 里的时间戳更准确
  auto& lights = frame->traffic_lights;

  // 1. Grouping: 按 Semantic ID 分组
  std::unordered_map<std::string, std::vector<int>> semantic_groups;
  for (size_t i = 0; i < lights.size(); ++i) {
    // 优先使用 semantic_id，如果没有(如非地图模式)，回退到 lights[i]->id
    std::string key;
    if (lights[i]->semantic > 0) {
      key = "Semantic_" + std::to_string(lights[i]->semantic);
    } else {
      key = "ID_" + lights[i]->id;
    }
    semantic_groups[key].push_back(i);
  }

  // 2. Garbage Collection: 清理过期的滤波器
  for (auto it = filters_.begin(); it != filters_.end();) {
    if (timestamp - it->second.GetLastTimestamp() > kFilterCleanupInterval) {
      it = filters_.erase(it);
    } else {
      ++it;
    }
  }

  // 3. Update & Result Write-back
  for (auto& entry : semantic_groups) {
    std::string key = entry.first;
    const auto& indices = entry.second;

    // 聚合观测
    auto obs = AggregateObservations(lights, indices);
    base::TLColor obs_color = obs.first;
    float obs_conf = obs.second;

    // 获取滤波器 (Lazy Init)
    if (filters_.find(key) == filters_.end()) {
      filters_[key] = LightBayesFilter();
    }

    // 贝叶斯更新
    filters_[key].Update(timestamp, obs_color, obs_conf);

    // 获取滤波后结果
    base::TLColor final_color = filters_[key].GetMaxProbColor();
    bool is_blink = filters_[key].IsBlinking();
    // 使用后验概率作为最终置信度
    float final_conf = filters_[key].GetProbs()[ColorToInt(final_color)];

    // 将结果回填到 Frame 中所有的 TrafficLight 对象
    for (int idx : indices) {
      auto& light = lights[idx];
      light->status.color = final_color;
      light->status.confidence = final_conf;
      light->status.blink = is_blink;

      // 标记为已跟踪
      light->region.is_detected =
          (final_color != base::TLColor::TL_UNKNOWN_COLOR);
    }

    ADEBUG << "Tracker [" << key << "] Obs: " << ColorToString(obs_color) << "("
           << obs_conf << ") -> Final: " << ColorToString(final_color) << "("
           << final_conf << ")";
  }

  return true;
}

bool SemanticReviser::Process(DataFrame* data_frame) {
  if (data_frame == nullptr || data_frame->camera_frame == nullptr) {
    return false;
  }
  TrafficLightTrackerOptions options;
  options.time_stamp = data_frame->camera_frame->timestamp;
  return Track(options, data_frame->camera_frame);
}

REGISTER_TRAFFIC_LIGHT_TRACKER(SemanticReviser);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
