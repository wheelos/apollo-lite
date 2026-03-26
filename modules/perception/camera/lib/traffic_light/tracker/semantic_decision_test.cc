#include "modules/perception/camera/lib/traffic_light/tracker/semantic_decision.h"

#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace camera {

class SemanticReviserTest : public ::testing::Test {
 protected:
  void SetUp() override {
    reviser_.reset(new SemanticReviser());
    // 如果有 Init 依赖，可以在这里 mock
  }

  std::unique_ptr<SemanticReviser> reviser_;

  // 辅助函数：创建一个只包含一个灯的 Frame
  CameraFrame MakeFrame(double timestamp, int semantic_id,
                        base::TLColor detect_color, float score) {
    CameraFrame frame;
    frame.timestamp = timestamp;
    base::TrafficLightPtr light(new base::TrafficLight);
    light->id = "test_light";
    light->semantic = semantic_id;
    light->region.is_detected = true;
    light->status.color = detect_color;
    light->region.detect_score = score;

    frame.traffic_lights.push_back(light);
    return frame;
  }
};

// 测试场景1：稳健性测试
// 输入序列：红 -> 红 -> 黑(漏检/遮挡) -> 红
// 期望输出：红 -> 红 -> 红 (补全) -> 红
TEST_F(SemanticReviserTest, TestRobustnessToNoise) {
  TrafficLightTrackerOptions options;
  int semantic_id = 101;

  // Frame 1: Red (High Conf)
  CameraFrame f1 = MakeFrame(1.0, semantic_id, base::TLColor::TL_RED, 0.9);
  reviser_->Track(options, &f1);
  EXPECT_EQ(f1.traffic_lights[0]->status.color, base::TLColor::TL_RED);

  // Frame 2: Red (High Conf)
  CameraFrame f2 = MakeFrame(1.1, semantic_id, base::TLColor::TL_RED, 0.9);
  reviser_->Track(options, &f2);
  EXPECT_EQ(f2.traffic_lights[0]->status.color, base::TLColor::TL_RED);

  // Frame 3: Black (Low Conf / Noise) - 模拟一次检测失误
  CameraFrame f3 = MakeFrame(1.2, semantic_id, base::TLColor::TL_BLACK, 0.2);
  reviser_->Track(options, &f3);

  // 关键点：虽然观测是 Black，但因为之前是 Red 且 Black 置信度低，
  // 贝叶斯滤波应该利用先验保持 Red，或者变成 Unknown，但不应直接跳变
  EXPECT_EQ(f3.traffic_lights[0]->status.color, base::TLColor::TL_RED);

  // Frame 4: Red (Recovery)
  CameraFrame f4 = MakeFrame(1.3, semantic_id, base::TLColor::TL_RED, 0.9);
  reviser_->Track(options, &f4);
  EXPECT_EQ(f4.traffic_lights[0]->status.color, base::TLColor::TL_RED);
}

// 测试场景2：状态切换
// 输入序列：绿 -> ... -> 绿 -> 黄 -> 红
// 期望：能够及时响应明确的变化
TEST_F(SemanticReviserTest, TestStateTransition) {
  TrafficLightTrackerOptions options;
  int semantic_id = 102;
  double t = 0.0;

  // 连续 5 帧绿色，建立高置信度
  for (int i = 0; i < 5; ++i) {
    t += 0.1;
    CameraFrame f = MakeFrame(t, semantic_id, base::TLColor::TL_GREEN, 0.9);
    reviser_->Track(options, &f);
    EXPECT_EQ(f.traffic_lights[0]->status.color, base::TLColor::TL_GREEN);
  }

  // 突然变为黄色 (观测置信度一般)
  t += 0.1;
  CameraFrame f_yellow =
      MakeFrame(t, semantic_id, base::TLColor::TL_YELLOW, 0.8);
  reviser_->Track(options, &f_yellow);

  // Green -> Yellow 是合法转换，且 Transition Matrix 中给了概率，
  // 应该能切换过去，或者在第一帧稍微犹豫一下。
  // 如果想要更灵敏，调整 Matrix 中的 G->Y 概率。
  // 这里我们期望至少不是 Red。
  base::TLColor res = f_yellow.traffic_lights[0]->status.color;
  bool is_yellow_or_green =
      (res == base::TLColor::TL_YELLOW || res == base::TLColor::TL_GREEN);
  EXPECT_TRUE(is_yellow_or_green);
}

// 测试场景3：非法跳变抑制
// 输入：红 -> 绿 (物理上几乎不可能瞬间发生，通常经过黄)
// 期望：由红变绿会有延迟，或者输出 Unknown，防止误判
TEST_F(SemanticReviserTest, TestIllegalTransition) {
  TrafficLightTrackerOptions options;
  int semantic_id = 103;

  // 稳定红色
  CameraFrame f1 = MakeFrame(10.0, semantic_id, base::TLColor::TL_RED, 0.95);
  reviser_->Track(options, &f1);
  reviser_->Track(options, &f1);  // 多跑几次稳定状态

  // 突然来一帧绿色 (可能是误检)
  CameraFrame f2 =
      MakeFrame(10.1, semantic_id, base::TLColor::TL_GREEN, 0.6);  // 置信度中等
  reviser_->Track(options, &f2);

  // 贝叶斯滤波器应当抑制这次跳变，因为 R->G 的转移概率极低 (设为0或者很小)
  // 结果应该依然是 Red，或者是 Unknown，绝对不能是 Green
  EXPECT_NE(f2.traffic_lights[0]->status.color, base::TLColor::TL_GREEN);
  EXPECT_EQ(f2.traffic_lights[0]->status.color, base::TLColor::TL_RED);
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
