#include "modules/perception/camera/lib/traffic_light/detector/detection/detection.h"

#include <cstdlib>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "modules/perception/pipeline/proto/pipeline_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/data_provider.h"

namespace apollo {
namespace perception {
namespace camera {
namespace {

using cyber::common::GetAbsolutePath;

// 辅助函数：获取文件名
std::string BaseName(const std::string& path) {
  return path.substr(path.find_last_of('/') + 1);
}

// 辅助函数：获取目录名
std::string DirName(const std::string& path) {
  std::string::size_type pos = path.find_last_of('/');
  return (pos == std::string::npos) ? "." : path.substr(0, pos);
}

// 辅助函数：颜色转字符串
std::string ColorToString(base::TLColor color) {
  switch (color) {
    case base::TLColor::TL_RED:
      return "RED";
    case base::TLColor::TL_YELLOW:
      return "YELLOW";
    case base::TLColor::TL_GREEN:
      return "GREEN";
    case base::TLColor::TL_BLACK:
      return "BLACK";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

class TrafficLightDetectionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 设置默认环境变量，方便测试
    // 实际使用时替换为真实的 TorchScript 模型路径
    model_path_ =
        "/apollo/modules/perception/production/data/perception/camera/models/"
        "traffic_light_detection/yolo_traffic.pt";
    image_path_ =
        "/apollo/modules/perception/camera/lib/traffic_light/detector/"
        "test_data/test.jpg";

    // 如果找不到图片，构造一个假的
    if (!cyber::common::PathExists(image_path_)) {
      AWARN << "Test image not found at " << image_path_
            << ", generating dummy image.";
      image_ = cv::Mat(640, 640, CV_8UC3, cv::Scalar(100, 100, 100));
      // 画一个红灯模拟
      cv::circle(image_, cv::Point(320, 320), 20, cv::Scalar(0, 0, 255), -1);
    } else {
      image_ = cv::imread(image_path_, cv::IMREAD_COLOR);
    }
  }

  std::string model_path_;
  std::string image_path_;
  cv::Mat image_;
};

TEST_F(TrafficLightDetectionTest, InferenceExecution) {
  // 1. 检查模型文件是否存在 (LibTorch 加载必须要有文件)
  // 如果没有真实模型，这里会 Skip，但代码逻辑是通的
  if (!cyber::common::PathExists(model_path_)) {
    AWARN << "Model file not found: " << model_path_
          << ". Skipping inference test.";
    // 为了验证代码逻辑，我们甚至可以 Mock 一个空的 Detection 类，
    // 但这里我们主要测试流程。如果没有模型，直接返回。
    GTEST_SKIP();
  }

  // 2. 构造配置 (Stage Config)
  pipeline::StageConfig stage_config;
  stage_config.set_stage_type(pipeline::StageType::TRAFFIC_LIGHT_DETECTION);
  stage_config.set_enabled(true);

  auto* detection_config =
      stage_config.mutable_traffic_light_detection_config();

  // Crop 配置
  detection_config->set_min_crop_size(640);
  detection_config->set_crop_method(1);  // 1 = Whole Image Crop (YOLO style)
  detection_config->set_crop_scale(1.0f);

  // 模型路径配置
  // 注意：InitInternal 逻辑是 Root + ModelName + WeightFile
  detection_config->set_traffic_light_detection_root_dir(DirName(model_path_));
  detection_config->set_model_name(".");  // 当前目录
  detection_config->set_weight_file(
      BaseName(model_path_));            // .pt 文件名放在 weight_file
  detection_config->set_proto_file("");  // LibTorch 不需要 proto

  // 硬件配置
  detection_config->set_gpu_id(0);  // 尝试使用 GPU 0

  // 3. 初始化检测器
  TrafficLightDetection detector;
  bool init_status = detector.Init(stage_config);
  ASSERT_TRUE(init_status)
      << "Detector Init failed. Check model path and format.";

  // 4. 初始化数据提供者 (DataProvider)
  DataProvider data_provider;
  DataProvider::InitOptions dp_init_options;
  dp_init_options.image_height = image_.rows;
  dp_init_options.image_width = image_.cols;
  dp_init_options.do_undistortion = false;
  dp_init_options.sensor_name = "test_sensor";
  dp_init_options.device_id = 0;
  ASSERT_TRUE(data_provider.Init(dp_init_options));

  // 填充图像数据 (Simulate Camera Driver)
  ASSERT_TRUE(data_provider.FillImageData(image_.rows, image_.cols, image_.data,
                                          "bgr8"));

  // 5. 构造 Frame 和 Dummy Light
  // 检测器需要 frame->traffic_lights 非空才会进入推理循环
  CameraFrame frame;
  frame.data_provider = &data_provider;

  base::TrafficLightPtr light(new base::TrafficLight);
  light->id = "test_light_01";
  // 假设这是一个全图检测，Projection ROI 设为全图或任意有效值
  light->region.projection_roi = base::RectI(0, 0, image_.cols, image_.rows);
  frame.traffic_lights.push_back(light);

  // 6. 执行推理
  TrafficLightDetectorOptions options;
  bool detect_status = detector.Detect(options, &frame);
  ASSERT_TRUE(detect_status) << "Inference execution failed.";

  // 7. 输出结果
  AINFO << "=== Inference Result ===";
  bool found_detection = false;
  for (const auto& l : frame.traffic_lights) {
    // 只有 is_detected 为 true 才表示检测到了
    if (l->region.is_detected) {
      found_detection = true;
      const auto& roi = l->region.detection_roi;
      AINFO << "Light ID: " << l->id;
      AINFO << "  -> Color: " << ColorToString(l->status.color);
      AINFO << "  -> Score: " << l->status.confidence;
      AINFO << "  -> Box: [" << roi.x << ", " << roi.y << ", " << roi.width
            << ", " << roi.height << "]";
    }
  }

  if (!found_detection) {
    AINFO << "No traffic lights detected (Is the model trained? Is the image "
             "correct?)";
  } else {
    AINFO << "Successfully detected traffic lights.";
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
