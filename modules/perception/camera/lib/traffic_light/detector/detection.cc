#include "modules/perception/camera/lib/traffic_light/detector/detection.h"

#include <algorithm>
#include <utility>
#include <opencv2/opencv.hpp>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

TrafficLightDetection::TrafficLightDetection() : device_(torch::kCPU) {
  class_id_map_[static_cast<base::TLDetectionClass>(0)] = base::TLColor::TL_BLACK;
  class_id_map_[static_cast<base::TLDetectionClass>(1)] = base::TLColor::TL_RED;
  class_id_map_[static_cast<base::TLDetectionClass>(2)] = base::TLColor::TL_YELLOW;
  class_id_map_[static_cast<base::TLDetectionClass>(3)] = base::TLColor::TL_GREEN;
}

TrafficLightDetection::~TrafficLightDetection() {
    torch_model_ = torch::jit::Module(); // 显式释放模型，不要等全局析构
}

bool TrafficLightDetection::Init(
    const TrafficLightDetectorInitOptions &options) {
  std::string proto_path = GetAbsolutePath(options.root_dir, options.conf_file);
  if (!cyber::common::GetProtoFromFile(proto_path, &detection_param_)) {
    return false;
  }
  return InitInternal(options.root_dir, options.gpu_id);
}

bool TrafficLightDetection::Init(const StageConfig &stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }
  detection_param_ = stage_config.traffic_light_detection_config();
  return InitInternal(detection_param_.traffic_light_detection_root_dir(),
                      detection_param_.gpu_id());
}

bool TrafficLightDetection::InitInternal(const std::string &root_dir,
                                         int gpu_id) {
  detection_root_dir_ = root_dir;
  gpu_id_ = gpu_id;

  // 1. 设置 Device
  torch::set_num_threads(1);  // 避免与 Cyber 线程池冲突
  if (gpu_id_ >= 0 && torch::cuda::is_available()) {
    device_ = torch::Device(torch::kCUDA, gpu_id_);
    AINFO << "LibTorch using CUDA device: " << gpu_id_;
  } else {
    device_ = torch::Device(torch::kCPU);
    AWARN << "LibTorch using CPU.";
  }

  // 2. 加载 TorchScript 模型 (.pt / .pth)
  std::string model_root =
      GetAbsolutePath(root_dir, detection_param_.model_name());
  // 注意：在 Proto 配置中，weight_file 应该指向 .pt 文件
  std::string model_file =
      GetAbsolutePath(model_root, detection_param_.weight_file());

  try {
    torch_model_ = torch::jit::load(model_file, device_);
    torch_model_.eval();  // 设置为评估模式
    AINFO << "Successfully loaded TorchScript model: " << model_file;
  } catch (const c10::Error &e) {
    AERROR << "Error loading model: " << e.msg();
    return false;
  }

  // 3. 辅助初始化
  int crop_size = detection_param_.min_crop_size();

  if (detection_param_.crop_method() == 1) {
    crop_.reset(new CropBoxWholeImage());
  } else {
    crop_.reset(new CropBox(detection_param_.crop_scale(), crop_size));
  }

  select_.Init(crop_size, crop_size);
  image_.reset(new base::Image8U(crop_size, crop_size, base::Color::BGR));

  // 4. Warmup - 跑一次空数据防止第一次推理延迟过高
  try {
    torch::Tensor dummy_input =
        torch::zeros({1, 3, crop_size, crop_size}).to(device_);
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(dummy_input);
    torch_model_.forward(inputs);
    AINFO << "Model Warmup done.";
  } catch (...) {
    AWARN << "Model Warmup failed, but continuing.";
  }

  return true;
}

bool TrafficLightDetection::Process(DataFrame *data_frame) {
  TrafficLightDetectorOptions options;
  return Detect(options, data_frame->camera_frame);
}

bool TrafficLightDetection::Detect(const TrafficLightDetectorOptions &options,
                                   CameraFrame *frame) {
  if (frame->traffic_lights.empty()) return true;

  auto &lights = frame->traffic_lights;

  // Reset logic
  for (auto &light : lights) {
    light->region.is_detected = false;
    light->status.color = base::TLColor::TL_UNKNOWN_COLOR;
    light->status.confidence = 0.0f;
    light->region.debug_roi.clear();
  }

  if (!Inference(&lights, frame->data_provider)) {
    AERROR << "Inference failed";
    return false;
  }

  // One-Stage 结果回填
  for (auto &light : lights) {
    if (light->region.is_detected) {
      light->status.color = ClassIdToColor(light->region.detect_class_id);
      light->status.confidence = light->region.detect_score;
      AINFO << "TL ID: " << light->id << " Color: " << static_cast<int>(light->status.color);
    }
  }

  return true;
}

bool TrafficLightDetection::Inference(
    std::vector<base::TrafficLightPtr> *lights, DataProvider *data_provider) {

  detected_bboxes_.clear();
  crop_box_list_.clear();
  std::vector<LetterboxInfo> letterbox_infos;

  // 建议从配置读取，YOLOv12n 默认通常是 640
  int target_size = detection_param_.min_crop_size();
  if (target_size == 0) target_size = 640;

  std::vector<torch::Tensor> batch_tensors;
  batch_tensors.reserve(lights->size());

  for (auto &light : *lights) {
    base::RectI cbox;
    crop_->getCropBox(data_provider->src_width(),
                      data_provider->src_height(), light, &cbox);

    if (cbox.width <= 0 || cbox.height <= 0) continue;

    data_provider_image_option_.crop_roi = cbox;
    data_provider_image_option_.do_crop = true;
    data_provider_image_option_.target_color = base::Color::BGR;

    // 1. 获取原始尺寸的 Crop 图像
    // 注意：这里我们临时需要一个能容纳原始crop的buffer，或者直接用opencv处理
    // 为了利用 DataProvider，我们先获取到 image_ (需调整 image_ 大小策略或每次new)
    // 更好的方式是直接操作 Blob，但这里为了兼容 Apollo 接口：
    if (image_->rows() != cbox.height || image_->cols() != cbox.width) {
        image_.reset(new base::Image8U(cbox.height, cbox.width, base::Color::BGR));
    }

    if (!data_provider->GetImage(data_provider_image_option_, image_.get())) {
      continue;
    }

    // 2. Letterbox 预处理
    cv::Mat src_img(image_->rows(), image_->cols(), CV_8UC3, image_->mutable_cpu_data());
    cv::Mat dst_img;

    // 计算缩放比例 (保持长宽比)
    float scale = std::min((float)target_size / src_img.cols, (float)target_size / src_img.rows);
    int new_w = std::round(src_img.cols * scale);
    int new_h = std::round(src_img.rows * scale);

    cv::Mat resized;
    cv::resize(src_img, resized, cv::Size(new_w, new_h));

    // 创建画布并填充灰色 (114)
    dst_img = cv::Mat::ones(target_size, target_size, CV_8UC3) * 114;

    // 居中放置
    int pad_w = (target_size - new_w) / 2;
    int pad_h = (target_size - new_h) / 2;
    resized.copyTo(dst_img(cv::Rect(pad_w, pad_h, new_w, new_h)));

    // 保存信息供后处理使用
    crop_box_list_.push_back(cbox);
    letterbox_infos.push_back({scale, pad_w, pad_h});

    // 3. 转 Tensor (BGR -> RGB check required based on training)
    // 假设模型是 PyTorch 默认训练的，通常需要 RGB
    cv::cvtColor(dst_img, dst_img, cv::COLOR_BGR2RGB);

    torch::Tensor tensor_img = torch::from_blob(
        dst_img.data, {target_size, target_size, 3}, torch::kUInt8);

    // [H, W, C] -> [C, H, W] -> Normalize
    tensor_img = tensor_img.permute({2, 0, 1}).to(torch::kFloat).div(255.0);
    batch_tensors.push_back(tensor_img);
  }

  if (batch_tensors.empty()) return true;

  // 4. 推理
  torch::NoGradGuard no_grad;
  try {
    torch::Tensor input_batch = torch::stack(batch_tensors).to(device_);

    // 鲁棒的 Forward 调用
    // 针对 YOLO12n，输入通常直接是 Tensor
    auto output_ivalue = torch_model_.forward({input_batch});

    torch::Tensor output;

    // 修复 Crash 的关键逻辑
    if (output_ivalue.isTuple()) {
        output = output_ivalue.toTuple()->elements()[0].toTensor();
    } else if (output_ivalue.isTensor()) {
        output = output_ivalue.toTensor();
    } else {
        AERROR << "Unexpected model output type: " << output_ivalue.tagKind();
        return false;
    }

    // 5. 后处理 (传入 Letterbox 信息)
    ProcessYOLOOutput(output.cpu(), crop_box_list_, letterbox_infos, &detected_bboxes_);

  } catch (const c10::Error &e) {
    AERROR << "Inference Error: " << e.msg();
    return false;
  }

  ApplyNMS(&detected_bboxes_);
  select_.SelectTrafficLights(detected_bboxes_, lights);

  return true;
}

bool TrafficLightDetection::ProcessYOLOOutput(
    const torch::Tensor &output_tensor,
    const std::vector<base::RectI> &crop_box_list,
    const std::vector<LetterboxInfo> &letterbox_infos,
    std::vector<base::TrafficLightPtr> *detected_lights) {

  // output_tensor shape: [Batch, Channels, Anchors] -> [1, 84, 8400]
  if (output_tensor.dim() != 3) return false;

  int batch_size = output_tensor.size(0);
  int num_channels = output_tensor.size(1);
  int num_anchors = output_tensor.size(2);

  // 检查是否使用了错误的 COCO 模型
  if (num_channels > 10) {
      AWARN_EVERY(100) << "Model has " << num_channels << " channels. "
                       << "Ensure you are using a custom trained Traffic Light model, not standard COCO!";
  }

  float conf_threshold = 0.25f;
  auto acc = output_tensor.accessor<float, 3>();

  for (int b = 0; b < batch_size; ++b) {
    if (static_cast<size_t>(b) >= crop_box_list.size()) {
        break;
    }

    const auto& info = letterbox_infos[b];

    for (int i = 0; i < num_anchors; ++i) {
      float max_score = 0.0f;
      int class_id = -1;

      // 寻找置信度最高的类别
      for (int c = 4; c < num_channels; ++c) {
        if (acc[b][c][i] > max_score) {
          max_score = acc[b][c][i];
          class_id = c - 4;
        }
      }

      if (max_score < conf_threshold) continue;

      // 提取坐标 (YOLO: cx, cy, w, h) based on 640x640 input
      float cx = acc[b][0][i];
      float cy = acc[b][1][i];
      float w = acc[b][2][i];
      float h = acc[b][3][i];

      // 1. 还原 Letterbox (去除 Padding)
      float x_no_pad = (cx - w/2.0f - info.pad_w);
      float y_no_pad = (cy - h/2.0f - info.pad_h);

      // 2. 还原 Scale
      float x_original = x_no_pad / info.scale;
      float y_original = y_no_pad / info.scale;
      float w_original = w / info.scale;
      float h_original = h / info.scale;

      // 3. 加上 Crop 偏移
      base::RectI rect;
      rect.x = static_cast<int>(x_original) + crop_box_list[b].x;
      rect.y = static_cast<int>(y_original) + crop_box_list[b].y;
      rect.width = static_cast<int>(w_original);
      rect.height = static_cast<int>(h_original);

      base::TrafficLightPtr light(new base::TrafficLight);
      light->region.detection_roi = rect;
      light->region.detect_score = max_score;
      light->region.detect_class_id = static_cast<base::TLDetectionClass>(class_id);
      light->region.is_detected = true;

      detected_lights->push_back(light);
    }
  }
  return true;
}

void TrafficLightDetection::ApplyNMS(std::vector<base::TrafficLightPtr> *lights,
                                     double iou_thresh) {
  if (!lights || lights->empty()) return;
  std::sort(lights->begin(), lights->end(),
            [](const base::TrafficLightPtr &a, const base::TrafficLightPtr &b) {
              return a->region.detect_score > b->region.detect_score;
            });
  std::vector<base::TrafficLightPtr> result;
  std::vector<bool> suppressed(lights->size(), false);
  for (size_t i = 0; i < lights->size(); ++i) {
    if (suppressed[i]) continue;
    result.push_back((*lights)[i]);
    for (size_t j = i + 1; j < lights->size(); ++j) {
      if (suppressed[j]) continue;
      float inter = ((*lights)[i]->region.detection_roi &
                     (*lights)[j]->region.detection_roi)
                        .Area();
      float un = ((*lights)[i]->region.detection_roi |
                  (*lights)[j]->region.detection_roi)
                     .Area();
      if (inter / un > iou_thresh) suppressed[j] = true;
    }
  }
  *lights = result;
}

base::TLColor TrafficLightDetection::ClassIdToColor(
    base::TLDetectionClass class_id) const {
  auto it = class_id_map_.find(class_id);
  if (it != class_id_map_.end()) {
    return it->second;
  }
  return base::TLColor::TL_UNKNOWN_COLOR;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
