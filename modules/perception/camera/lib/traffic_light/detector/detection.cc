#include "modules/perception/camera/lib/traffic_light/detector/detection.h"

#include <algorithm>
#include <utility>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

TrafficLightDetection::TrafficLightDetection() : device_(torch::kCPU) {
  // 1. 设置颜色映射 (根据模型训练时的 Label ID 修改)
  // 假设: 0=Black, 1=Red, 2=Yellow, 3=Green
  class_id_map_[0] = base::TLColor::TL_BLACK;
  class_id_map_[1] = base::TLColor::TL_RED;
  class_id_map_[2] = base::TLColor::TL_YELLOW;
  class_id_map_[3] = base::TLColor::TL_GREEN;
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
  if (!Initialize(stage_config)) return false;
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

  // 4. 预热 (Warmup) - 跑一次空数据防止第一次推理延迟过高
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
      AINFO << "TL ID: " << light->id << " Color: " << light->status.color;
    }
  }

  return true;
}

bool TrafficLightDetection::Inference(
    std::vector<base::TrafficLightPtr> *lights, DataProvider *data_provider) {
  // 清理上一帧数据
  detected_bboxes_.clear();
  crop_box_list_.clear();
  resize_scale_list_.clear();

  int crop_size = detection_param_.min_crop_size();

  // 1. 构建 Batch Input
  std::vector<torch::Tensor> batch_tensors;
  // 预分配内存，避免 vector 扩容开销
  batch_tensors.reserve(lights->size());

  for (auto &light : *lights) {
    base::RectI cbox;
    // 获取基于地图投影的 Crop 区域，如果失败则跳过
    if (!crop_->getCropBox(data_provider->src_width(),
                           data_provider->src_height(), light, &cbox)) {
      continue;
    }

    // 边界安全检查
    if (cbox.width <= 0 || cbox.height <= 0) continue;

    data_provider_image_option_.crop_roi = cbox;
    data_provider_image_option_.do_crop = true;
    data_provider_image_option_.target_color = base::Color::BGR;

    // 从 DataProvider 获取图像
    if (!data_provider->GetImage(data_provider_image_option_, image_.get())) {
      AERROR << "Failed to get image from data provider.";
      continue;
    }

    crop_box_list_.push_back(cbox);
    // 计算缩放比例，后续用于坐标还原
    float scale = static_cast<float>(crop_size) /
                  static_cast<float>(std::min(cbox.width, cbox.height));
    resize_scale_list_.push_back(scale);

    // Zero-Copy 创建 Tensor (H, W, C)
    torch::Tensor tensor_img = torch::from_blob(
        image_->mutable_cpu_data(),
        {image_->rows(), image_->cols(), image_->channels()}, torch::kUInt8);

    // 预处理 pipeline: Permute(CHW) -> Float -> Normalize(0-1)
    // 注意：.to(device_) 会发生显存拷贝，这是必要的
    tensor_img = tensor_img.permute({2, 0, 1}).to(torch::kFloat).div(255.0);
    batch_tensors.push_back(tensor_img);
  }

  if (batch_tensors.empty()) return true;

  // 2. 推理执行 (Inference Execution)
  torch::NoGradGuard no_grad;  // 关键：关闭梯度计算

  try {
    // Stack tensors: [Batch, C, H, W]
    torch::Tensor input_batch = torch::stack(batch_tensors).to(device_);

    std::vector<torch::jit::IValue> inputs;
    inputs.emplace_back(input_batch);

    // Forward
    auto output_tuple = torch_model_.forward(inputs);

    // 处理输出：根据模型导出的格式，可能是 Tuple 或 Tensor
    // 假设标准格式: [Batch, N, 7] (batch_idx, class, score, x, y, x, y)
    // 如果模型输出是 Tuple (prediction, ...)，这里需要相应调整
    torch::Tensor output;
    if (output_tuple.isTuple()) {
      output = output_tuple.toTuple()->elements()[0].toTensor();
    } else {
      output = output_tuple.toTensor();
    }

    // 将结果移回 CPU
    SelectOutputBoxes(output.cpu(), crop_box_list_, resize_scale_list_,
                      resize_scale_list_, &detected_bboxes_);

  } catch (const c10::Error &e) {
    AERROR << "LibTorch Inference Error: " << e.msg();
    return false;  // 保护进程不崩溃
  }

  // 3. 后处理：NMS 与 匹配 (NMS & Matching)
  ApplyNMS(&detected_bboxes_);

  // Select 模块负责将视觉检测结果 (detected_bboxes_)
  // 匹配回输入的地图投影结果 (lights)
  // 匹配成功会将 detect_class_id, detect_score 等信息填入 light->region
  select_.SelectTrafficLights(detected_bboxes_, lights);

  return true;
}

bool TrafficLightDetection::SelectOutputBoxes(
    const torch::Tensor &output_tensor,
    const std::vector<base::RectI> &crop_box_list,
    const std::vector<float> &resize_scale_list_col,
    const std::vector<float> &resize_scale_list_row,
    std::vector<base::TrafficLightPtr> *detected_lights) {
  // 假设 Tensor 形状: [Batch, NumBoxes, 7]
  // 格式: [batch_index, class_id, score, x1, y1, x2, y2]
  // 注意：很多 PyTorch 模型 (如 YOLO) 输出是 [Batch, NumBoxes, 5+Class]

  if (output_tensor.dim() < 2) return false;

  // 获取 Tensor 的数据指针 (float)
  auto output_accessor = output_tensor.accessor<float, 3>();  // 假设 [B, N, 7]
  // 如果是 [N, 7] (即 Batch 维度被 flatten 了)，使用 accessor<float, 2>

  int batch_size = output_tensor.size(0);
  int num_boxes = output_tensor.size(1);
  int feature_dim = output_tensor.size(2);  // 应该 >= 6 或 7

  for (int b = 0; b < batch_size; ++b) {
    for (int i = 0; i < num_boxes; ++i) {
      // 根据具体的模型输出调整索引
      // 常见格式 A: [x1, y1, x2, y2, conf, cls_conf] -> 需配合 batch index
      // 常见格式 B: [batch_idx, class, score, x1, y1, x2, y2] (类似 Caffe
      // DetectionOutput)

      // 下面以格式 B (Flattened 或 Batch Separated) 为例进行通用适配
      // 假设我们现在的 output_tensor 是 [Batch, N, 6] -> [x1, y1, x2, y2,
      // conf, cls]

      float score = output_accessor[b][i][4];
      if (score < detection_param_.score_threshold()) continue;

      float x1 = output_accessor[b][i][0];
      float y1 = output_accessor[b][i][1];
      float x2 = output_accessor[b][i][2];
      float y2 = output_accessor[b][i][3];
      int class_id = static_cast<int>(output_accessor[b][i][5]);

      // 如果模型输出是归一化的 (0-1)，需要乘以 crop_size
      // x1 *= detection_param_.min_crop_size(); ...

      // 坐标映射回原图
      int img_id = b;  // 因为我们做了 batch stack
      if (img_id >= crop_box_list.size()) continue;

      float scale_w = 1.0f / resize_scale_list_col[img_id];
      float scale_h = 1.0f / resize_scale_list_row[img_id];

      base::RectI rect;
      rect.x = static_cast<int>(x1 * scale_w) + crop_box_list[img_id].x;
      rect.y = static_cast<int>(y1 * scale_h) + crop_box_list[img_id].y;
      rect.width = static_cast<int>((x2 - x1) * scale_w);
      rect.height = static_cast<int>((y2 - y1) * scale_h);

      base::TrafficLightPtr light(new base::TrafficLight);
      light->region.detection_roi = rect;
      light->region.detect_score = score;
      light->region.detect_class_id = class_id;
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

base::TLColor TrafficLightDetection::ClassIdToColor(int class_id) const {
  if (class_id_map_.find(class_id) != class_id_map_.end()) {
    return class_id_map_.at(class_id);
  }
  return base::TLColor::TL_UNKNOWN_COLOR;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
