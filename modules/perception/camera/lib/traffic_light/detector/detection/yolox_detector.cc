/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/camera/lib/traffic_light/detector/detection/yolox_detector.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/inference/utils/resize.h"

namespace apollo {
namespace perception {
namespace camera {

namespace {

struct SearchWindowTask {
  int light_index = -1;
  base::RectI crop_box;
};

std::map<std::string, std::vector<int>> BuildShapeMapOrDefault(
    const YoloxTrafficLightDetectionConfig& cfg, int max_batch_size) {
  std::map<std::string, std::vector<int>> shapes;
  for (const auto& blob : cfg.blobs()) {
    std::vector<int> shape(blob.shape().begin(), blob.shape().end());
    if (!shape.empty() && shape[0] == 1) {
      shape[0] = std::max(1, max_batch_size);
    }
    shapes.emplace(blob.name(), std::move(shape));
  }
  if (!shapes.empty()) {
    return shapes;
  }

  // Fallback to Apollo 10.x default shapes used by YOLOX TL detector.
  const int batch = std::max(1, max_batch_size);
  const int anchors = 3024;
  shapes.emplace(cfg.input_name(),
                 std::vector<int>{batch, 3, cfg.resize_image_height(),
                                  cfg.resize_image_width()});
  shapes.emplace(cfg.output_bbox_name(), std::vector<int>{batch, anchors, 4});
  shapes.emplace(cfg.output_conf_name(), std::vector<int>{batch, anchors});
  shapes.emplace(cfg.output_cls_name(), std::vector<int>{batch, anchors});
  return shapes;
}

base::RectI ShiftCropBox(const base::RectI& crop_box, int shift_x, int shift_y,
                         int image_width, int image_height) {
  if (crop_box.width <= 0 || crop_box.height <= 0) {
    return base::RectI(0, 0, 0, 0);
  }

  base::RectI shifted = crop_box;
  shifted.x += shift_x;
  shifted.y += shift_y;

  if (shifted.width > image_width) {
    shifted.width = image_width;
    shifted.x = 0;
  } else {
    shifted.x = std::max(0, std::min(shifted.x, image_width - shifted.width));
  }

  if (shifted.height > image_height) {
    shifted.height = image_height;
    shifted.y = 0;
  } else {
    shifted.y =
        std::max(0, std::min(shifted.y, image_height - shifted.height));
  }

  return shifted;
}

std::vector<base::RectI> BuildSearchWindows(const base::RectI& crop_box,
                                            int image_width,
                                            int image_height,
                                            int crop_method) {
  std::vector<base::RectI> windows;
  if (crop_box.Area() <= 0) {
    return windows;
  }

  if (crop_method != 0) {
    windows.push_back(crop_box);
    return windows;
  }

  windows.reserve(9);
  for (int row = -1; row <= 1; ++row) {
    for (int col = -1; col <= 1; ++col) {
      const base::RectI shifted =
          ShiftCropBox(crop_box, col * crop_box.width, row * crop_box.height,
                       image_width, image_height);
      if (shifted.Area() <= 0) {
        continue;
      }
      if (std::find(windows.begin(), windows.end(), shifted) == windows.end()) {
        windows.push_back(shifted);
      }
    }
  }

  if (windows.empty()) {
    windows.push_back(crop_box);
  }
  return windows;
}

}  // namespace

TrafficLightYoloxDetector::~TrafficLightYoloxDetector() {
  if (owns_stream_ && stream_ != 0) {
    cudaStreamDestroy(stream_);
    stream_ = 0;
  }
}

bool TrafficLightYoloxDetector::Init(
    const TrafficLightDetectionConfig& config) {
  config_ = config;
  yolox_config_ = config.yolox_config();

  gpu_id_ = config.gpu_id();
  max_batch_size_ = std::max(1, config.max_batch_size());
  cls_th_ = static_cast<float>(yolox_config_.cls_threshold());
  resize_height_ = yolox_config_.resize_image_height();
  resize_width_ = yolox_config_.resize_image_width();

  if (cudaSetDevice(gpu_id_) != cudaSuccess) {
    AERROR << "Failed to set device to: " << gpu_id_;
    return false;
  }

  if (stream_ == 0) {
    cudaError_t err = cudaStreamCreate(&stream_);
    if (err != cudaSuccess) {
      AERROR << "Failed to create cuda stream: " << cudaGetErrorString(err);
      return false;
    }
    owns_stream_ = true;
  }

  input_names_ = {yolox_config_.input_name()};
  output_names_ = {yolox_config_.output_bbox_name(),
                   yolox_config_.output_conf_name(),
                   yolox_config_.output_cls_name()};

  const std::string model_root_dir = yolox_config_.model_root_dir();
  const std::string onnx_file =
      cyber::common::GetAbsolutePath(model_root_dir, yolox_config_.onnx_file());

  infer_ = std::make_shared<inference::MultiBatchInference>();
  infer_->set_gpu_id(gpu_id_);
  infer_->set_max_batch_size(max_batch_size_);
  infer_->set_enable_fp16(yolox_config_.enable_fp16());
  infer_->set_model_info(onnx_file, input_names_, output_names_);
  infer_->SetStream(stream_);

  auto shapes = BuildShapeMapOrDefault(yolox_config_, max_batch_size_);
  if (!infer_->Init(shapes)) {
    AERROR << "Failed to init MultiBatchInference for YOLOX TL detection";
    return false;
  }

  switch (config.crop_method()) {
    default:
    case 0:
      crop_.reset(new CropBox(config.crop_scale(), config.min_crop_size()));
      break;
    case 1:
      crop_.reset(new CropBoxWholeImage());
      break;
  }

  select_.Init(config.min_crop_size(), config.min_crop_size());
  padding_image_container_ =
      base::Image8U(max_batch_roi_, max_batch_roi_, base::Color::BGR);

  return true;
}

bool TrafficLightYoloxDetector::Detect(CameraFrame* frame) {
  return DetectTL(frame);
}

bool TrafficLightYoloxDetector::PadAndCopyImage(const base::Image8U& src,
                                                base::Image8U* dst,
                                                int left_pad, int top_pad,
                                                uint8_t pad_value) {
  if (dst == nullptr) {
    return false;
  }
  const int channels = src.channels();
  if (channels != dst->channels()) {
    AERROR << "PadAndCopyImage channel mismatch";
    return false;
  }

#if USE_GPU == 0
  std::memset(dst->mutable_cpu_data(), pad_value,
              dst->rows() * dst->width_step());
  const int row_bytes =
      src.cols() * channels * static_cast<int>(sizeof(uint8_t));
  for (int r = 0; r < src.rows(); ++r) {
    auto* dst_row = dst->mutable_cpu_ptr(r + top_pad) + left_pad * channels;
    const auto* src_row = src.cpu_ptr(r);
    std::memcpy(dst_row, src_row, row_bytes);
  }
  return true;
#else
  cudaError_t err = cudaMemsetAsync(dst->mutable_gpu_data(), pad_value,
                                    dst->rows() * dst->width_step(), stream_);
  if (err != cudaSuccess) {
    AERROR << "cudaMemsetAsync failed: " << cudaGetErrorString(err);
    return false;
  }

  uint8_t* dst_ptr = dst->mutable_gpu_data() + top_pad * dst->width_step() +
                     left_pad * channels * static_cast<int>(sizeof(uint8_t));
  const uint8_t* src_ptr = src.gpu_data();
  const size_t width_bytes =
      src.cols() * channels * static_cast<size_t>(sizeof(uint8_t));

  err = cudaMemcpy2DAsync(dst_ptr, dst->width_step(), src_ptr, src.width_step(),
                          width_bytes, src.rows(), cudaMemcpyDeviceToDevice,
                          stream_);
  if (err != cudaSuccess) {
    AERROR << "cudaMemcpy2DAsync failed: " << cudaGetErrorString(err);
    return false;
  }
  // NOTE: inference::ResizeGPU launches kernels on the default stream. Without
  // synchronization here, ResizeGPU may read before the async padding/copy
  // finishes on stream_.
  err = cudaStreamSynchronize(stream_);
  if (err != cudaSuccess) {
    AERROR << "cudaStreamSynchronize failed: " << cudaGetErrorString(err);
    return false;
  }
  return true;
#endif
}

bool TrafficLightYoloxDetector::DetectTL(CameraFrame* frame) {
  detected_bboxes_.clear();
  detected_heads_.clear();
  pad_resize_params_.clear();
  detected_light_ids_.clear();
  detected_light_indices_.clear();
  detected_crop_rois_.clear();

  if (frame == nullptr) {
    return false;
  }
  if (frame->traffic_lights.empty()) {
    AINFO << "no hdmap lights to detect";
    return true;
  }
  if (cudaSetDevice(gpu_id_) != cudaSuccess) {
    AERROR << "Failed To Set Device To: " << gpu_id_;
    return false;
  }

  std::vector<base::TrafficLightPtr>& lights_ref = frame->traffic_lights;
  AINFO << "detect lights num: " << lights_ref.size();

  auto input_image_blob = infer_->get_blob(input_names_[0]);
  auto output_loc_blob = infer_->get_blob(output_names_[0]);
  auto output_conf_blob = infer_->get_blob(output_names_[1]);
  auto output_cls_blob = infer_->get_blob(output_names_[2]);

  const auto& data_provider = frame->data_provider;
  const int img_width = data_provider->src_width();
  const int img_height = data_provider->src_height();

  for (auto& light : lights_ref) {
    base::RectI debug_rect(0, 0, 0, 0);
    light->region.detection_roi = light->region.projection_roi;
    light->region.debug_roi.clear();
    light->region.debug_roi_detect_scores.clear();
    light->region.debug_roi.push_back(debug_rect);
    light->region.debug_roi_detect_scores.push_back(0.0f);
  }

  for (auto& light : lights_ref) {
    if (light->region.outside_image ||
        OutOfValidRegion(light->region.projection_roi, img_width, img_height) ||
        light->region.projection_roi.Area() <= 0) {
      light->region.projection_roi = base::RectI(0, 0, 0, 0);
    }
  }

  std::vector<SearchWindowTask> search_tasks;
  search_tasks.reserve(lights_ref.size() * 9);
  for (size_t light_idx = 0; light_idx < lights_ref.size(); ++light_idx) {
    auto& light = lights_ref[light_idx];
    base::RectI base_crop_box;
    crop_->getCropBox(img_width, img_height, light, &base_crop_box);
    if (base_crop_box.Area() <= 0) {
      light->region.crop_roi = base::RectI(0, 0, 0, 0);
      continue;
    }

    const auto windows = BuildSearchWindows(base_crop_box, img_width, img_height,
                                            config_.crop_method());
    if (windows.empty()) {
      light->region.crop_roi = base_crop_box;
      light->region.debug_roi[0] = base_crop_box;
      continue;
    }

    base::RectI search_roi = windows.front();
    for (const auto& window : windows) {
      search_roi = search_roi | window;
      search_tasks.push_back({static_cast<int>(light_idx), window});
    }

    light->region.crop_roi = search_roi;
    light->region.debug_roi[0] = search_roi;
  }

  const int batch_num =
      (static_cast<int>(search_tasks.size()) + max_batch_size_ - 1) /
      max_batch_size_;

  float debug_max_conf = -1.0f;
  int debug_cnt_ge_010 = 0;
  int debug_cnt_ge_th = 0;

  for (int batch_id = 0; batch_id < batch_num; ++batch_id) {
    const int batch_begin = batch_id * max_batch_size_;
    const int batch_end = std::min(static_cast<int>(search_tasks.size()),
                                   (batch_id + 1) * max_batch_size_);
    const int batch_size = batch_end - batch_begin;

    input_image_blob->Reshape(max_batch_size_, 3, resize_height_, resize_width_);

    const int box_num = output_loc_blob->shape(1);
    output_loc_blob->Reshape({max_batch_size_, box_num, 4});
    output_conf_blob->Reshape({max_batch_size_, box_num});
    output_cls_blob->Reshape({max_batch_size_, box_num});

#if USE_GPU == 0
    std::memset(input_image_blob->mutable_cpu_data(), 0,
                input_image_blob->count() * sizeof(float));
#else
    cudaMemsetAsync(input_image_blob->mutable_gpu_data(), 0,
                    input_image_blob->count() * sizeof(float), stream_);
    // ResizeGPU uses the default stream; make sure the async memset finishes
    // before any writes happen on another stream.
    cudaStreamSynchronize(stream_);
#endif

    const int base_offset = batch_id * max_batch_size_;
    if (static_cast<int>(pad_resize_params_.size()) <
        base_offset + max_batch_size_) {
      pad_resize_params_.resize(base_offset + max_batch_size_);
    }
    if (static_cast<int>(detected_light_ids_.size()) <
        base_offset + max_batch_size_) {
      detected_light_ids_.resize(base_offset + max_batch_size_);
    }
    if (static_cast<int>(detected_light_indices_.size()) <
        base_offset + max_batch_size_) {
      detected_light_indices_.resize(base_offset + max_batch_size_, -1);
    }
    if (static_cast<int>(detected_crop_rois_.size()) <
        base_offset + max_batch_size_) {
      detected_crop_rois_.resize(base_offset + max_batch_size_);
    }
    for (int slot = 0; slot < max_batch_size_; ++slot) {
      YoloxPadResizeParam prp;
      prp.roi_width = img_width;
      prp.roi_height = img_height;
      prp.scale = 1.0f;
      pad_resize_params_[base_offset + slot] = prp;
      detected_light_ids_[base_offset + slot].clear();
      detected_light_indices_[base_offset + slot] = -1;
      detected_crop_rois_[base_offset + slot] = base::RectI(0, 0, 0, 0);
    }

    for (int slot = 0; slot < batch_size; ++slot) {
      const int task_idx = batch_begin + slot;
      const auto& task = search_tasks[task_idx];
      if (task.light_index < 0 ||
          task.light_index >= static_cast<int>(lights_ref.size())) {
        continue;
      }
      auto& light_hdmap = lights_ref[task.light_index];
      const base::RectI& cbox = task.crop_box;

      image_options_.crop_roi = cbox;
      image_options_.do_crop = true;
      image_options_.target_color = base::Color::BGR;
      base::Image8U image;
      data_provider->GetImage(image_options_, &image);

      const int crop_roi_width = cbox.width;
      const int crop_roi_height = cbox.height;

      int left_pad = 0;
      int right_pad = 0;
      int top_pad = 0;
      int bottom_pad = 0;
      if (crop_roi_width > crop_roi_height) {
        bottom_pad = crop_roi_width - crop_roi_height;
      } else if (crop_roi_height > crop_roi_width) {
        right_pad = crop_roi_height - crop_roi_width;
      }

      const int dst_height = crop_roi_height + top_pad + bottom_pad;
      const int dst_width = crop_roi_width + left_pad + right_pad;
      if (dst_width <= 0 || dst_height <= 0 || dst_width > max_batch_roi_ ||
          dst_height > max_batch_roi_) {
        AWARN << "Invalid padded roi: " << dst_width << "x" << dst_height;
        detected_light_ids_[base_offset + slot].clear();
        detected_light_indices_[base_offset + slot] = -1;
        detected_crop_rois_[base_offset + slot] = base::RectI(0, 0, 0, 0);
        continue;
      }

      base::Image8U tmp_image =
          padding_image_container_(base::RectI(0, 0, dst_width, dst_height));

      const float scale = resize_width_ * 1.0f / static_cast<float>(dst_width);
      YoloxPadResizeParam prp;
      prp.left_pad = left_pad;
      prp.right_pad = right_pad;
      prp.top_pad = top_pad;
      prp.bottom_pad = bottom_pad;
      prp.roi_width = img_width;
      prp.roi_height = img_height;
      prp.scale = scale;
      pad_resize_params_[base_offset + slot] = prp;

      if (!PadAndCopyImage(image, &tmp_image, left_pad, top_pad,
                           static_cast<uint8_t>(yolox_config_.pad_value()))) {
        detected_light_ids_[base_offset + slot].clear();
        detected_light_indices_[base_offset + slot] = -1;
        detected_crop_rois_[base_offset + slot] = base::RectI(0, 0, 0, 0);
        continue;
      }

      detected_light_ids_[base_offset + slot] = light_hdmap->id;
      detected_light_indices_[base_offset + slot] = task.light_index;
      detected_crop_rois_[base_offset + slot] = cbox;

      inference::ResizeGPU(tmp_image, input_image_blob, dst_width, slot,
                           0.0f, 0.0f, 0.0f, false, 1.0f);
    }

    cudaDeviceSynchronize();
    infer_->Infer();
    cudaDeviceSynchronize();

    {
      const int anchor_num = output_conf_blob->shape(1);
      const float* conf_data_cpu = output_conf_blob->cpu_data();
      for (int i = 0; i < batch_size; ++i) {
        const float* conf = conf_data_cpu + i * anchor_num;
        for (int j = 0; j < anchor_num; ++j) {
          const float c = conf[j];
          if (c > debug_max_conf) {
            debug_max_conf = c;
          }
          if (c >= 0.10f) {
            ++debug_cnt_ge_010;
          }
          if (c >= cls_th_) {
            ++debug_cnt_ge_th;
          }
        }
      }
    }

    GetCandidateHeads(batch_id, batch_size, &detected_bboxes_, lights_ref);
  }

  NMS(&detected_bboxes_, frame->data_provider->sensor_name());
  for (size_t j = 0; j < detected_bboxes_.size(); ++j) {
    base::RectI& region = detected_bboxes_[j]->region.detection_roi;
    float score = detected_bboxes_[j]->region.detect_score;
    lights_ref[0]->region.debug_roi.push_back(region);
    lights_ref[0]->region.debug_roi_detect_scores.push_back(score);
  }

  select_.SelectTrafficLights(detected_bboxes_, &lights_ref);
  int detected_num = 0;
  for (const auto& light : lights_ref) {
    if (light->region.is_detected) {
      ++detected_num;
    }
  }
  if (detected_num == 0) {
    static int no_det_warn_cnt = 0;
    if ((++no_det_warn_cnt % 50) == 1) {
      AWARN << "TL_DET_DEBUG no detected lights. hdmap_lights: "
            << lights_ref.size() << ", candidates(after_nms): "
            << detected_bboxes_.size() << ", search_tasks: "
            << search_tasks.size() << ", camera: "
            << frame->data_provider->sensor_name()
            << ", projection_roi: " << lights_ref[0]->region.projection_roi.x
            << " " << lights_ref[0]->region.projection_roi.y << " "
            << lights_ref[0]->region.projection_roi.width << " "
            << lights_ref[0]->region.projection_roi.height << ", crop_roi: "
            << lights_ref[0]->region.crop_roi.x << " "
            << lights_ref[0]->region.crop_roi.y << " "
            << lights_ref[0]->region.crop_roi.width << " "
            << lights_ref[0]->region.crop_roi.height << ", max_conf: "
            << debug_max_conf << ", cnt_conf>=0.10: " << debug_cnt_ge_010
            << ", cnt_conf>=th(" << cls_th_ << "): " << debug_cnt_ge_th;
    }
  }
  return true;
}

bool TrafficLightYoloxDetector::GetCandidateHeads(
    int batch_id, int valid_batch_size, std::vector<base::TrafficLightPtr>* out,
    std::vector<base::TrafficLightPtr>& lights_ref) {
  auto output_loc_blob = infer_->get_blob(output_names_[0]);
  auto output_conf_blob = infer_->get_blob(output_names_[1]);
  auto output_cls_blob = infer_->get_blob(output_names_[2]);

  const int batch_size = std::min(valid_batch_size, max_batch_size_);
  const int anchor_num = output_cls_blob->shape()[1];
  const auto* conf_data_cpu = output_conf_blob->cpu_data();
  const auto* bbx_data_cpu = output_loc_blob->cpu_data();
  const auto* cls_data_cpu =
      reinterpret_cast<const int*>(output_cls_blob->cpu_data());

  const int bbox_dims = 4;
  for (int i = 0; i < batch_size; ++i) {
    const auto* conf = conf_data_cpu + i * anchor_num;
    for (int j = 0; j < anchor_num; ++j) {
      if (conf[j] < cls_th_) {
        continue;
      }

      const int light_idx = batch_id * max_batch_size_ + i;
      if (light_idx < 0 ||
          light_idx >= static_cast<int>(pad_resize_params_.size()) ||
          light_idx >= static_cast<int>(detected_light_ids_.size()) ||
          light_idx >= static_cast<int>(detected_light_indices_.size()) ||
          light_idx >= static_cast<int>(detected_crop_rois_.size())) {
        continue;
      }
      const int hdmap_light_idx = detected_light_indices_[light_idx];
      if (hdmap_light_idx < 0 ||
          hdmap_light_idx >= static_cast<int>(lights_ref.size())) {
        continue;
      }

      base::TrafficLightPtr tmp(new base::TrafficLight);
      const YoloxPadResizeParam prp = pad_resize_params_[light_idx];
      const base::RectI& crop_roi = detected_crop_rois_[light_idx];

      const float x = bbx_data_cpu[i * anchor_num * bbox_dims + j * bbox_dims];
      const float y =
          bbx_data_cpu[i * anchor_num * bbox_dims + j * bbox_dims + 1];
      const float w =
          bbx_data_cpu[i * anchor_num * bbox_dims + j * bbox_dims + 2];
      const float h =
          bbx_data_cpu[i * anchor_num * bbox_dims + j * bbox_dims + 3];

      const double head_area = (w / prp.scale) * (h / prp.scale);
      if (head_area < min_head_area_) {
        continue;
      }

      const int shape_idx = i * anchor_num + j;
      tmp->region.detect_class_id =
          base::TLDetectionClass(cls_data_cpu[shape_idx]);

      tmp->region.detection_roi.width = static_cast<int>(w / prp.scale);
      tmp->region.detection_roi.height = static_cast<int>(h / prp.scale);
      tmp->region.detection_roi.x =
          static_cast<int>(x / prp.scale - prp.left_pad + crop_roi.x -
                           tmp->region.detection_roi.width / 2.0);
      tmp->region.detection_roi.y =
          static_cast<int>(y / prp.scale - prp.top_pad + crop_roi.y -
                           tmp->region.detection_roi.height / 2.0);
      tmp->region.detect_score = conf[j];
      tmp->region.is_detected = true;
      tmp->id = detected_light_ids_[light_idx];

      if (OutOfValidRegion(tmp->region.detection_roi, prp.roi_width,
                           prp.roi_height) ||
          tmp->region.detection_roi.Area() <= 0) {
        continue;
      }

      RefineBox(tmp->region.detection_roi, prp.roi_width, prp.roi_height,
                &(tmp->region.detection_roi));
      out->push_back(tmp);
    }
  }
  return true;
}

void TrafficLightYoloxDetector::NMS(std::vector<base::TrafficLightPtr>* lights,
                                    const std::string& camera_name) {
  for (const auto& light : *lights) {
    detected_heads_[camera_name].push_back(light);
  }
  lights->clear();
  for (auto& entry : detected_heads_) {
    auto lights_for_each_camera = entry.second;
    ApplyOverlapNMS(&lights_for_each_camera);
    ApplyNMS(&lights_for_each_camera);
    for (auto& light : lights_for_each_camera) {
      lights->push_back(light);
    }
  }
}

void TrafficLightYoloxDetector::ApplyOverlapNMS(
    std::vector<base::TrafficLightPtr>* lights) {
  std::vector<std::pair<int, int>> area_index(lights->size());
  for (size_t i = 0; i < lights->size(); ++i) {
    area_index[i].first = lights->at(i)->region.detection_roi.width *
                          lights->at(i)->region.detection_roi.height;
    area_index[i].second = static_cast<int>(i);
  }
  std::stable_sort(
      area_index.begin(), area_index.end(),
      [](const auto& a, const auto& b) { return a.first < b.first; });

  std::vector<int> kept;
  while (!area_index.empty()) {
    const int idx = area_index.back().second;
    bool keep = true;
    for (int kept_idx : kept) {
      const auto& rect1 = lights->at(idx)->region.detection_roi;
      const auto& rect2 = lights->at(kept_idx)->region.detection_roi;
      const float overlap =
          static_cast<float>((rect1 & rect2).Area()) / rect1.Area();
      keep = std::fabs(overlap) < yolox_config_.overlap_nms_threshold();
      if (!keep) break;
    }
    if (keep) kept.push_back(idx);
    area_index.pop_back();
  }

  int idx = 0;
  auto it = std::stable_partition(
      lights->begin(), lights->end(), [&](const base::TrafficLightPtr&) {
        return std::find(kept.begin(), kept.end(), idx++) != kept.end();
      });
  lights->erase(it, lights->end());
}

void TrafficLightYoloxDetector::ApplyNMS(
    std::vector<base::TrafficLightPtr>* lights) {
  std::vector<std::pair<float, int>> score_index(lights->size());
  for (size_t i = 0; i < lights->size(); ++i) {
    score_index[i].first = lights->at(i)->region.detect_score;
    score_index[i].second = static_cast<int>(i);
  }
  std::stable_sort(
      score_index.begin(), score_index.end(),
      [](const auto& a, const auto& b) { return a.first < b.first; });

  std::vector<int> kept;
  while (!score_index.empty()) {
    const int idx = score_index.back().second;
    bool keep = true;
    for (int kept_idx : kept) {
      const auto& rect1 = lights->at(idx)->region.detection_roi;
      const auto& rect2 = lights->at(kept_idx)->region.detection_roi;
      const float iou =
          static_cast<float>((rect1 & rect2).Area()) / (rect1 | rect2).Area();
      keep = std::fabs(iou) < yolox_config_.iou_nms_threshold();
      if (!keep) break;
    }
    if (keep) kept.push_back(idx);
    score_index.pop_back();
  }

  int idx = 0;
  auto it = std::stable_partition(
      lights->begin(), lights->end(), [&](const base::TrafficLightPtr&) {
        return std::find(kept.begin(), kept.end(), idx++) != kept.end();
      });
  lights->erase(it, lights->end());
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
