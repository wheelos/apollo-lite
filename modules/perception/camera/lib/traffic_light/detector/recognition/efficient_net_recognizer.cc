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

#include "modules/perception/camera/lib/traffic_light/detector/recognition/efficient_net_recognizer.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <utility>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/inference/utils/resize.h"

namespace apollo {
namespace perception {
namespace camera {

namespace {

std::map<std::string, std::vector<int>> BuildShapeMapOrDefault(
    const EfficientNetRecognitionConfig& cfg) {
  std::map<std::string, std::vector<int>> shapes;
  for (const auto& blob : cfg.blobs()) {
    std::vector<int> shape(blob.shape().begin(), blob.shape().end());
    if (!shape.empty() && shape[0] == 1) {
      shape[0] = std::max(1, cfg.max_batch_size());
    }
    shapes.emplace(blob.name(), std::move(shape));
  }
  if (!shapes.empty()) {
    return shapes;
  }

  const int batch = std::max(1, cfg.max_batch_size());
  shapes.emplace(cfg.input_name(),
                 std::vector<int>{batch, 3, cfg.classify_resize_height(),
                                  cfg.classify_resize_width()});
  shapes.emplace(cfg.output_cls_name(), std::vector<int>{batch, 4});
  shapes.emplace(cfg.output_status_name(), std::vector<int>{batch, 1});
  return shapes;
}

}  // namespace

TrafficLightEfficientNetRecognizer::~TrafficLightEfficientNetRecognizer() {
  if (owns_stream_ && stream_ != 0) {
    cudaStreamDestroy(stream_);
    stream_ = 0;
  }
}

bool TrafficLightEfficientNetRecognizer::Init(
    const TrafficLightRecognitionConfig& config) {
  config_ = config;
  eff_cfg_ = config.efficient_net();

  gpu_id_ = eff_cfg_.gpu_id();
  resize_height_ = eff_cfg_.classify_resize_height();
  resize_width_ = eff_cfg_.classify_resize_width();
  unknown_threshold_ = static_cast<float>(eff_cfg_.classify_threshold());
  scale_ = static_cast<float>(eff_cfg_.scale());
  is_bgr_ = eff_cfg_.is_bgr();
  max_batch_size_ = std::max(1, eff_cfg_.max_batch_size());

  if (is_bgr_) {
    mean_[0] = static_cast<float>(eff_cfg_.mean_b());
    mean_[1] = static_cast<float>(eff_cfg_.mean_g());
    mean_[2] = static_cast<float>(eff_cfg_.mean_r());
  } else {
    // ResizeGPU expects mean in BGR order even when input is RGB.
    mean_[0] = static_cast<float>(eff_cfg_.mean_r());
    mean_[1] = static_cast<float>(eff_cfg_.mean_g());
    mean_[2] = static_cast<float>(eff_cfg_.mean_b());
  }

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

  const std::string onnx_file = cyber::common::GetAbsolutePath(
      eff_cfg_.model_root_dir(), eff_cfg_.onnx_file());

  infer_ = std::make_shared<inference::MultiBatchInference>();
  infer_->set_gpu_id(gpu_id_);
  infer_->set_max_batch_size(max_batch_size_);
  infer_->set_enable_fp16(eff_cfg_.enable_fp16());
  infer_->set_model_info(
      onnx_file, {eff_cfg_.input_name()},
      {eff_cfg_.output_cls_name(), eff_cfg_.output_status_name()});
  infer_->SetStream(stream_);

  auto shapes = BuildShapeMapOrDefault(eff_cfg_);
  if (!infer_->Init(shapes)) {
    AERROR << "Failed to init MultiBatchInference for EfficientNet TL";
    return false;
  }

  input_blob_ = infer_->get_blob(eff_cfg_.input_name());
  outputs_cls_ = infer_->get_blob(eff_cfg_.output_cls_name());
  outputs_status_ = infer_->get_blob(eff_cfg_.output_status_name());
  if (input_blob_ == nullptr || outputs_cls_ == nullptr ||
      outputs_status_ == nullptr) {
    AERROR << "Failed to get EfficientNet blobs";
    return false;
  }

  image_.reset(
      new base::Image8U(resize_height_, resize_width_, base::Color::BGR));
  return true;
}

bool TrafficLightEfficientNetRecognizer::Recognize(CameraFrame* frame) {
  if (frame == nullptr) {
    return false;
  }
  if (frame->traffic_lights.empty()) {
    AINFO << "No recognizable traffic lights";
    return true;
  }

  std::vector<base::TrafficLightPtr> candidate;
  candidate.reserve(frame->traffic_lights.size());
  for (const auto& light : frame->traffic_lights) {
    if (light->region.is_detected) {
      candidate.push_back(light);
    } else {
      light->status.color = base::TLColor::TL_UNKNOWN_COLOR;
      light->status.confidence = 0;
    }
  }

  if (candidate.empty()) {
    static int no_rec_warn_cnt = 0;
    if ((++no_rec_warn_cnt % 50) == 1) {
      AWARN << "TL_REC_DEBUG no recognized candidates (is_detected=false for all). "
            << "hdmap_lights: " << frame->traffic_lights.size() << ", camera: "
            << frame->data_provider->sensor_name();
    }
    return true;
  }

  Perform(frame, &candidate);
  return true;
}

void TrafficLightEfficientNetRecognizer::Perform(
    const CameraFrame* frame, std::vector<base::TrafficLightPtr>* tl) {
  if (tl == nullptr || tl->empty()) {
    return;
  }

  const int total = static_cast<int>(tl->size());
  const int provider_w = frame->data_provider->src_width();
  const int provider_h = frame->data_provider->src_height();

  for (int batch_begin = 0; batch_begin < total; batch_begin += max_batch_size_) {
    const int batch_size = std::min(max_batch_size_, total - batch_begin);

    // Keep a fixed batch size to match TensorRT engine static batch.
    input_blob_->Reshape({max_batch_size_, 3, resize_height_, resize_width_});
    outputs_cls_->Reshape({max_batch_size_, 4});
    outputs_status_->Reshape({max_batch_size_, 1});

#if USE_GPU == 0
    std::memset(input_blob_->mutable_cpu_data(), 0,
                input_blob_->count() * sizeof(float));
#else
    cudaMemsetAsync(input_blob_->mutable_gpu_data(), 0,
                    input_blob_->count() * sizeof(float), stream_);
    // ResizeGPU uses the default stream; make sure the async memset finishes
    // before any writes happen on another stream.
    cudaStreamSynchronize(stream_);
#endif

    for (int i = 0; i < batch_size; ++i) {
      auto& light = tl->at(batch_begin + i);
      image_options_.crop_roi = light->region.detection_roi;
      image_options_.do_crop = true;
      image_options_.target_color = base::Color::BGR;
      frame->data_provider->GetImage(image_options_, image_.get());

      base::Image8U padded;
      int dst_h = 0;
      int dst_w = 0;
      ZeroPadding(light->region.detection_roi, &padded, &dst_w, &dst_h,
                  provider_w, provider_h);

      inference::ResizeGPU(padded, input_blob_, padded.cols(), i, mean_[0],
                           mean_[1], mean_[2], false, scale_);
    }

    cudaDeviceSynchronize();
    infer_->Infer();
    cudaDeviceSynchronize();

    for (int i = 0; i < batch_size; ++i) {
      auto& light = tl->at(batch_begin + i);
      float* output = outputs_cls_->mutable_cpu_data() + outputs_cls_->offset(i);
      Prob2Color(output, unknown_threshold_, light);
    }
  }
}

bool TrafficLightEfficientNetRecognizer::PadAndCopyImage(
    const base::Image8U& src, base::Image8U* dst, int left_pad, int top_pad,
    uint8_t pad_value) {
  if (dst == nullptr) {
    return false;
  }
  const int channels = src.channels();
  if (channels != dst->channels()) {
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

void TrafficLightEfficientNetRecognizer::ZeroPadding(
    const base::RectI& roi, base::Image8U* padded, int* dst_width,
    int* dst_height, int provider_w, int provider_h) {
  int roi_width = roi.width;
  int roi_height = roi.height;
  const int max_len = std::max(roi_height, roi_width);
  const int min_len = std::min(roi_height, roi_width);
  const float edge_ratio =
      static_cast<float>(max_len) / static_cast<float>(min_len);

  int pad_left = 0;
  int pad_right = 0;
  int pad_up = 0;
  int pad_down = 0;
  if (edge_ratio > 1.4f && roi_height > roi_width) {
    pad_left = (roi_height - roi_width) / 2;
    pad_right = roi_height - roi_width - pad_left;
  } else if (edge_ratio > 1.4f && roi_width > roi_height) {
    pad_up = (roi_width - roi_height) / 2;
    pad_down = roi_width - roi_height - pad_up;
  } else {
    const int height_dis = (max_len - roi_height) / 2;
    const int width_dis = (max_len - roi_width) / 2;
    pad_up = max_len + height_dis;
    pad_down = 3 * max_len - roi_height - pad_up;
    pad_left = max_len + width_dis;
    pad_right = 3 * max_len - roi_width - pad_left;
  }

  *dst_height = roi_height + pad_up + pad_down;
  *dst_width = roi_width + pad_left + pad_right;
  if (*dst_height > provider_h) {
    const int dist = *dst_height - provider_h;
    pad_up = pad_up - dist / 2 - 1;
    pad_down = pad_down - dist / 2 - 1;
  }
  if (*dst_width > provider_w) {
    const int dist = *dst_width - provider_w;
    pad_left = pad_left - dist / 2 - 1;
    pad_right = pad_right - dist / 2 - 1;
  }

  *dst_height = roi_height + pad_up + pad_down;
  *dst_width = roi_width + pad_left + pad_right;
  *padded = base::Image8U(*dst_height, *dst_width, base::Color::BGR);
  PadAndCopyImage(*image_, padded, pad_left, pad_up, 0);
}

void TrafficLightEfficientNetRecognizer::Prob2Color(
    const float* output_data, float threshold,
    const base::TrafficLightPtr& light) {
  const std::vector<base::TLColor> status_map = {
      base::TLColor::TL_BLACK, base::TLColor::TL_RED, base::TLColor::TL_YELLOW,
      base::TLColor::TL_GREEN};
  std::vector<float> prob(output_data, output_data + status_map.size());
  auto max_prob = std::max_element(prob.begin(), prob.end());
  const int max_color_id =
      (*max_prob > threshold)
          ? static_cast<int>(std::distance(prob.begin(), max_prob))
          : 0;
  light->status.color = status_map[max_color_id];
  light->status.confidence = output_data[max_color_id];
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
