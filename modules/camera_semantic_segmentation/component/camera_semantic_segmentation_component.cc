// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/camera_semantic_segmentation/component/camera_semantic_segmentation_component.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace camera_semantic_segmentation {

namespace {

void CopyPreprocessOptions(const ImagePreprocessConfig& config,
                           ImagePreprocessOptions* options) {
  options->width = config.width();
  options->height = config.height();
  options->to_rgb = config.to_rgb();
  if (config.channel_mean_size() > 0) {
    options->channel_mean.assign(config.channel_mean().begin(),
                                 config.channel_mean().end());
  }
  if (config.channel_std_size() > 0) {
    options->channel_std.assign(config.channel_std().begin(),
                                config.channel_std().end());
  }
}

bool ToCameraTimestamp(double timestamp_sec, uint64_t* timestamp_ns) {
  if (timestamp_ns == nullptr || !std::isfinite(timestamp_sec) ||
      timestamp_sec < 0.0 ||
      timestamp_sec >
          static_cast<double>(std::numeric_limits<uint64_t>::max()) * 1.0e-9) {
    return false;
  }
  *timestamp_ns = static_cast<uint64_t>(timestamp_sec * 1.0e9);
  return true;
}

}  // namespace

bool CameraSemanticSegmentationComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Unable to load camera semantic segmentation config";
    return false;
  }
  if (config_.engine_path().empty() || config_.source_topic().empty()) {
    AERROR << "SegFormer engine_path and source_topic must be configured";
    return false;
  }
  SegFormerModelOptions options;
  if (!BuildOptions(config_, &options)) {
    return false;
  }
  executor_.reset(new TensorRtSegFormerExecutor);
  segmenter_.reset(new SegFormerSegmenter(executor_.get()));
  std::string error;
  if (!segmenter_->Init(options, &error)) {
    AERROR << "Unable to initialize SegFormer segmenter: " << error;
    return false;
  }
  writer_ = node_->CreateWriter<CameraSemanticSegmentationResult>(
      config_.output_channel());
  return writer_ != nullptr;
}

bool CameraSemanticSegmentationComponent::Proc(
    const std::shared_ptr<apollo::drivers::Image>& image) {
  if (image == nullptr || segmenter_ == nullptr || writer_ == nullptr) {
    return false;
  }
  ImageView image_view;
  if (!MakeImageView(*image, &image_view)) {
    return false;
  }
  auto result = std::make_shared<CameraSemanticSegmentationResult>();
  std::string error;
  if (!segmenter_->Segment(image_view, result.get(), &error)) {
    AERROR << "SegFormer camera semantic segmentation failed: " << error;
    return false;
  }
  writer_->Write(result);
  return true;
}

bool CameraSemanticSegmentationComponent::BuildOptions(
    const CameraSemanticSegmentationComponentConfig& config,
    SegFormerModelOptions* options) const {
  if (options == nullptr || !config.has_preprocess() || !config.has_tensor()) {
    AERROR << "SegFormer preprocess and tensor config must be present";
    return false;
  }
  options->engine_path = config.engine_path();
  options->device_id = config.gpu_device_id();
  options->camera_name = config.camera_name();
  options->source_topic = config.source_topic();
  options->tensor_names.input = config.tensor().input_tensor_name();
  options->tensor_names.output = config.tensor().output_tensor_name();
  options->num_classes = config.tensor().num_classes();
  options->output_layout = config.tensor().output_layout();
  CopyPreprocessOptions(config.preprocess(), &options->preprocess);
  return true;
}

bool CameraSemanticSegmentationComponent::MakeImageView(
    const apollo::drivers::Image& message, ImageView* image_view) const {
  if (image_view == nullptr) {
    return false;
  }
  ImageEncoding encoding;
  std::size_t expected_byte_count = 0;
  if (!ParseImageEncoding(message.encoding(), &encoding) ||
      !ImageView::ExpectedByteCount(message.width(), message.height(),
                                    &expected_byte_count) ||
      message.step() != expected_byte_count / message.height() ||
      message.data().size() != expected_byte_count) {
    AERROR << "Rejected image payload with unsupported encoding, dimensions, "
              "stride, or byte count";
    return false;
  }
  image_view->bytes = reinterpret_cast<const uint8_t*>(message.data().data());
  image_view->byte_count = message.data().size();
  image_view->width = message.width();
  image_view->height = message.height();
  image_view->encoding = encoding;
  image_view->timestamp_sec = message.measurement_time();
  if (message.has_header()) {
    const auto& header = message.header();
    if (header.has_camera_timestamp()) {
      image_view->camera_timestamp_ns = header.camera_timestamp();
    }
    if (header.has_sequence_num()) {
      image_view->sequence_num = header.sequence_num();
    }
  }
  if (image_view->camera_timestamp_ns == 0) {
    ToCameraTimestamp(image_view->timestamp_sec,
                      &image_view->camera_timestamp_ns);
  }
  image_view->frame_id = message.frame_id();
  if (image_view->frame_id.empty() && message.has_header() &&
      message.header().has_frame_id()) {
    image_view->frame_id = message.header().frame_id();
  }
  image_view->camera_name = config_.camera_name();
  std::string error;
  if (!image_view->Validate(&error)) {
    AERROR << "Invalid image view: " << error;
    return false;
  }
  return true;
}

}  // namespace camera_semantic_segmentation
}  // namespace apollo

CYBER_REGISTER_COMPONENT(
    apollo::camera_semantic_segmentation::CameraSemanticSegmentationComponent)

