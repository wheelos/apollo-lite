#include "modules/lidar_semantic_segmentation/inference/rangeret.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace apollo {
namespace lidar_semantic_segmentation {

namespace {

void SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

bool IsSupportedLayout(const std::string& layout) {
  return layout == "NHWC" || layout == "NCHW";
}

}  // namespace

bool RangeRetSegmenter::Init(const RangeRetModelOptions& options,
                             std::string* error) {
  if (executor_ == nullptr) {
    SetError("RangeRet executor is null", error);
    return false;
  }
  if (options.num_classes == 0U) {
    SetError("RangeRet num_classes must be positive", error);
    return false;
  }
  if (!IsSupportedLayout(options.output_layout)) {
    SetError("RangeRet output_layout must be NHWC or NCHW", error);
    return false;
  }
  if (!projector_.Init(options.projection, error)) {
    return false;
  }
  if (!executor_->Init(options)) {
    SetError("failed to initialize RangeRet executor", error);
    return false;
  }
  options_ = options;
  output_is_nhwc_ = options.output_layout == "NHWC";
  initialized_ = true;
  return true;
}

bool RangeRetSegmenter::Segment(
    const apollo::drivers::PointCloud& cloud,
    LidarSemanticSegmentationResult* result, std::string* error) const {
  if (!initialized_) {
    SetError("RangeRet segmenter is not initialized", error);
    return false;
  }
  if (result == nullptr) {
    SetError("segmentation result output is null", error);
    return false;
  }

  RangeImage image;
  if (!projector_.Project(cloud, &image, error)) {
    return false;
  }

  RangeRetTensor logits;
  if (!executor_->Run(image.input_chw, &logits)) {
    SetError("RangeRet executor failed", error);
    return false;
  }

  std::vector<SemanticPointPrediction> predictions;
  if (!Decode(image, logits, &predictions, error)) {
    return false;
  }

  result->Clear();
  if (cloud.has_header()) {
    *result->mutable_header() = cloud.header();
  }
  result->set_source_topic(options_.source_topic);
  result->set_source_frame_id(cloud.frame_id());
  result->set_sensor_name(options_.sensor_name);
  result->set_point_count(static_cast<uint32_t>(predictions.size()));
  result->set_range_image_width(image.width);
  result->set_range_image_height(image.height);
  result->set_num_classes(options_.num_classes);
  for (std::size_t index = 0; index < predictions.size(); ++index) {
    auto* label = result->add_point_label();
    label->set_point_index(static_cast<uint32_t>(index));
    label->set_semantic_label(predictions[index].label);
    label->set_confidence(predictions[index].confidence);
    if (image.points[index].valid) {
      label->set_range_image_x(static_cast<int32_t>(image.points[index].x));
      label->set_range_image_y(static_cast<int32_t>(image.points[index].y));
    }
  }
  return true;
}

bool RangeRetSegmenter::Decode(
    const RangeImage& image, const RangeRetTensor& logits,
    std::vector<SemanticPointPrediction>* predictions,
    std::string* error) const {
  if (predictions == nullptr) {
    SetError("prediction output is null", error);
    return false;
  }
  const std::size_t pixel_count = image.PixelCount();
  const std::size_t expected_count =
      pixel_count * static_cast<std::size_t>(options_.num_classes);
  if (logits.values.size() != expected_count) {
    SetError("RangeRet output size does not match projection geometry", error);
    return false;
  }
  predictions->assign(image.points.size(), SemanticPointPrediction());
  for (std::size_t point_index = 0; point_index < image.points.size();
       ++point_index) {
    const ProjectedPoint& projected = image.points[point_index];
    if (!projected.valid) {
      continue;
    }
    uint32_t best_label = 0U;
    float best_logit = -std::numeric_limits<float>::infinity();
    double exp_sum = 0.0;
    for (uint32_t cls = 0U; cls < options_.num_classes; ++cls) {
      const float value = logits.values[LogitOffset(projected.y, projected.x,
                                                    cls)];
      if (!std::isfinite(value)) {
        SetError("RangeRet output contains non-finite logits", error);
        return false;
      }
      if (value > best_logit) {
        best_logit = value;
        best_label = cls;
      }
    }
    for (uint32_t cls = 0U; cls < options_.num_classes; ++cls) {
      exp_sum += std::exp(static_cast<double>(
          logits.values[LogitOffset(projected.y, projected.x, cls)] -
          best_logit));
    }
    (*predictions)[point_index].label = best_label;
    (*predictions)[point_index].confidence =
        exp_sum > 0.0 ? static_cast<float>(1.0 / exp_sum) : 0.0F;
  }
  return true;
}

std::size_t RangeRetSegmenter::LogitOffset(uint32_t y, uint32_t x,
                                           uint32_t cls) const {
  const std::size_t height = options_.projection.height;
  const std::size_t width = options_.projection.width;
  const std::size_t classes = options_.num_classes;
  if (output_is_nhwc_) {
    return ((static_cast<std::size_t>(y) * width + x) * classes) + cls;
  }
  return ((static_cast<std::size_t>(cls) * height + y) * width) + x;
}

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
