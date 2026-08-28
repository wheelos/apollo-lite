#include "modules/lidar_semantic_segmentation/component/lidar_semantic_segmentation_component.h"

#include <string>

#include "cyber/common/log.h"

namespace apollo {
namespace lidar_semantic_segmentation {

namespace {

void CopyProjectionOptions(const RangeProjectionConfig& config,
                           RangeImageProjectionOptions* options) {
  options->width = config.width();
  options->height = config.height();
  options->fov_up_degrees = config.fov_up_degrees();
  options->fov_down_degrees = config.fov_down_degrees();
  options->max_points = config.max_points();
  if (config.channel_mean_size() > 0) {
    options->channel_mean.assign(config.channel_mean().begin(),
                                 config.channel_mean().end());
  }
  if (config.channel_std_size() > 0) {
    options->channel_std.assign(config.channel_std().begin(),
                                config.channel_std().end());
  }
}

}  // namespace

bool LidarSemanticSegmentationComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Unable to load LiDAR semantic segmentation config";
    return false;
  }
  if (config_.engine_path().empty() || config_.source_topic().empty()) {
    AERROR << "RangeRet engine_path and source_topic must be configured";
    return false;
  }
  RangeRetModelOptions options;
  if (!BuildOptions(config_, &options)) {
    return false;
  }
  executor_.reset(new TensorRtRangeRetExecutor);
  segmenter_.reset(new RangeRetSegmenter(executor_.get()));
  std::string error;
  if (!segmenter_->Init(options, &error)) {
    AERROR << "Unable to initialize RangeRet segmenter: " << error;
    return false;
  }
  writer_ = node_->CreateWriter<LidarSemanticSegmentationResult>(
      config_.output_channel());
  return writer_ != nullptr;
}

bool LidarSemanticSegmentationComponent::Proc(
    const std::shared_ptr<apollo::drivers::PointCloud>& message) {
  if (message == nullptr || segmenter_ == nullptr || writer_ == nullptr) {
    return false;
  }
  auto result = std::make_shared<LidarSemanticSegmentationResult>();
  std::string error;
  if (!segmenter_->Segment(*message, result.get(), &error)) {
    AERROR << "RangeRet LiDAR semantic segmentation failed: " << error;
    return false;
  }
  writer_->Write(result);
  return true;
}

bool LidarSemanticSegmentationComponent::BuildOptions(
    const LidarSemanticSegmentationComponentConfig& config,
    RangeRetModelOptions* options) const {
  if (options == nullptr || !config.has_projection() || !config.has_tensor()) {
    AERROR << "RangeRet projection and tensor config must be present";
    return false;
  }
  options->engine_path = config.engine_path();
  options->device_id = config.gpu_device_id();
  options->sensor_name = config.sensor_name();
  options->source_topic = config.source_topic();
  options->tensor_names.input = config.tensor().input_tensor_name();
  options->tensor_names.output = config.tensor().output_tensor_name();
  options->num_classes = config.tensor().num_classes();
  options->output_layout = config.tensor().output_layout();
  CopyProjectionOptions(config.projection(), &options->projection);
  return true;
}

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
