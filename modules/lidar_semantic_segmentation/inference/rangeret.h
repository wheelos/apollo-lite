#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

#include "modules/lidar_semantic_segmentation/proto/lidar_semantic_segmentation.pb.h"
#include "modules/lidar_semantic_segmentation/types/range_projection.h"
#include "modules/lidar_semantic_segmentation/types/semantic_types.h"

namespace apollo {
namespace lidar_semantic_segmentation {

struct RangeRetTensorNames {
  std::string input = "input";
  std::string output = "output";
};

struct RangeRetModelOptions {
  std::string engine_path;
  int device_id = 0;
  uint32_t num_classes = 20U;
  std::string output_layout = "NHWC";
  std::string sensor_name;
  std::string source_topic;
  RangeRetTensorNames tensor_names;
  RangeImageProjectionOptions projection;
};

struct RangeRetTensor {
  std::vector<int64_t> shape;
  std::vector<float> values;
};

class RangeRetExecutor {
 public:
  virtual ~RangeRetExecutor() = default;
  virtual bool Init(const RangeRetModelOptions& options) = 0;
  virtual bool Run(const std::vector<float>& input,
                   RangeRetTensor* output) = 0;
};

class RangeRetSegmenter {
 public:
  explicit RangeRetSegmenter(RangeRetExecutor* executor) : executor_(executor) {}

  bool Init(const RangeRetModelOptions& options, std::string* error);

  bool Segment(const apollo::drivers::PointCloud& cloud,
               LidarSemanticSegmentationResult* result,
               std::string* error) const;

  bool Decode(const RangeImage& image, const RangeRetTensor& logits,
              std::vector<SemanticPointPrediction>* predictions,
              std::string* error) const;

 private:
  std::size_t LogitOffset(uint32_t y, uint32_t x, uint32_t cls) const;

  RangeRetExecutor* executor_ = nullptr;
  RangeRetModelOptions options_;
  RangeImageProjector projector_;
  bool output_is_nhwc_ = true;
  bool initialized_ = false;
};

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
