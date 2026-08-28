#pragma once

#include <memory>

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/component/component.h"
#include "modules/lidar_semantic_segmentation/inference/rangeret.h"
#include "modules/lidar_semantic_segmentation/inference/tensorrt_executor.h"
#include "modules/lidar_semantic_segmentation/proto/lidar_semantic_segmentation.pb.h"

namespace apollo {
namespace lidar_semantic_segmentation {

class LidarSemanticSegmentationComponent final
    : public cyber::Component<apollo::drivers::PointCloud> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<apollo::drivers::PointCloud>& message) override;

 private:
  bool BuildOptions(const LidarSemanticSegmentationComponentConfig& config,
                    RangeRetModelOptions* options) const;

  LidarSemanticSegmentationComponentConfig config_;
  std::unique_ptr<TensorRtRangeRetExecutor> executor_;
  std::unique_ptr<RangeRetSegmenter> segmenter_;
  std::shared_ptr<cyber::Writer<LidarSemanticSegmentationResult>> writer_;
};

CYBER_REGISTER_COMPONENT(LidarSemanticSegmentationComponent)

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
