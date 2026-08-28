#pragma once

#include <memory>
#include <vector>

#include "modules/lidar_semantic_segmentation/inference/rangeret.h"

namespace apollo {
namespace lidar_semantic_segmentation {

class TensorRtRangeRetExecutor final : public RangeRetExecutor {
 public:
  TensorRtRangeRetExecutor();
  ~TensorRtRangeRetExecutor() override;

  TensorRtRangeRetExecutor(const TensorRtRangeRetExecutor&) = delete;
  TensorRtRangeRetExecutor& operator=(const TensorRtRangeRetExecutor&) = delete;

  bool Init(const RangeRetModelOptions& options) override;
  bool Run(const std::vector<float>& input, RangeRetTensor* output) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
