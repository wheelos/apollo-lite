#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace apollo {
namespace lidar_semantic_segmentation {

constexpr std::size_t kRangeRetInputChannels = 5U;

struct RangeImageProjectionOptions {
  uint32_t width = 1024U;
  uint32_t height = 64U;
  float fov_up_degrees = 3.0F;
  float fov_down_degrees = -25.0F;
  std::vector<float> channel_mean = {12.12F, 10.88F, 0.23F, -1.04F, 0.21F};
  std::vector<float> channel_std = {12.32F, 11.47F, 6.91F, 0.86F, 0.16F};
  uint32_t max_points = 150000U;
};

struct ProjectedPoint {
  bool valid = false;
  uint32_t x = 0U;
  uint32_t y = 0U;
  float range = 0.0F;
};

struct RangeImage {
  uint32_t width = 0U;
  uint32_t height = 0U;
  std::vector<float> input_chw;
  std::vector<float> projected_range;
  std::vector<int32_t> pixel_to_point_index;
  std::vector<ProjectedPoint> points;

  std::size_t PixelCount() const {
    return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  }
};

struct SemanticPointPrediction {
  uint32_t label = 0U;
  float confidence = 0.0F;
};

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
