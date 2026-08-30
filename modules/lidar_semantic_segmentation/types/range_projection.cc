#include "modules/lidar_semantic_segmentation/types/range_projection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace apollo {
namespace lidar_semantic_segmentation {

namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kMinimumRangeMeters = 1.0e-3F;

void SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

std::size_t ChannelOffset(std::size_t channel, std::size_t pixel_count) {
  return channel * pixel_count;
}

}  // namespace

bool RangeImageProjector::Init(const RangeImageProjectionOptions& options,
                               std::string* error) {
  if (options.width == 0U || options.height == 0U) {
    SetError("range image width and height must be positive", error);
    return false;
  }
  if (options.fov_up_degrees <= options.fov_down_degrees) {
    SetError("fov_up_degrees must be larger than fov_down_degrees", error);
    return false;
  }
  if (options.channel_mean.size() != kRangeRetInputChannels ||
      options.channel_std.size() != kRangeRetInputChannels) {
    SetError("RangeRet requires five channel means/stds", error);
    return false;
  }
  for (const float std_value : options.channel_std) {
    if (!std::isfinite(std_value) || std::fabs(std_value) <=
                                          std::numeric_limits<float>::epsilon()) {
      SetError("channel std values must be finite and non-zero", error);
      return false;
    }
  }
  if (!std::isfinite(options.intensity_scale) ||
      options.intensity_scale <= 0.0F) {
    SetError("intensity_scale must be finite and positive", error);
    return false;
  }
  options_ = options;
  initialized_ = true;
  return true;
}

bool RangeImageProjector::Project(const apollo::drivers::PointCloud& cloud,
                                  RangeImage* image,
                                  std::string* error) const {
  if (!initialized_) {
    SetError("range image projector is not initialized", error);
    return false;
  }
  if (image == nullptr) {
    SetError("range image output is null", error);
    return false;
  }
  if (cloud.point_size() > static_cast<int>(options_.max_points)) {
    SetError("point cloud exceeds configured max_points", error);
    return false;
  }

  image->width = options_.width;
  image->height = options_.height;
  const std::size_t pixel_count = image->PixelCount();
  image->input_chw.assign(kRangeRetInputChannels * pixel_count, 0.0F);
  image->projected_range.assign(pixel_count, -1.0F);
  image->pixel_to_point_index.assign(pixel_count, -1);
  image->points.assign(static_cast<std::size_t>(cloud.point_size()),
                       ProjectedPoint());

  const float fov_up = options_.fov_up_degrees * kPi / 180.0F;
  const float fov_down = options_.fov_down_degrees * kPi / 180.0F;
  const float fov = std::fabs(fov_down) + std::fabs(fov_up);
  std::vector<int> order(static_cast<std::size_t>(cloud.point_size()));
  std::iota(order.begin(), order.end(), 0);
  std::vector<float> ranges(order.size(), 0.0F);

  for (int index = 0; index < cloud.point_size(); ++index) {
    const auto& point = cloud.point(index);
    const float x = point.x();
    const float y = point.y();
    const float z = point.z();
    const float range = std::sqrt(x * x + y * y + z * z);
    ranges[static_cast<std::size_t>(index)] = range;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(range) || range <= kMinimumRangeMeters) {
      continue;
    }
    const float yaw = -std::atan2(y, x);
    const float pitch = std::asin(std::max(-1.0F, std::min(1.0F, z / range)));
    float proj_x = 0.5F * (yaw / kPi + 1.0F);
    float proj_y = 1.0F - (pitch + std::fabs(fov_down)) / fov;
    proj_x *= static_cast<float>(options_.width);
    proj_y *= static_cast<float>(options_.height);
    const int pixel_x = std::max(
        0, std::min(static_cast<int>(options_.width) - 1,
                    static_cast<int>(std::floor(proj_x))));
    const int pixel_y = std::max(
        0, std::min(static_cast<int>(options_.height) - 1,
                    static_cast<int>(std::floor(proj_y))));
    ProjectedPoint* projected = &image->points[static_cast<std::size_t>(index)];
    projected->valid = true;
    projected->x = static_cast<uint32_t>(pixel_x);
    projected->y = static_cast<uint32_t>(pixel_y);
    projected->range = range;
  }

  std::sort(order.begin(), order.end(), [&ranges](int left, int right) {
    return ranges[static_cast<std::size_t>(left)] >
           ranges[static_cast<std::size_t>(right)];
  });

  for (const int point_index : order) {
    const ProjectedPoint& projected =
        image->points[static_cast<std::size_t>(point_index)];
    if (!projected.valid) {
      continue;
    }
    const auto& point = cloud.point(point_index);
    const std::size_t pixel =
        static_cast<std::size_t>(projected.y) * options_.width + projected.x;
    image->projected_range[pixel] = projected.range;
    image->pixel_to_point_index[pixel] = point_index;
    const float raw[kRangeRetInputChannels] = {
        projected.range, point.x(), point.y(), point.z(),
        static_cast<float>(point.intensity()) * options_.intensity_scale};
    for (std::size_t channel = 0; channel < kRangeRetInputChannels; ++channel) {
      image->input_chw[ChannelOffset(channel, pixel_count) + pixel] =
          (raw[channel] - options_.channel_mean[channel]) /
          options_.channel_std[channel];
    }
  }
  return true;
}

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
