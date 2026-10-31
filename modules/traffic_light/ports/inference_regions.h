// Copyright 2026 The Apollo Authors. All Rights Reserved.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "modules/traffic_light/common/context.h"

namespace apollo {
namespace traffic_light {

struct InferenceRegion {
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
};

struct LetterboxTransform {
  float resize_scale = 1.0f;
  float pad_x = 0.0f;
  float pad_y = 0.0f;
  InferenceRegion region;
};

inline std::vector<InferenceRegion> BuildInferenceRegions(
    const PipelineContext& context, const CameraFrameState& frame, int limit) {
  std::vector<InferenceRegion> regions;
  if (limit <= 0 || frame.image.cols <= 0 || frame.image.rows <= 0) {
    return regions;
  }
  const bool explore =
      std::any_of(context.prompts.begin(), context.prompts.end(),
                  [&](const PromptRegion& prompt) {
                    return prompt.source == PromptSource::FULL_FRAME &&
                           (prompt.camera_name.empty() ||
                            prompt.camera_name == frame.camera_name);
                  });
  const int crop_limit = limit - (explore ? 1 : 0);
  for (const auto& prompt : context.prompts) {
    if (static_cast<int>(regions.size()) >= crop_limit) {
      break;
    }
    if (prompt.source == PromptSource::FULL_FRAME ||
        (!prompt.camera_name.empty() &&
         prompt.camera_name != frame.camera_name) ||
        !std::isfinite(prompt.roi_box.x) || !std::isfinite(prompt.roi_box.y) ||
        !std::isfinite(prompt.roi_box.width) ||
        !std::isfinite(prompt.roi_box.height) || !prompt.roi_box.IsValid()) {
      continue;
    }
    const float x1 = std::max(
        0.0f, std::min(prompt.roi_box.x, static_cast<float>(frame.image.cols)));
    const float y1 = std::max(
        0.0f, std::min(prompt.roi_box.y, static_cast<float>(frame.image.rows)));
    const float x2 =
        std::max(0.0f, std::min(prompt.roi_box.x + prompt.roi_box.width,
                                static_cast<float>(frame.image.cols)));
    const float y2 =
        std::max(0.0f, std::min(prompt.roi_box.y + prompt.roi_box.height,
                                static_cast<float>(frame.image.rows)));
    InferenceRegion region;
    region.x = static_cast<int>(std::floor(x1));
    region.y = static_cast<int>(std::floor(y1));
    region.width = static_cast<int>(std::ceil(x2)) - region.x;
    region.height = static_cast<int>(std::ceil(y2)) - region.y;
    if (region.width <= 0 || region.height <= 0) {
      continue;
    }
    const bool duplicate = std::any_of(
        regions.begin(), regions.end(), [&](const InferenceRegion& existing) {
          return existing.x == region.x && existing.y == region.y &&
                 existing.width == region.width &&
                 existing.height == region.height;
        });
    if (!duplicate) {
      regions.push_back(region);
    }
  }
  if (explore || regions.empty()) {
    regions.push_back({0, 0, frame.image.cols, frame.image.rows});
  }
  return regions;
}

inline Rect2f UndoLetterbox(float center_x, float center_y, float width,
                            float height, const LetterboxTransform& transform) {
  if (!std::isfinite(transform.resize_scale) ||
      transform.resize_scale <= 0.0f || !std::isfinite(center_x) ||
      !std::isfinite(center_y) || !std::isfinite(width) ||
      !std::isfinite(height) || width <= 0.0f || height <= 0.0f) {
    return {};
  }
  const float x1 = std::max(0.0f, (center_x - 0.5f * width - transform.pad_x) /
                                      transform.resize_scale);
  const float y1 = std::max(0.0f, (center_y - 0.5f * height - transform.pad_y) /
                                      transform.resize_scale);
  const float x2 = std::min(
      static_cast<float>(transform.region.width),
      (center_x + 0.5f * width - transform.pad_x) / transform.resize_scale);
  const float y2 = std::min(
      static_cast<float>(transform.region.height),
      (center_y + 0.5f * height - transform.pad_y) / transform.resize_scale);
  return {x1 + transform.region.x, y1 + transform.region.y,
          std::max(0.0f, x2 - x1), std::max(0.0f, y2 - y1)};
}

}  // namespace traffic_light
}  // namespace apollo
