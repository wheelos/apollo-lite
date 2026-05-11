/******************************************************************************
 * Copyright 2026 The WheelOS Team. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/drivers/camera_gst/frame_stitcher.h"

#include <set>
#include <unordered_map>

#include "cyber/cyber.h"
#include "opencv2/imgproc.hpp"

namespace apollo {
namespace drivers {
namespace camera_gst {

GridFrameStitcher::GridFrameStitcher(const config::Config& config)
    : config_(config) {
  slots_.reserve(static_cast<size_t>(config_.layout_slots_size()));
  for (const auto& slot : config_.layout_slots()) {
    slots_.push_back(
        LayoutSlot{slot.source_name(), static_cast<int>(slot.row()),
                   static_cast<int>(slot.col())});
  }
  valid_ = ValidateConfig();
}

int GridFrameStitcher::output_width() const {
  return static_cast<int>(config_.cols() * config_.tile_width());
}

int GridFrameStitcher::output_height() const {
  return static_cast<int>(config_.rows() * config_.tile_height());
}

bool GridFrameStitcher::Stitch(const std::vector<CapturedFrame>& frames,
                               cv::Mat* stitched_rgb) const {
  if (!valid_ || stitched_rgb == nullptr) {
    return false;
  }

  std::unordered_map<std::string, const CapturedFrame*> frame_index;
  for (const auto& frame : frames) {
    frame_index.emplace(frame.source_name, &frame);
  }

  stitched_rgb->create(output_height(), output_width(), CV_8UC3);
  stitched_rgb->setTo(cv::Scalar::all(0));
  for (const auto& slot : slots_) {
    auto iter = frame_index.find(slot.source_name);
    if (iter == frame_index.end() || iter->second->image_rgb.empty()) {
      AERROR << "Missing frame for stitch slot source " << slot.source_name;
      return false;
    }

    cv::Mat tile;
    if (iter->second->image_rgb.cols != static_cast<int>(config_.tile_width()) ||
        iter->second->image_rgb.rows !=
            static_cast<int>(config_.tile_height())) {
      cv::resize(iter->second->image_rgb, tile,
                 cv::Size(config_.tile_width(), config_.tile_height()));
    } else {
      tile = iter->second->image_rgb;
    }

    cv::Rect roi(slot.col * static_cast<int>(config_.tile_width()),
                 slot.row * static_cast<int>(config_.tile_height()),
                 static_cast<int>(config_.tile_width()),
                 static_cast<int>(config_.tile_height()));
    tile.copyTo((*stitched_rgb)(roi));
  }
  return true;
}

bool GridFrameStitcher::ValidateConfig() const {
  if (config_.rows() == 0 || config_.cols() == 0 || config_.tile_width() == 0 ||
      config_.tile_height() == 0) {
    AERROR << "camera_gst rows/cols/tile sizes must be positive.";
    return false;
  }
  if (config_.sources_size() == 0 || config_.layout_slots_size() == 0) {
    AERROR << "camera_gst requires sources and layout slots.";
    return false;
  }
  if (config_.layout_slots_size() != config_.sources_size()) {
    AERROR << "camera_gst requires one layout slot per source.";
    return false;
  }

  std::set<std::string> source_names;
  for (const auto& source : config_.sources()) {
    if (source.name().empty()) {
      AERROR << "camera_gst source name must not be empty.";
      return false;
    }
    if (!source_names.insert(source.name()).second) {
      AERROR << "Duplicate camera_gst source name: " << source.name();
      return false;
    }
  }

  std::set<std::string> slot_sources;
  std::set<std::pair<int, int>> occupied_cells;
  for (const auto& slot : slots_) {
    if (!source_names.count(slot.source_name)) {
      AERROR << "Layout references unknown source " << slot.source_name;
      return false;
    }
    if (!slot_sources.insert(slot.source_name).second) {
      AERROR << "Duplicate layout slot for source " << slot.source_name;
      return false;
    }
    if (slot.row < 0 || slot.row >= static_cast<int>(config_.rows()) ||
        slot.col < 0 || slot.col >= static_cast<int>(config_.cols())) {
      AERROR << "Layout slot out of range for source " << slot.source_name;
      return false;
    }
    if (!occupied_cells.insert(std::make_pair(slot.row, slot.col)).second) {
      AERROR << "Multiple sources mapped to layout cell (" << slot.row << ", "
             << slot.col << ")";
      return false;
    }
  }
  return true;
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
