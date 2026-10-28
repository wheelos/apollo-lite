// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "modules/lidar_fusion/proto/lidar_fusion.pb.h"

namespace apollo {
namespace lidar_fusion {

using SpaceKey = std::pair<int, int>;

// One frame's touched cells. Dense slot IDs accelerate lookup without creating
// a protobuf cell for every location in the ROI.
class SpaceCells {
 public:
  using Storage = std::vector<std::pair<SpaceKey, SpaceCell>>;
  explicit SpaceCells(const SpaceMap& map);
  SpaceCell& operator[](const SpaceKey& key);
  Storage::iterator find(const SpaceKey& key);
  Storage::iterator begin() { return cells_.begin(); }
  Storage::iterator end() { return cells_.end(); }
  void Publish(SpaceMap* map);

 private:
  int* Slot(const SpaceKey& key);
  int width_ = 0;
  int height_ = 0;
  std::vector<int> slots_;
  std::map<SpaceKey, int> overflow_;
  Storage cells_;
};

bool ValidateSpatial(const FusionConfig& config, std::string* error);
void EnrichSpatial(const FusionConfig& config, FusionOutput* output);
void CheckOutput(const FusionConfig& config, FusionOutput* output);
void EnrichSpatial(const FusionConfig& config, FusionOutput* output,
                   SpaceCells* cells);
void CheckOutput(const FusionConfig& config, FusionOutput* output,
                 SpaceCells* cells);

}  // namespace lidar_fusion
}  // namespace apollo
