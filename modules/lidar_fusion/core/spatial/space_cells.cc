// Copyright 2026 WheelOS. All Rights Reserved.

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <utility>
#include <vector>

#include "modules/lidar_fusion/core/spatial/spatial.h"

namespace apollo {
namespace lidar_fusion {

SpaceCells::SpaceCells(const SpaceMap& map) {
  const uint64_t count = uint64_t{map.width()} * map.height();
  if (count > 0 && count <= 1000000) {
    width_ = map.width();
    height_ = map.height();
    slots_.assign(count, -1);
  }
  cells_.reserve(map.cell_size());
  for (const auto& cell : map.cell()) {
    (*this)[{cell.x_index(), cell.y_index()}] = cell;
  }
}

int* SpaceCells::Slot(const SpaceKey& key) {
  if (key.first >= 0 && key.first < width_ && key.second >= 0 &&
      key.second < height_) {
    return &slots_[key.first * height_ + key.second];
  }
  const auto found = overflow_.find(key);
  return found == overflow_.end() ? nullptr : &found->second;
}

SpaceCell& SpaceCells::operator[](const SpaceKey& key) {
  int* slot = Slot(key);
  if (slot != nullptr && *slot >= 0) {
    return cells_[*slot].second;
  }
  const int index = cells_.size();
  if (slot != nullptr) {
    *slot = index;
  } else {
    overflow_.emplace(key, index);
  }
  cells_.emplace_back(key, SpaceCell{});
  return cells_.back().second;
}

SpaceCells::Storage::iterator SpaceCells::find(const SpaceKey& key) {
  const int* slot = Slot(key);
  return slot == nullptr || *slot < 0 ? end() : begin() + *slot;
}

void SpaceCells::Publish(SpaceMap* map) {
  std::vector<std::size_t> order(cells_.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return cells_[a].first < cells_[b].first;
  });
  map->clear_cell();
  for (std::size_t id : order) {
    auto& entry = cells_[id];
    entry.second.set_x_index(entry.first.first);
    entry.second.set_y_index(entry.first.second);
    *map->add_cell() = std::move(entry.second);
  }
}

}  // namespace lidar_fusion
}  // namespace apollo
