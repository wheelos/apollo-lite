/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/ndt_localization/map_support/ndt_map/ndt_map_matrix.h"

#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

TEST(MapNdtTestSuite, CellBinaryRoundTrip) {
  NdtMapCells cells;
  const int altitude_index =
      cells.AddSample(42.0f, 3.0f, 1.0f, Eigen::Vector3f(0.25f, 0.5f, 0.75f),
                      true);
  std::vector<unsigned char> binary(cells.GetBinarySize());
  EXPECT_EQ(cells.CreateBinary(binary.data(), binary.size()), binary.size());

  NdtMapCells restored;
  EXPECT_EQ(restored.LoadBinary(binary.data()), binary.size());
  ASSERT_EQ(restored.cells_.count(altitude_index), 1);
  EXPECT_EQ(restored.cells_.at(altitude_index).count_, 1);
  EXPECT_FLOAT_EQ(restored.cells_.at(altitude_index).intensity_, 42.0f);
  EXPECT_EQ(restored.road_cell_indices_.size(), 1);
  EXPECT_EQ(restored.road_cell_indices_[0], altitude_index);
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
