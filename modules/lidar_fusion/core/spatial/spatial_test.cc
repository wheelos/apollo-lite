// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/lidar_fusion/core/spatial/spatial.h"

#include <string>
#include <utility>

#include "gtest/gtest.h"

namespace apollo {
namespace lidar_fusion {
namespace {

FusionOutput Map() {
  FusionOutput out;
  auto* map = out.mutable_space_map();
  map->set_origin_x(0);
  map->set_origin_y(0);
  map->set_resolution(1);
  map->set_width(8);
  map->set_height(8);
  return out;
}

TEST(SpatialTest, RequiresVisibilityAndExplicitVehicleLimits) {
  FusionConfig c;
  c.set_resolution(1);
  auto* o = c.mutable_spatial();
  o->set_traversability(true);
  std::string error;
  EXPECT_FALSE(ValidateSpatial(c, &error));
  o->set_trace_rays(true);
  o->set_verified_single_origin(true);
  o->set_vehicle_length(0.1);
  o->set_vehicle_width(0.1);
  o->set_vehicle_height(1);
  o->set_safety_margin(0);
  o->set_max_slope(0.1);
  o->set_max_step(0.1);
  o->set_max_roughness(0.05);
  o->set_ground_clearance(0.2);
  EXPECT_TRUE(ValidateSpatial(c, &error)) << error;
  o->set_verified_single_origin(false);
  EXPECT_FALSE(ValidateSpatial(c, &error));
}

TEST(SpatialTest, UnknownViewPreservesPointOrderAndSparseMembership) {
  auto out = Map();
  for (uint32_t id : {2U, 999U, 1U, 0U}) {
    auto* p = out.add_point_evidence();
    p->set_point_id(id);
    p->set_finite(true);
    out.add_point_decision()->set_point_id(id);
  }
  auto* unknown = out.add_unknown();
  for (uint32_t id : {1U, 999U, 1U}) {
    unknown->add_point_id(id);
  }
  FusionConfig c;
  c.mutable_spatial()->set_point_views(true);
  EnrichSpatial(c, &out);
  const auto& view = out.point_view(UNKNOWN_POINTS);
  ASSERT_EQ(view.point_id_size(), 2);
  EXPECT_EQ(view.point_id(0), 999U);
  EXPECT_EQ(view.point_id(1), 1U);
}

TEST(SpatialTest, RayStopsAtOccluderAndApproximationNeverEnablesFree) {
  auto out = Map();
  auto* wall = out.mutable_space_map()->add_cell();
  wall->set_x_index(3);
  wall->set_y_index(0);
  wall->set_occupancy_obs(true);
  auto* p = out.add_point_evidence();
  p->set_finite(true);
  p->mutable_position()->set_x(7.5);
  p->mutable_position()->set_y(0.5);
  p->mutable_position()->set_z(1);
  FusionConfig c;
  c.mutable_sensor_to_base()->set_x(0.5);
  c.mutable_sensor_to_base()->set_y(0.5);
  c.mutable_sensor_to_base()->set_z(1);
  c.mutable_spatial()->set_trace_rays(true);
  EnrichSpatial(c, &out);
  bool observed = false;
  for (const auto& cell : out.space_map().cell()) {
    EXPECT_LE(cell.x_index(), 3U);
    EXPECT_NE(cell.traversability(), FREE);
    if (cell.visibility_mask() != 0) {
      observed = true;
      EXPECT_TRUE(cell.visibility_approximate());
    }
  }
  EXPECT_TRUE(observed);
}

TEST(SpatialTest, TraversabilityRequiresAllBandsGroundAndFootprint) {
  auto out = Map();
  FusionConfig c;
  c.set_resolution(1);
  auto* o = c.mutable_spatial();
  o->set_traversability(true);
  o->set_verified_single_origin(true);
  o->set_height_bands(2);
  o->set_vehicle_length(0.1);
  o->set_vehicle_width(0.1);
  o->set_max_slope(0.2);
  o->set_max_step(0.1);
  o->set_max_roughness(0.1);
  o->set_seed_x(1.5);
  o->set_seed_y(1.5);
  for (int x = 1; x <= 4; ++x) {
    auto* cell = out.mutable_space_map()->add_cell();
    cell->set_x_index(x);
    cell->set_y_index(1);
    cell->set_ground_supported(true);
    cell->set_visibility_mask(3);
    cell->set_semantic(SURFACE);
  }
  out.mutable_space_map()->mutable_cell(1)->set_visibility_mask(1);
  out.mutable_space_map()->mutable_cell(2)->set_ground_supported(false);
  EnrichSpatial(c, &out);
  EXPECT_EQ(out.space_map().cell(0).traversability(), FREE);
  EXPECT_TRUE(out.space_map().cell(0).reachable());
  EXPECT_EQ(out.space_map().cell(1).traversability(), UNKNOWN);
  EXPECT_EQ(out.space_map().cell(2).traversability(), UNKNOWN);
  EXPECT_EQ(out.space_map().cell(3).traversability(), FREE);
  EXPECT_FALSE(out.space_map().cell(3).reachable());
  o->set_vehicle_width(2);
  o->set_vehicle_length(2);
  EnrichSpatial(c, &out);
  for (const auto& cell : out.space_map().cell()) {
    EXPECT_EQ(cell.traversability(), UNKNOWN);
  }
}

TEST(SpatialTest, RepairsConflictOccupancyAndDanglingReferences) {
  auto out = Map();
  auto* object = out.add_object();
  object->set_id(1);
  object->set_consistency(CONFLICT);
  object->mutable_box()->mutable_center()->set_x(2);
  object->mutable_box()->mutable_center()->set_y(2);
  object->mutable_box()->mutable_size()->set_x(1);
  object->mutable_box()->mutable_size()->set_y(1);
  auto* cell = out.mutable_space_map()->add_cell();
  cell->set_x_index(2);
  cell->set_y_index(2);
  cell->set_traversability(FREE);
  cell->set_reachable(true);
  cell->add_object_ref(999);
  auto* unknown = out.add_unknown();
  unknown->add_related_object_id(999);
  unknown->add_relation()->set_object_id(999);
  FusionConfig c;
  CheckOutput(c, &out);
  EXPECT_TRUE(out.object(0).class_conflict());
  EXPECT_EQ(out.unknown(0).related_object_id_size(), 0);
  EXPECT_EQ(out.unknown(0).relation_size(), 0);
  EXPECT_GT(out.health().consistency_repairs(), 0U);
  for (const auto& repaired : out.space_map().cell()) {
    EXPECT_TRUE(repaired.occupancy_det());
    EXPECT_EQ(repaired.traversability(), OCCUPIED);
    EXPECT_FALSE(repaired.reachable());
    ASSERT_EQ(repaired.object_ref_size(), 1);
    EXPECT_EQ(repaired.object_ref(0), 1U);
  }
  const auto repairs = out.health().consistency_repairs();
  CheckOutput(c, &out);
  EXPECT_EQ(out.health().consistency_repairs(), repairs);
}

TEST(SpatialTest, UnknownFootprintCannotContainFreeAndDoesNotInventDetections) {
  auto out = Map();
  auto* unknown = out.add_unknown();
  for (const auto& xy :
       {std::make_pair(1., 1.), {3., 1.}, {3., 3.}, {1., 3.}}) {
    auto* p = unknown->add_polygon();
    p->set_x(xy.first);
    p->set_y(xy.second);
  }
  auto* free = out.mutable_space_map()->add_cell();
  free->set_x_index(2);
  free->set_y_index(2);
  free->set_traversability(FREE);
  free->set_reachable(true);
  const auto original = unknown->SerializeAsString();
  FusionConfig c;
  CheckOutput(c, &out);
  EXPECT_EQ(out.unknown(0).SerializeAsString(), original);
  for (const auto& cell : out.space_map().cell()) {
    EXPECT_TRUE(cell.occupancy_unknown());
    EXPECT_FALSE(cell.occupancy_det());
    EXPECT_EQ(cell.traversability(), OCCUPIED);
    EXPECT_FALSE(cell.reachable());
  }
}

TEST(SpatialTest, GroundAndRoadAloneCannotJustifyFree) {
  auto out = Map();
  auto* cell = out.mutable_space_map()->add_cell();
  cell->set_x_index(2);
  cell->set_y_index(2);
  cell->set_ground_supported(true);
  cell->set_semantic(SURFACE);
  cell->set_physically_traversable(true);
  cell->set_traversability(FREE);
  FusionConfig c;
  c.mutable_spatial()->set_traversability(true);
  c.mutable_spatial()->set_verified_single_origin(true);
  CheckOutput(c, &out);
  EXPECT_EQ(out.space_map().cell(0).traversability(), UNKNOWN);
  EXPECT_FALSE(out.space_map().cell(0).reachable());
}

TEST(SpatialTest, PointSupportedUnknownDoesNotFillUnobservedExtent) {
  auto out = Map();
  auto* unknown = out.add_unknown();
  unknown->set_point_supported_footprint(true);
  for (double x : {1.5, 6.5}) {
    const int id = out.point_evidence_size();
    auto* p = out.add_point_evidence();
    p->set_finite(true);
    p->set_point_id(id);
    p->mutable_position()->set_x(x);
    p->mutable_position()->set_y(1.5);
    unknown->add_point_id(id);
    auto* v = unknown->add_polygon();
    v->set_x(x);
    v->set_y(1.5);
  }
  auto* gap = out.mutable_space_map()->add_cell();
  gap->set_x_index(3);
  gap->set_y_index(1);
  gap->set_traversability(UNKNOWN);
  FusionConfig c;
  EnrichSpatial(c, &out);
  CheckOutput(c, &out);
  EXPECT_EQ(out.health().consistency_repairs(), 0);
  ASSERT_EQ(out.space_map().cell_size(), 3);
  for (const auto& cell : out.space_map().cell()) {
    if (cell.x_index() == 3) {
      EXPECT_FALSE(cell.occupancy_unknown());
      EXPECT_EQ(cell.traversability(), UNKNOWN);
    } else {
      EXPECT_TRUE(cell.occupancy_unknown());
      EXPECT_EQ(cell.traversability(), OCCUPIED);
    }
  }
}

TEST(SpatialTest, RepairsUnknownRiskOverExistingOccupancyWithoutFillingGaps) {
  auto out = Map();
  auto* unknown = out.add_unknown();
  unknown->set_point_supported_footprint(true);
  for (double x : {1.5, 6.5}) {
    const int id = out.point_evidence_size();
    auto* p = out.add_point_evidence();
    p->set_finite(true);
    p->set_point_id(id);
    p->mutable_position()->set_x(x);
    p->mutable_position()->set_y(1.5);
    unknown->add_point_id(id);
    auto* v = unknown->add_polygon();
    v->set_x(x);
    v->set_y(1.5);
    auto* cell = out.mutable_space_map()->add_cell();
    cell->set_x_index(id == 0 ? 1 : 6);
    cell->set_y_index(1);
    cell->set_occupancy_obs(id == 0);
    cell->set_occupancy_det(id != 0);
  }
  auto* gap = out.mutable_space_map()->add_cell();
  gap->set_x_index(3);
  gap->set_y_index(1);
  gap->set_traversability(UNKNOWN);
  gap->set_reachable(false);
  const auto original_gap = gap->SerializeAsString();
  FusionConfig c;
  EnrichSpatial(c, &out);
  for (auto& cell : *out.mutable_space_map()->mutable_cell()) {
    if (cell.x_index() != 3) {
      ASSERT_TRUE(cell.occupancy_unknown());
      cell.set_occupancy_unknown(false);
    }
  }

  CheckOutput(c, &out);
  EXPECT_EQ(out.health().consistency_repairs(), 2U);
  ASSERT_EQ(out.risk_event_size(), 1);
  EXPECT_EQ(out.risk_event(0), OUTPUT_REPAIRED);
  ASSERT_EQ(out.space_map().cell_size(), 3);
  for (const auto& cell : out.space_map().cell()) {
    if (cell.x_index() == 3) {
      EXPECT_EQ(cell.SerializeAsString(), original_gap);
    } else {
      EXPECT_EQ(cell.occupancy_obs(), cell.x_index() == 1);
      EXPECT_EQ(cell.occupancy_det(), cell.x_index() == 6);
      EXPECT_TRUE(cell.occupancy_unknown());
      EXPECT_EQ(cell.traversability(), OCCUPIED);
      EXPECT_FALSE(cell.reachable());
      EXPECT_FALSE(cell.physically_traversable());
      ASSERT_EQ(cell.reason_size(), 1);
      EXPECT_EQ(cell.reason(0), OUTPUT_REPAIRED);
    }
    EXPECT_NE(cell.traversability(), FREE);
    EXPECT_NE(cell.traversability(), FREE_SOFT);
  }
  const auto repaired = out.SerializeAsString();
  CheckOutput(c, &out);
  EXPECT_EQ(out.SerializeAsString(), repaired);
}

TEST(SpatialTest, SharedCellsMatchIndependentChecksAcrossGrowthAndRepairs) {
  auto out = Map();
  out.mutable_space_map()->set_width(32);
  out.mutable_space_map()->set_height(32);
  auto* unknown = out.add_unknown();
  unknown->set_point_supported_footprint(true);
  for (int x = 1; x < 30; ++x) {
    for (int y = 1; y < 30; ++y) {
      auto* point = out.add_point_evidence();
      point->set_finite(true);
      point->mutable_position()->set_x(x + 0.5);
      point->mutable_position()->set_y(y + 0.5);
      point->mutable_position()->set_z(1);
      unknown->add_point_id(out.point_evidence_size() - 1);
    }
  }
  auto* object = out.add_object();
  object->set_id(1);
  object->set_consistency(CONFLICT);
  object->mutable_box()->mutable_center()->set_x(2);
  object->mutable_box()->mutable_center()->set_y(2);
  object->mutable_box()->mutable_size()->set_x(1);
  object->mutable_box()->mutable_size()->set_y(1);
  auto* bad = out.mutable_space_map()->add_cell();
  bad->set_x_index(31);
  bad->set_y_index(31);
  bad->add_object_ref(999);
  bad->set_traversability(FREE);
  auto expected = out;
  FusionConfig c;
  c.mutable_spatial()->set_trace_rays(true);
  EnrichSpatial(c, &expected);
  CheckOutput(c, &expected);
  SpaceCells cells(out.space_map());
  EnrichSpatial(c, &out, &cells);
  CheckOutput(c, &out, &cells);
  cells.Publish(out.mutable_space_map());
  EXPECT_EQ(out.SerializeAsString(), expected.SerializeAsString());
  EXPECT_GT(out.health().consistency_repairs(), 0);
  EXPECT_GE(out.space_map().cell_size(), 841);
}

}  // namespace
}  // namespace lidar_fusion
}  // namespace apollo
