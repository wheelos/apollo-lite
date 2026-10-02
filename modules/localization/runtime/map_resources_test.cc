// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/runtime/map_resources.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

GlobalEstimatorConfig Config() {
  GlobalEstimatorConfig config;
  auto* map = config.mutable_map();
  map->set_map_id("site");
  map->set_version("v1");
  map->set_frame_id("map");
  map->set_calibration_id("calibration");
  map->set_pcd_path("");
  config.set_max_position_std(2.0);
  config.set_max_attitude_std(0.2);
  return config;
}

MapSwitchRequest Request() {
  MapSwitchRequest request;
  request.mutable_current_epoch()->set_producer_session("local");
  request.mutable_current_epoch()->set_generation(7);
  request.set_request_id(1);
  request.set_request_time(10.0);
  request.set_expected_map_id("site");
  request.set_expected_map_version("v1");
  *request.mutable_next_map() = Config().map();
  request.mutable_next_map()->set_version("v2");
  return request;
}

TEST(MapResourcesTest, ValidRequestDoesNotMutateCurrentMapOrEpoch) {
  const auto config = Config();
  const Epoch epoch{"local", 7};
  ASSERT_TRUE(ValidateMapSwitch(Request(), config, epoch, 0, 10.0).ok());
  EXPECT_EQ("v1", config.map().version());
  EXPECT_EQ(7U, epoch.generation);
}

TEST(MapResourcesTest, RejectsStaleReplayEpochAndMapVersion) {
  const auto config = Config();
  const Epoch epoch{"local", 7};
  EXPECT_EQ(Reason::INVALID_INPUT,
            ValidateMapSwitch(Request(), config, epoch, 0, 20.0).reason);
  EXPECT_EQ(Reason::DUPLICATE,
            ValidateMapSwitch(Request(), config, epoch, 1, 10.0).reason);
  EXPECT_EQ(Reason::EPOCH_MISMATCH,
            ValidateMapSwitch(Request(), config, {"local", 8}, 0, 10.0).reason);
  auto request = Request();
  request.set_expected_map_version("v0");
  EXPECT_EQ(Reason::MAP_MISMATCH,
            ValidateMapSwitch(request, config, epoch, 0, 10.0).reason);
}

TEST(MapResourcesTest, FrameAndCalibrationCannotBeSilentlyReplaced) {
  const auto config = Config();
  auto request = Request();
  request.mutable_next_map()->set_frame_id("building");
  EXPECT_EQ(Reason::MAP_MISMATCH,
            ValidateMapSwitch(request, config, {"local", 7}, 0, 10.0).reason);
  request = Request();
  request.mutable_next_map()->set_calibration_id("unverified");
  EXPECT_EQ(Reason::MAP_MISMATCH,
            ValidateMapSwitch(request, config, {"local", 7}, 0, 10.0).reason);
  request = Request();
  request.mutable_next_map()->set_version("v1");
  EXPECT_EQ(Reason::MAP_MISMATCH,
            ValidateMapSwitch(request, config, {"local", 7}, 0, 10.0).reason);
}

TEST(MapResourcesTest, DisabledMatchingLoadsNoFileAndLeavesNoFakeMap) {
  std::shared_ptr<MapMatcher> matcher;
  std::vector<Eigen::Isometry3d> seeds{Eigen::Isometry3d::Identity()};
  ASSERT_TRUE(LoadGlobalMap(Config(), &matcher, &seeds).ok());
  EXPECT_EQ(nullptr, matcher);
  EXPECT_TRUE(seeds.empty());
  auto config = Config();
  config.set_enable_map_matching(true);
  EXPECT_EQ(Reason::MAP_MISMATCH,
            ValidateMapSwitch(Request(), config, {"local", 7}, 0, 10.0).reason);
  EXPECT_EQ(Reason::CONFIG_INVALID,
            LoadGlobalMap(config, &matcher, &seeds).reason);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
