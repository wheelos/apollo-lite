// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "modules/lidar_fusion/core/runtime/single_frame_fusion.h"

namespace apollo {
namespace lidar_fusion {
namespace testing_support {

inline FusionConfig Config() {
  FusionConfig c;
  c.set_pointcloud_channel("/test/cloud");
  c.set_detection_channel("/test/det");
  c.set_segmentation_channel("/test/seg");
  c.set_output_channel("/test/fusion");
  c.set_sensor_frame("lidar");
  c.set_detection_backend(
      lidar_object_detection::LIDAR_OBJECT_DETECTION_BACKEND_CENTERPOINT);
  auto* t = c.mutable_sensor_to_base();
  t->set_version("fixture-calibration");
  t->set_x(0);
  t->set_y(0);
  t->set_z(0);
  t->set_roll(0);
  t->set_pitch(0);
  t->set_yaw(0);
  auto* det = c.add_detection_mapping();
  det->set_raw_id(0);
  det->set_group(THING);
  det->set_unified_class(CAR);
  for (const auto& entry : {std::make_pair(0U, GENERIC_OBS),
                            {1U, THING},
                            {9U, SURFACE},
                            {13U, STATIC_STRUCT},
                            {15U, VEGETATION},
                            {20U, NOISE}}) {
    auto* mapping = c.add_semantic_mapping();
    mapping->set_raw_id(entry.first);
    mapping->set_group(entry.second);
    if (entry.second == THING) {
      mapping->set_unified_class(CAR);
    }
  }
  c.set_connectivity_size(0.2);
  c.set_min_x(0);
  c.set_max_x(8);
  c.set_min_y(-4);
  c.set_max_y(4);
  return c;
}

class FusionTest : public testing::Test {
 protected:
  void SetUp() override {
    config = Config();
    cloud.set_frame_id("lidar");
    cloud.set_measurement_time(100);
    cloud.mutable_header()->set_frame_id("lidar");
    cloud.mutable_header()->set_sequence_num(7);
    cloud.mutable_header()->set_lidar_timestamp(100000000000ULL);
    cloud.mutable_header()->set_timestamp_sec(100.01);
    // Independent, flat, noncollinear ground samples in one fitting tile.
    for (double y : {0.2, 0.8}) {
      for (double x : {1.0, 1.6, 2.2}) {
        Add(x, y, 0, 9);
      }
    }
    Sync();
  }

  void Add(double x, double y, double z, uint32_t label) {
    auto* p = cloud.add_point();
    p->set_x(x);
    p->set_y(y);
    p->set_z(z);
    auto* s = seg.add_point_label();
    s->set_point_index(cloud.point_size() - 1);
    s->set_semantic_label(label);
    s->set_confidence(0.95F);
  }

  void Obstacle(uint32_t label = 1) {
    for (double y : {1.3, 1.4}) {
      for (double x : {1.9, 2.0, 2.1}) {
        Add(x, y, 0.8, label);
      }
    }
    Sync();
  }

  void Box(float score = 0.8F) {
    auto* d = det.add_detection();
    d->set_center_x(2);
    d->set_center_y(1.4);
    d->set_center_z(0.8);
    d->set_size_x(0.8);
    d->set_size_y(0.6);
    d->set_size_z(0.8);
    d->set_yaw(0);
    d->set_class_id(0);
    d->set_score(score);
  }

  void Sync() {
    *det.mutable_header() = cloud.header();
    det.set_sensor_frame_id("lidar");
    det.set_model_revision("fixture-model");
    det.set_backend(config.detection_backend());
    // Accumulated sweep count is intentionally not the current cloud count.
    det.set_input_point_count(1000);
    *seg.mutable_header() = cloud.header();
    seg.set_source_frame_id("lidar");
    seg.set_source_topic(config.pointcloud_channel());
    seg.set_point_count(cloud.point_size());
  }

  void Run(const DetectionFrame* d, const SemanticFrame* s,
           const GroundPrior* ground_prior = nullptr,
           const ObjectPrior* object_prior = nullptr) {
    SingleFrameFusion fusion;
    std::string error;
    ASSERT_TRUE(fusion.Init(config, &error)) << error;
    ASSERT_TRUE(fusion.Process(cloud, d, s, &output, &error, ground_prior,
                               object_prior))
        << error;
  }

  FusionConfig config;
  drivers::PointCloud cloud;
  DetectionFrame det;
  SemanticFrame seg;
  FusionOutput output;
};

}  // namespace testing_support
}  // namespace lidar_fusion
}  // namespace apollo
