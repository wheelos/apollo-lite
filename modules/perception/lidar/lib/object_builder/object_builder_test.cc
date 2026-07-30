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

//  Created Date: 2026-03-06
//  Author: daohu527

#include "modules/perception/lidar/lib/object_builder/object_builder.h"

#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace lidar {

namespace {

template <typename T>
void EmitMetric(const std::string& name, const T& value) {
  std::ostringstream stream;
  stream << value;
  std::cout << "METRIC " << name << "=" << stream.str() << std::endl;
  ::testing::Test::RecordProperty(name, stream.str());
}

pipeline::StageConfig MakeStageConfig() {
  pipeline::StageConfig stage_config;
  stage_config.set_stage_type(pipeline::StageType::OBJECT_GEOMETRY_BUILDER);
  stage_config.set_enabled(true);
  stage_config.set_type("ObjectGeometryBuilder");
  stage_config.mutable_object_geometry_builder_config();
  return stage_config;
}

void AddObjectPoint(base::ObjectPtr object, float x, float y, float z,
                    double timestamp) {
  base::PointF point;
  point.x = x;
  point.y = y;
  point.z = z;
  object->lidar_supplement.cloud.push_back(point, timestamp);
}

base::ObjectPtr MakeBoxLikeObject() {
  auto object = std::make_shared<base::Object>();
  AddObjectPoint(object, 0.0f, 0.0f, 0.0f, 0.1);
  AddObjectPoint(object, 4.0f, 0.0f, 0.0f, 0.2);
  AddObjectPoint(object, 4.0f, 2.0f, 1.0f, 0.3);
  AddObjectPoint(object, 0.0f, 2.0f, 1.0f, 0.4);
  return object;
}

base::ObjectPtr MakeDegenerateObject() {
  auto object = std::make_shared<base::Object>();
  AddObjectPoint(object, 10.0f, 0.0f, 0.0f, 0.5);
  AddObjectPoint(object, 10.0f, 1.0f, 0.0f, 0.7);
  return object;
}

}  // namespace

TEST(ObjectGeometryBuilderStageTest, process_stage_metrics) {
  ObjectGeometryBuilder builder;
  auto stage_config = MakeStageConfig();
  EXPECT_TRUE(builder.Init(stage_config));
  EXPECT_EQ(builder.Name(), "ObjectGeometryBuilder");

  LidarFrame frame;
  frame.segmented_objects.push_back(MakeBoxLikeObject());
  frame.segmented_objects.push_back(MakeDegenerateObject());

  pipeline::DataFrame data_frame{};
  data_frame.lidar_frame = &frame;

  EXPECT_FALSE(builder.Process(nullptr));
  EXPECT_TRUE(builder.Process(&data_frame));

  ASSERT_EQ(frame.segmented_objects.size(), 2u);

  const auto& primary_object = frame.segmented_objects[0];
  ASSERT_NE(primary_object, nullptr);
  EXPECT_EQ(primary_object->id, 0);
  EXPECT_EQ(primary_object->polygon.size(), 4u);
  EXPECT_NEAR(primary_object->center.x(), 2.0, 1e-3);
  EXPECT_NEAR(primary_object->center.y(), 1.0, 1e-3);
  EXPECT_NEAR(primary_object->center.z(), 0.0, 1e-3);
  EXPECT_NEAR(primary_object->size(0), 4.0f, 1e-3f);
  EXPECT_NEAR(primary_object->size(1), 2.0f, 1e-3f);
  EXPECT_NEAR(primary_object->size(2), 1.0f, 1e-3f);
  EXPECT_NEAR(primary_object->theta, 0.0f, 1e-6f);
  EXPECT_NEAR(primary_object->latest_tracked_time, 0.25, 1e-6);

  const auto& degenerate_object = frame.segmented_objects[1];
  ASSERT_NE(degenerate_object, nullptr);
  EXPECT_EQ(degenerate_object->id, 1);
  EXPECT_EQ(degenerate_object->polygon.size(), 4u);
  EXPECT_NEAR(degenerate_object->center.x(), 10.0, 1e-3);
  EXPECT_NEAR(degenerate_object->center.y(), 0.5, 1e-3);
  EXPECT_NEAR(degenerate_object->center.z(), 0.0, 1e-3);
  EXPECT_GE(degenerate_object->size(0), 1.0f);
  EXPECT_GE(degenerate_object->size(1), 0.01f);
  EXPECT_GE(degenerate_object->size(2), 0.01f);
  EXPECT_NEAR(degenerate_object->latest_tracked_time, 0.6, 1e-6);

  EmitMetric("object_count", frame.segmented_objects.size());
  EmitMetric("primary_polygon_vertices", primary_object->polygon.size());
  EmitMetric("degenerate_polygon_vertices", degenerate_object->polygon.size());
  EmitMetric("primary_length", primary_object->size(0));
  EmitMetric("primary_width", primary_object->size(1));
  EmitMetric("primary_height", primary_object->size(2));
  EmitMetric("primary_latest_tracked_time",
             primary_object->latest_tracked_time);
  EmitMetric("degenerate_length", degenerate_object->size(0));
  EmitMetric("degenerate_width", degenerate_object->size(1));
  EmitMetric("degenerate_height", degenerate_object->size(2));
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
