#pragma once

#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/fusion/base/sensor_frame.h"
#include "modules/perception/fusion/base/track.h"

namespace apollo {
namespace perception {
namespace fusion {
namespace testing {

class FusionTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    FLAGS_work_root = "modules/perception/fusion/tests/data";
    FLAGS_config_manager_path = ".";
    FLAGS_obs_sensor_meta_path = "sensor_meta.pt";
    FLAGS_obs_sensor_intrinsic_path = "modules/perception/fusion/tests/data";
    ASSERT_TRUE(common::SensorManager::Instance()->Init());
  }
};

inline base::FramePtr Observation(
    double timestamp, double x = 0.0, const std::string& sensor = "velodyne128",
    base::SensorType type = base::SensorType::VELODYNE_128, int local_id = 7) {
  auto frame = std::make_shared<base::Frame>();
  frame->timestamp = timestamp;
  frame->sensor_info.name = sensor;
  frame->sensor_info.type = type;
  auto object = std::make_shared<base::Object>();
  object->center = Eigen::Vector3d(x, 0.0, 10.0);
  object->anchor_point = object->center;
  object->size = Eigen::Vector3f(4.0f, 2.0f, 2.0f);
  object->velocity = Eigen::Vector3f(2.0f, 0.0f, 0.0f);
  object->center_uncertainty.setIdentity();
  object->velocity_uncertainty.setIdentity();
  object->acceleration_uncertainty.setIdentity();
  object->track_id = local_id;
  object->type = base::ObjectType::VEHICLE;
  object->type_probs[static_cast<size_t>(base::ObjectType::VEHICLE)] = 1.0f;
  object->latest_tracked_time = timestamp;
  object->radar_supplement.range = 50.0;
  object->camera_supplement.local_center = object->center.cast<float>();
  object->camera_supplement.box.xmin = 0;
  object->camera_supplement.box.ymin = 0;
  object->camera_supplement.box.xmax = 1280;
  object->camera_supplement.box.ymax = 720;
  object->polygon.resize(4);
  for (size_t i = 0; i < 4; ++i) {
    object->polygon[i].x = x + (i < 2 ? -2.0 : 2.0);
    object->polygon[i].y = i % 2 == 0 ? -1.0 : 1.0;
    object->polygon[i].z = 10.0;
  }
  frame->objects.push_back(object);
  return frame;
}

inline SensorObjectPtr Measurement(const base::FramePtr& frame) {
  auto sensor_frame = std::make_shared<SensorFrame>(frame);
  return sensor_frame->GetForegroundObjects().front();
}

inline TrackPtr NewTrack(const base::FramePtr& frame) {
  auto track = std::make_shared<Track>();
  track->Initialize(Measurement(frame));
  return track;
}

}  // namespace testing
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
