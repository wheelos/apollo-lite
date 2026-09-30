#include "cyber/common/file.h"
#include "modules/perception/fusion/app/fusion_runtime.h"
#include "modules/perception/fusion/lib/data_fusion/all_latest_fusion/all_latest_fusion.h"
#include "modules/perception/fusion/lib/fusion_system/probabilistic_fusion/probabilistic_fusion.h"
#include "modules/perception/fusion/lib/gatekeeper/collect_fused_object.h"
#include "modules/perception/fusion/tests/test_support.h"
#include "modules/perception/onboard/msg_serializer/msg_serializer.h"

namespace apollo {
namespace perception {
namespace fusion {
namespace testing {

class FusionPipelineTest : public FusionTest {
 protected:
  void SetUp() override {
    SensorDataManager::Instance()->Reset();
    ASSERT_TRUE(SensorDataManager::Instance()->Init());
    pipeline::PipelineConfig config;
    ASSERT_TRUE(cyber::common::GetProtoFromFile(
        "modules/perception/pipeline/config/multi_sensor_fusion_pipeline.txt",
        &config));
    ASSERT_EQ(config.stage_config_size(), 3);
    ASSERT_TRUE(scheduler_.Init(config.stage_config(0)));
    ASSERT_TRUE(fusion_.Init(config.stage_config(1)));
    ASSERT_TRUE(collection_.Init(config.stage_config(2)));
    ASSERT_TRUE(runtime_.Init("velodyne128", 0, [this](FusionFrame* frame) {
      pipeline::DataFrame data;
      data.fusion_frame = frame;
      return scheduler_.Process(&data) && fusion_.Process(&data) &&
             collection_.Process(&data);
    }));
  }

  AllLatestFusion scheduler_;
  ProbabilisticFusion fusion_;
  CollectFusedObject collection_;
  FusionRuntime runtime_;
};

TEST_F(FusionPipelineTest, ConfirmationCannotAdvanceWithoutNewMeasurements) {
  ASSERT_TRUE(runtime_.Add(Observation(10.0)));
  FusionFrame output;
  ASSERT_TRUE(runtime_.Tick(10.01, &output));
  ASSERT_EQ(output.scene_ptr->GetForegroundTracks().size(), 1);
  const auto track = output.scene_ptr->GetForegroundTracks()[0];
  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(runtime_.Tick(10.02 + i * 0.01, &output));
    EXPECT_TRUE(output.fused_objects.empty());
  }
  EXPECT_EQ(track->GetTrackedTimes(), 1);
  for (int i = 1; i < 4; ++i) {
    const double timestamp = 10.1 + i * 0.01;
    ASSERT_TRUE(runtime_.Add(Observation(timestamp, 2.0 * (timestamp - 10.0))));
  }
  ASSERT_TRUE(runtime_.Tick(10.14, &output));
  ASSERT_EQ(output.fused_objects.size(), 1);
  EXPECT_EQ(track->GetTrackedTimes(), 4);
}

TEST_F(FusionPipelineTest, ConsumesAllQueuedFramesInOrderAndRejectsLateFrames) {
  ASSERT_TRUE(runtime_.Add(Observation(10.15, 0.3)));
  ASSERT_TRUE(runtime_.Add(Observation(10.05, 0.1)));
  ASSERT_TRUE(runtime_.Add(Observation(10.0, 0.0)));
  ASSERT_TRUE(runtime_.Add(Observation(10.1, 0.2)));
  FusionFrame output;
  ASSERT_TRUE(runtime_.Tick(10.16, &output));
  ASSERT_EQ(output.fused_objects.size(), 1);
  ASSERT_EQ(output.scene_ptr->GetForegroundTracks().size(), 1);
  EXPECT_EQ(output.scene_ptr->GetForegroundTracks()[0]->GetTrackedTimes(), 4);
  EXPECT_NEAR(output.fused_objects[0]->center.x(), 0.32, 0.01);
  EXPECT_FALSE(runtime_.Add(Observation(10.12, 0.24)));
  ASSERT_TRUE(runtime_.Tick(10.17, &output));
  ASSERT_EQ(output.fused_objects.size(), 1);
  EXPECT_NEAR(output.fused_objects[0]->center.x(), 0.34, 0.01);
}

TEST_F(FusionPipelineTest, ThreeSensorsShareOneTrackAndExposeMeasurementTimes) {
  for (int i = 0; i < 4; ++i) {
    const double timestamp = 10.0 + i * 0.05;
    ASSERT_TRUE(runtime_.Add(Observation(timestamp, 0.1 * i)));
    ASSERT_TRUE(runtime_.Add(Observation(timestamp, 0.1 * i, "radar_front",
                                         base::SensorType::LONG_RANGE_RADAR)));
    ASSERT_TRUE(runtime_.Add(Observation(timestamp, 0.1 * i, "front_6mm",
                                         base::SensorType::MONOCULAR_CAMERA)));
  }
  FusionFrame output;
  ASSERT_TRUE(runtime_.Tick(10.16, &output));
  ASSERT_EQ(output.fused_objects.size(), 1);
  ASSERT_EQ(output.scene_ptr->GetForegroundTracks().size(), 1);
  EXPECT_EQ(output.scene_ptr->GetForegroundTracks()[0]->GetTrackedTimes(), 4);
  EXPECT_FALSE(output.degraded);
  EXPECT_NEAR(output.fused_objects[0]->latest_tracked_time, 10.16, 1e-8);
  ASSERT_EQ(output.fused_objects[0]->fusion_supplement.measurements.size(), 3);
  for (const auto& measurement :
       output.fused_objects[0]->fusion_supplement.measurements) {
    EXPECT_NEAR(measurement.timestamp, 10.15, 1e-8);
  }
}

TEST_F(FusionPipelineTest, RadarOnlyPublishesDegradedObjectsThenExpires) {
  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(
        runtime_.Add(Observation(10.0 + i * 0.05, 0.1 * i, "radar_front",
                                 base::SensorType::LONG_RANGE_RADAR, 7 + i)));
  }
  FusionFrame output;
  ASSERT_TRUE(runtime_.Tick(10.16, &output));
  EXPECT_TRUE(output.degraded);
  ASSERT_EQ(output.fused_objects.size(), 1);
  const auto initial = output.fused_objects[0];
  ASSERT_TRUE(runtime_.Tick(10.25, &output));
  ASSERT_EQ(output.fused_objects.size(), 1);
  EXPECT_NEAR(output.fused_objects[0]->center.x(), 0.5, 0.01);
  EXPECT_GT(output.fused_objects[0]->center_uncertainty(0, 0),
            initial->center_uncertainty(0, 0));
  ASSERT_TRUE(runtime_.Tick(10.7, &output));
  EXPECT_TRUE(output.fused_objects.empty());
}

TEST_F(FusionPipelineTest, MainSensorLossIsExplicitAndRadarMaintainsIdentity) {
  for (int i = 0; i < 4; ++i) {
    const double timestamp = 10.0 + i * 0.05;
    ASSERT_TRUE(runtime_.Add(Observation(timestamp, i * 0.1)));
    ASSERT_TRUE(runtime_.Add(Observation(timestamp, i * 0.1, "radar_front",
                                         base::SensorType::LONG_RANGE_RADAR)));
  }
  FusionFrame output;
  ASSERT_TRUE(runtime_.Tick(10.16, &output));
  ASSERT_EQ(output.fused_objects.size(), 1);
  const int id = output.fused_objects[0]->track_id;
  ASSERT_TRUE(runtime_.Add(Observation(10.5, 1.0, "radar_front",
                                       base::SensorType::LONG_RANGE_RADAR)));
  ASSERT_TRUE(runtime_.Tick(10.51, &output));
  ASSERT_EQ(output.fused_objects.size(), 1);
  EXPECT_EQ(output.fused_objects[0]->track_id, id);
  EXPECT_TRUE(output.degraded);
  apollo::perception::PerceptionObstacles message;
  ASSERT_TRUE(onboard::MsgSerializer::SerializeMsg(
      output.frame->timestamp, 0, 1, output.fused_objects,
      apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS, &message));
  ASSERT_EQ(message.perception_obstacle_size(), 1);
  EXPECT_EQ(message.error_code(),
            apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS);
  EXPECT_NEAR(message.perception_obstacle(0).timestamp(), 10.51, 1e-8);
  ASSERT_EQ(message.perception_obstacle(0).position_covariance_size(), 9);
  EXPECT_GT(message.perception_obstacle(0).position_covariance(0), 0.0);
}

TEST_F(FusionPipelineTest,
       EmptyCyclesPublishDegradationWithoutSyntheticTracks) {
  FusionFrame output;
  ASSERT_TRUE(runtime_.Tick(10.0, &output));
  EXPECT_TRUE(output.degraded);
  EXPECT_TRUE(output.fused_objects.empty());
  EXPECT_TRUE(output.scene_ptr->GetForegroundTracks().empty());
  ASSERT_TRUE(runtime_.Tick(10.1, &output));
  EXPECT_TRUE(output.fused_objects.empty());
}

TEST_F(FusionPipelineTest, Camera2dCannotCreateTracksOrRefreshMotionAge) {
  auto camera = Observation(10.0, 1000, "front_6mm",
                             base::SensorType::MONOCULAR_CAMERA);
  camera->objects[0]->camera_supplement.local_center.setZero();
  ASSERT_TRUE(runtime_.Add(camera));
  FusionFrame output;
  ASSERT_TRUE(runtime_.Tick(10.01, &output));
  EXPECT_TRUE(output.scene_ptr->GetForegroundTracks().empty());
  for (int index = 0; index < 4; ++index) {
    const double timestamp = 10.05 + index * 0.05;
    ASSERT_TRUE(runtime_.Add(Observation(timestamp, 0, "radar_front",
                                          base::SensorType::LONG_RANGE_RADAR)));
  }
  ASSERT_TRUE(runtime_.Tick(10.21, &output));
  ASSERT_EQ(output.scene_ptr->GetForegroundTracks().size(), 1);
  const auto track = output.scene_ptr->GetForegroundTracks()[0];
  const double last_motion = track->GetLastMotionObservationTimestamp();
  camera = Observation(10.3, 1000, "front_6mm",
                        base::SensorType::MONOCULAR_CAMERA);
  camera->objects[0]->camera_supplement.local_center.setZero();
  auto& box = camera->objects[0]->camera_supplement.box;
  box.xmin = 470;
  box.xmax = 855;
  box.ymin = 270;
  box.ymax = 450;
  ASSERT_TRUE(runtime_.Add(camera));
  ASSERT_TRUE(runtime_.Tick(10.31, &output));
  ASSERT_EQ(output.scene_ptr->GetForegroundTracks().size(), 1);
  EXPECT_GT(track->GetTrackedTimes(), 4);
  EXPECT_DOUBLE_EQ(track->GetLastMotionObservationTimestamp(), last_motion);
  ASSERT_TRUE(runtime_.Tick(10.71, &output));
  EXPECT_TRUE(output.fused_objects.empty());
  EXPECT_TRUE(output.scene_ptr->GetForegroundTracks().empty());
}

TEST_F(FusionPipelineTest, QueuedObservationDoesNotClearSourceFailure) {
  ASSERT_TRUE(runtime_.ReportSourceError("velodyne128", 10.0));
  ASSERT_TRUE(runtime_.Add(Observation(10.2)));
  FusionFrame output;
  ASSERT_TRUE(runtime_.Tick(10.1, &output));
  EXPECT_TRUE(output.degraded);
  ASSERT_TRUE(runtime_.Tick(10.21, &output));
  EXPECT_FALSE(output.degraded);
  ASSERT_TRUE(runtime_.ReportSourceError("velodyne128", 10.0));
  ASSERT_TRUE(runtime_.Tick(10.22, &output));
  EXPECT_FALSE(output.degraded);
}

}  // namespace testing
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
