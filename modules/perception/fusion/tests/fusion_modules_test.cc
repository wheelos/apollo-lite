#include <limits>
#include <cmath>

#include "Eigen/Eigenvalues"

#include "modules/perception/fusion/lib/data_association/hm_data_association/hm_tracks_objects_match.h"
#include "modules/perception/fusion/base/object_snapshot.h"
#include "modules/perception/fusion/base/sensor_data_manager.h"
#include "modules/perception/fusion/common/dst_evidence.h"
#include "modules/perception/fusion/common/kalman_filter.h"
#include "modules/perception/fusion/lib/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.h"
#include "modules/perception/fusion/lib/data_fusion/motion_fusion/kalman_motion_fusion/kalman_motion_fusion.h"
#include "modules/perception/fusion/lib/data_fusion/shape_fusion/pbf_shape_fusion/pbf_shape_fusion.h"
#include "modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper/pbf_gatekeeper.h"
#include "modules/perception/fusion/tests/test_support.h"

namespace apollo {
namespace perception {
namespace fusion {
namespace testing {

TEST_F(FusionTest, PredictionMovesGeometryAndGrowsPosteriorCovariance) {
  auto track = NewTrack(Observation(10.0));
  KalmanMotionFusion motion(track);
  ASSERT_TRUE(motion.Init());
  const auto object = track->GetFusedObject()->GetBaseObject();
  const float initial_covariance = object->center_uncertainty(0, 0);
  const double initial_corner = object->polygon[0].x;
  ASSERT_TRUE(motion.PredictTo(11.0));
  EXPECT_NEAR(object->center.x(), 2.0, 1e-8);
  EXPECT_NEAR(object->polygon[0].x, initial_corner + 2.0, 1e-8);
  EXPECT_GT(object->center_uncertainty(0, 0), initial_covariance);
  const auto covariance = object->center_uncertainty;
  ASSERT_TRUE(motion.PredictTo(11.0));
  EXPECT_TRUE(object->center_uncertainty.isApprox(covariance));
  EXPECT_FALSE(motion.PredictTo(10.9));
  EXPECT_NEAR(object->center.x(), 2.0, 1e-8);
  EXPECT_DOUBLE_EQ(track->GetLastObservationTimestamp(), 10.0);
}

TEST_F(FusionTest, CorrectionPublishesPosteriorRatherThanInputCovariance) {
  auto track = NewTrack(Observation(10.0));
  KalmanMotionFusion motion(track, 4.0, 0.001, false);
  ASSERT_TRUE(motion.Init());
  ASSERT_TRUE(motion.PredictTo(10.1));
  auto observation = Observation(10.1, 0.4);
  ASSERT_TRUE(motion.UpdateWithMeasurement(Measurement(observation), 10.1));
  auto object = track->GetFusedObject()->GetBaseObject();
  EXPECT_GT(object->center.x(), 0.2);
  EXPECT_LT(object->center.x(), 0.4);
  EXPECT_LT(object->center_uncertainty(0, 0), 1.0f);
  EXPECT_TRUE(object->center_uncertainty.isApprox(
      object->center_uncertainty.transpose()));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(
      object->center_uncertainty.topLeftCorner<2, 2>());
  EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0f);
}

TEST_F(FusionTest, UnconvergedVelocityIsNotAnObservedState) {
  auto track = NewTrack(Observation(10.0));
  KalmanMotionFusion motion(track);
  ASSERT_TRUE(motion.Init());
  auto observation = Observation(10.1, 0.2);
  observation->objects[0]->velocity.x() = 1000.0f;
  observation->objects[0]->velocity_converged = false;
  ASSERT_TRUE(motion.UpdateWithMeasurement(Measurement(observation), 10.1));
  EXPECT_NEAR(track->GetFusedObject()->GetBaseObject()->velocity.x(), 2.0,
              1e-5);
}

TEST_F(FusionTest, UnconvergedBirthVelocityStartsWithAnUncertainPrior) {
  auto observation = Observation(10.0);
  observation->objects[0]->velocity_converged = false;
  observation->objects[0]->velocity.x() = 1000;
  auto track = NewTrack(observation);
  KalmanMotionFusion motion(track);
  ASSERT_TRUE(motion.Init());
  EXPECT_FLOAT_EQ(track->GetFusedObject()->GetBaseObject()->velocity.x(), 0);
  EXPECT_GE(track->GetFusedObject()->GetBaseObject()->velocity_uncertainty(0, 0),
            100);
}

TEST_F(FusionTest, CovarianceIntersectionDoesNotDoubleCountRepeatedEvidence) {
  KalmanFilter filter;
  ASSERT_TRUE(filter.Init(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity()));
  Eigen::Matrix3d projection = Eigen::Matrix3d::Zero();
  projection(0, 0) = 1;
  ASSERT_TRUE(filter.SetControlMatrix(projection));
  for (int index = 0; index < 100; ++index) {
    ASSERT_TRUE(filter.CorrectCorrelated(Eigen::Vector3d::Zero(),
                                         Eigen::Matrix3d::Identity(), 0.5));
  }
  EXPECT_TRUE(filter.GetUncertainty().isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_FALSE(filter.CorrectCorrelated(Eigen::Vector3d::Zero(),
                                        Eigen::Matrix3d::Identity(), 0));
  Eigen::Matrix3d invalid = Eigen::Matrix3d::Identity();
  invalid(0, 0) = -1;
  EXPECT_FALSE(filter.Correct(Eigen::Vector3d::Zero(), invalid));
  EXPECT_TRUE(filter.GetUncertainty().isApprox(Eigen::Matrix3d::Identity()));
}

TEST_F(FusionTest, DstRejectsInvalidMassAndTotalConflictWithoutPoisoningState) {
  auto* manager = DstManager::Instance();
  ASSERT_TRUE(manager->AddApp("fusion_conflict_test", {1, 2, 3}));
  Dst lhs("fusion_conflict_test");
  Dst rhs("fusion_conflict_test");
  Dst result("fusion_conflict_test");
  ASSERT_TRUE(lhs.SetBba({{1, 1}}));
  ASSERT_TRUE(rhs.SetBba({{2, 1}}));
  EXPECT_FALSE(lhs.TryCombine(rhs, &result));
  EXPECT_DOUBLE_EQ(result.GetSubsetBfmass(3), 1);
  EXPECT_FALSE(lhs.SetBba({{1, 0}}));
  EXPECT_FALSE(lhs.SetBba(
      {{1, std::numeric_limits<double>::quiet_NaN()}}));
  EXPECT_DOUBLE_EQ(lhs.GetSubsetBfmass(1), 1);
}

TEST_F(FusionTest, UnconfiguredMissCoverageDecaysButDoesNotInventAbsence) {
  ASSERT_TRUE(DstExistenceFusion::Init());
  auto initial = Observation(10.0, 0, "radar_front",
                             base::SensorType::LONG_RANGE_RADAR);
  auto track = NewTrack(initial);
  DstExistenceFusion existence(track);
  ASSERT_TRUE(existence.UpdateWithMeasurement(Measurement(initial), 10.0, 0));
  const double before = existence.GetExistenceProbability();
  auto missing = Observation(10.1, 0, "radar_front",
                             base::SensorType::LONG_RANGE_RADAR);
  missing->objects.clear();
  ASSERT_TRUE(SensorDataManager::Instance()->AddSensorMeasurements(missing));
  ASSERT_TRUE(existence.UpdateWithoutMeasurement("radar_front", 10.1, 10.1, 0));
  EXPECT_NEAR(existence.GetExistenceProbability(),
      0.5 + (before - 0.5) * std::exp2(-0.1 / 5.0), 1e-8);
}

TEST_F(FusionTest, InvalidMotionMeasurementsAreRejectedBeforeMutation) {
  auto track = NewTrack(Observation(10.0));
  KalmanMotionFusion motion(track);
  ASSERT_TRUE(motion.Init());
  auto observation = Observation(10.1, 0.2);
  observation->objects[0]->center_uncertainty(0, 0) = -1;
  EXPECT_FALSE(motion.UpdateWithMeasurement(Measurement(observation), 10.1));
  EXPECT_DOUBLE_EQ(track->GetFusedObject()->GetBaseObject()->center.x(), 0.0);
  observation->objects[0]->center_uncertainty.setIdentity();
  observation->objects[0]->center.x() =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(motion.UpdateWithMeasurement(Measurement(observation), 10.1));
}

TEST_F(FusionTest, ShapeFusionCannotOverwriteMotionCenterOrCovariance) {
  auto track = NewTrack(Observation(10.0));
  auto object = track->GetFusedObject()->GetBaseObject();
  object->center.x() = 1.0;
  object->anchor_point = object->center;
  object->center_uncertainty *= 0.25f;
  PbfShapeFusion shape(track);
  auto observation = Observation(10.1, 4.0);
  observation->objects[0]->size.x() = 6.0f;
  shape.UpdateWithMeasurement(Measurement(observation), 10.1);
  EXPECT_DOUBLE_EQ(object->center.x(), 1.0);
  EXPECT_DOUBLE_EQ(object->anchor_point.x(), 1.0);
  EXPECT_FLOAT_EQ(object->center_uncertainty(0, 0), 0.25f);
  EXPECT_FLOAT_EQ(object->size.x(), 6.0f);
  EXPECT_DOUBLE_EQ(object->polygon[0].x, -1.0);
}

TEST_F(FusionTest, LifecycleCountsObservationTimesNotSensorsOrQueries) {
  auto track = NewTrack(Observation(10.0));
  EXPECT_EQ(track->GetTrackedTimes(), 1);
  track->UpdateWithSensorObject(Measurement(Observation(
      10.0, 0.0, "radar_front", base::SensorType::LONG_RANGE_RADAR)));
  EXPECT_EQ(track->GetTrackedTimes(), 1);
  track->UpdateWithSensorObject(Measurement(Observation(10.1, 0.2)));
  EXPECT_EQ(track->GetTrackedTimes(), 2);
  track->ExpireSensorObjects(10.2);
  EXPECT_EQ(track->GetTrackedTimes(), 2);
  track->ExpireSensorObjects(11.0);
  EXPECT_FALSE(track->IsAlive());
  track->SetExistenceProb(0.9);
  track->Reset();
  EXPECT_EQ(track->GetTrackedTimes(), 0);
  EXPECT_DOUBLE_EQ(track->GetExistenceProb(), 0.0);
}

TEST_F(FusionTest, GatekeeperIsReadOnlyAndConfirmationRequiresObservations) {
  pipeline::PluginConfig config;
  config.mutable_pbf_gatekeeper_config()->set_use_track_time_pub_strategy(true);
  config.mutable_pbf_gatekeeper_config()->set_pub_track_time_thresh(1);
  PbfGatekeeper gate;
  ASSERT_TRUE(gate.Init(config));
  auto track = NewTrack(Observation(10.0));
  EXPECT_EQ(gate.Decide(track).reason, PublicationReason::kUnconfirmed);
  for (int i = 0; i < 10; ++i) EXPECT_FALSE(gate.AbleToPublish(track));
  EXPECT_EQ(track->GetTrackedTimes(), 1);
  track->UpdateWithSensorObject(Measurement(Observation(10.1, 0.2)));
  EXPECT_TRUE(gate.AbleToPublish(track));
  EXPECT_TRUE(gate.AbleToPublish(track));
  EXPECT_EQ(track->GetTrackedTimes(), 2);
  track->Expire();
  EXPECT_EQ(gate.Decide(track).reason, PublicationReason::kExpired);
}

TEST_F(FusionTest, RadarAndArbitraryCameraSourcesUseConfiguredPolicy) {
  pipeline::PluginConfig config;
  auto policy = config.mutable_pbf_gatekeeper_config();
  policy->set_radar_existence_threshold(0.9);
  policy->set_existence_threshold(0.7);
  PbfGatekeeper gate;
  ASSERT_TRUE(gate.Init(config));
  auto radar = NewTrack(Observation(10.0, 0.0, "radar_front",
                                    base::SensorType::LONG_RANGE_RADAR));
  radar->SetExistenceProb(0.95);
  EXPECT_TRUE(gate.AbleToPublish(radar));
  auto camera = NewTrack(Observation(10.0, 0.0, "arbitrary_camera",
                                     base::SensorType::MONOCULAR_CAMERA));
  camera->SetExistenceProb(0.8);
  EXPECT_TRUE(gate.AbleToPublish(camera));
  policy->add_blocked_publish_sensors("radar_front");
  ASSERT_TRUE(gate.Init(config));
  EXPECT_FALSE(gate.AbleToPublish(radar));
  EXPECT_TRUE(gate.AbleToPublish(camera));
}

TEST_F(FusionTest, AssociationReturnsActualNormalizedLossForLocalIdMatches) {
  auto scene = std::make_shared<Scene>();
  scene->AddForegroundTrack(NewTrack(Observation(10.0)));
  HMTrackersObjectsAssociation matcher;
  ASSERT_TRUE(matcher.Init());
  AssociationResult result;
  ASSERT_TRUE(matcher.Associate(
      AssociationOptions(), std::make_shared<SensorFrame>(Observation(10.1, 2)),
      scene, &result));
  ASSERT_EQ(result.assignments.size(), 1);
  EXPECT_NEAR(result.track_association_loss[0], 0.5, 1e-8);
}

TEST_F(FusionTest, LocalIdCannotBypassGeometryGate) {
  auto scene = std::make_shared<Scene>();
  scene->AddForegroundTrack(NewTrack(Observation(10.0)));
  HMTrackersObjectsAssociation matcher;
  ASSERT_TRUE(matcher.Init());
  AssociationResult result;
  ASSERT_TRUE(matcher.Associate(
      AssociationOptions(),
      std::make_shared<SensorFrame>(Observation(10.1, 20)), scene, &result));
  EXPECT_TRUE(result.assignments.empty());
  ASSERT_EQ(result.unassigned_tracks.size(), 1);
  EXPECT_DOUBLE_EQ(result.track_miss_similarity[0], 0.0);
}

TEST_F(FusionTest, RadarOnlyTracksCanAssociateAcrossLocalIdChanges) {
  auto scene = std::make_shared<Scene>();
  scene->AddForegroundTrack(NewTrack(Observation(
      10.0, 0.0, "radar_front", base::SensorType::LONG_RANGE_RADAR)));
  HMTrackersObjectsAssociation matcher;
  ASSERT_TRUE(matcher.Init());
  AssociationResult result;
  auto observation = Observation(10.1, 0.2, "radar_front",
                                 base::SensorType::LONG_RANGE_RADAR, 8);
  ASSERT_TRUE(matcher.Associate(AssociationOptions(),
                                std::make_shared<SensorFrame>(observation),
                                scene, &result));
  ASSERT_EQ(result.assignments.size(), 1);
  EXPECT_NEAR(result.track_association_loss[0], 0.05, 1e-8);
}

TEST_F(FusionTest, TightCovarianceCanRejectAGeometricallyNearbyMatch) {
  auto initial = Observation(10.0);
  initial->objects[0]->center_uncertainty *= 0.01f;
  auto incoming = Observation(10.1, 1);
  incoming->objects[0]->center_uncertainty *= 0.01f;
  auto scene = std::make_shared<Scene>();
  scene->AddForegroundTrack(NewTrack(initial));
  HMTrackersObjectsAssociation matcher;
  ASSERT_TRUE(matcher.Init());
  AssociationResult result;
  ASSERT_TRUE(matcher.Associate(AssociationOptions(),
      std::make_shared<SensorFrame>(incoming), scene, &result));
  EXPECT_TRUE(result.assignments.empty());
}

TEST_F(FusionTest, AssociationUsesVelocityGateAndShapeAndClassPenalties) {
  auto scene = std::make_shared<Scene>();
  scene->AddForegroundTrack(NewTrack(Observation(10.0)));
  auto incoming = Observation(10.1);
  incoming->objects[0]->velocity.x() = 100;
  HMTrackersObjectsAssociation matcher;
  ASSERT_TRUE(matcher.Init());
  AssociationResult result;
  ASSERT_TRUE(matcher.Associate(AssociationOptions(),
      std::make_shared<SensorFrame>(incoming), scene, &result));
  EXPECT_TRUE(result.assignments.empty());
  incoming->objects[0]->velocity.x() = 2;
  incoming->objects[0]->size.x() = 8;
  incoming->objects[0]->type = base::ObjectType::BICYCLE;
  ASSERT_TRUE(matcher.Associate(AssociationOptions(),
      std::make_shared<SensorFrame>(incoming), scene, &result));
  ASSERT_EQ(result.assignments.size(), 1);
  EXPECT_NEAR(result.track_association_loss[0], 0.2 + 0.1 / 6.0, 1e-8);
}

TEST_F(FusionTest, Camera2dAssociationUsesProjectionNotFabricatedWorldPosition) {
  auto scene = std::make_shared<Scene>();
  scene->AddForegroundTrack(NewTrack(Observation(10.0)));
  auto camera = Observation(10.1, 1000, "front_6mm",
                             base::SensorType::MONOCULAR_CAMERA);
  camera->objects[0]->camera_supplement.local_center.setZero();
  camera->objects[0]->center.z() = -100;
  auto& box = camera->objects[0]->camera_supplement.box;
  box.xmin = 462;
  box.xmax = 818;
  box.ymin = 271;
  box.ymax = 449;
  ASSERT_TRUE(SensorDataManager::Instance()->AddSensorMeasurements(camera));
  HMTrackersObjectsAssociation matcher;
  ASSERT_TRUE(matcher.Init());
  AssociationResult result;
  ASSERT_TRUE(matcher.Associate(AssociationOptions(),
      std::make_shared<SensorFrame>(camera), scene, &result));
  ASSERT_EQ(result.assignments.size(), 1);
  EXPECT_LT(result.track_association_loss[0], 0.01);
}

TEST_F(FusionTest, SnapshotsRequirePredictedTimeAndRemainIndependentOfTheTrack) {
  auto track = NewTrack(Observation(10.0));
  KalmanMotionFusion motion(track);
  ASSERT_TRUE(motion.Init());
  ASSERT_TRUE(motion.PredictTo(10.1));
  std::vector<base::ObjectPtr> output;
  EXPECT_FALSE(AppendTrackSnapshot(10.2, track, &output));
  EXPECT_TRUE(output.empty());
  ASSERT_TRUE(AppendTrackSnapshot(10.1, track, &output));
  ASSERT_EQ(output.size(), 1);
  EXPECT_NEAR(output[0]->center.x(), 0.2, 1e-8);
  ASSERT_TRUE(motion.PredictTo(10.2));
  EXPECT_NEAR(output[0]->center.x(), 0.2, 1e-8);
  EXPECT_DOUBLE_EQ(output[0]->fusion_supplement.measurements[0].timestamp, 10.0);
}

}  // namespace testing
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
