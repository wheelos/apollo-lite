// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <unistd.h>

#include <cmath>
#include <fstream>
#include <limits>

#include "gtest/gtest.h"
#include "native_global_test_input.h"

namespace apollo::localization::openloc_diagnostic {
namespace {
using Clock = NativeGlobalWorker::Clock;

class GlobalWorkerTest : public ::testing::Test {
 protected:
  std::string Journal() {
    return ::testing::TempDir() + "global-" +
        ::testing::UnitTest::GetInstance()->current_test_info()->name() + "-" +
        std::to_string(getpid()) + ".csv";
  }
  void Start(bool map_enabled = false, bool raw_gga = false, bool reset = false) {
    auto config = test::Config(Journal());
    if (raw_gga) test::EnableRawGga(&config);
    if (reset) test::EnableReset(&config);
    if (map_enabled) config.set_map_topic("/openloc/diagnostic/global-test/map");
    worker = std::make_unique<NativeGlobalWorker>(
        config, [&](const NativeGlobalProcessed& value) {
          {
            std::lock_guard<std::mutex> lock(mutex);
            decisions.push_back(value);
          }
          condition.notify_all();
        });
  }
  template <typename T>
  NativeGlobalProcessed Send(const T& input) {
    std::size_t count;
    {
      std::lock_guard<std::mutex> lock(mutex);
      count = decisions.size();
    }
    if constexpr (std::is_same_v<T, NativeOdometryDiagnostic>)
      worker->AddOdometry(input, Clock::now());
    else if constexpr (std::is_same_v<T, NativeJointOdometryInput>)
      worker->AddGraphOdometry(input, Clock::now());
    else if constexpr (std::is_same_v<T, NativeGlobalGnssInput>)
      worker->AddGnss(input, Clock::now());
    else if constexpr (std::is_same_v<T, NativeGlobalWorker::RawGgaPacket>)
      worker->AddRawGga(input, Clock::now());
    else if constexpr (std::is_same_v<T, NativeGlobalWorker::BestPosePacket>)
      worker->AddBestPose(input, Clock::now());
    else if constexpr (std::is_same_v<T, NativeGlobalWorker::HeadingPacket>)
      worker->AddHeading(input, Clock::now());
    else if constexpr (std::is_same_v<T, NativeGlobalResetRequest>)
      worker->AddReset(input, Clock::now());
    else if constexpr (std::is_same_v<T, NativeGlobalResetAck>)
      worker->AddResetAck(input, Clock::now());
    else
      worker->AddMap(input, Clock::now());
    std::unique_lock<std::mutex> lock(mutex);
    if (!condition.wait_for(lock, std::chrono::seconds(5),
                            [&] { return decisions.size() > count; }))
      throw std::runtime_error("global input completion timeout");
    return decisions.back();
  }
  void Acquire(uint64_t alignment_generation = 0) {
    for (int i = 0; i < 12; ++i) {
      const auto source = test::Odometry(i);
      Send(source);
      Send(alignment_generation ? test::Alignment(test::Gnss(source),
                                                   alignment_generation)
                                : test::Gnss(source));
    }
  }
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeGlobalProcessed> decisions;
  std::unique_ptr<NativeGlobalWorker> worker;
};

NativeJointOdometryInput SyntheticJointOdometry(int index) {
  NativeGraphFrame frame;
  frame.nominal.stamp = {100 + 0.01 * index, 100 + 0.01 * index,
                         static_cast<uint64_t>(index + 1), "unix"};
  frame.nominal.epoch = {"synthetic-global-test", 1};
  frame.nominal.position.x() = index;
  frame.nominal.valid = true;
  frame.nominal.covariance.setIdentity();
  frame.joint = frame.nominal;
  frame.joint.covariance *= 1 + index;
  frame.joint.joint_error = ::openloc::localization::unified::JointLocalError{
      Eigen::MatrixXd::Identity(15, 15),
      Eigen::MatrixXd::Identity(15, 15) * std::sqrt(1 + index)};
  if (index > 0) {
    frame.motion.emplace();
    auto& motion = *frame.motion;
    motion.epoch = frame.nominal.epoch;
    motion.start = {100 + 0.01 * (index - 1), 100 + 0.01 * (index - 1),
                    static_cast<uint64_t>(index), "unix"};
    motion.end = frame.nominal.stamp;
    motion.cross_covariance.setIdentity();
    motion.cross_covariance *= index;
    motion.joint_cross_covariance =
        Eigen::MatrixXd::Identity(15, 15) * std::sqrt(index / (1.0 + index));
    motion.sources = {{"imu", static_cast<uint64_t>(index)},
                      {"imu", static_cast<uint64_t>(index + 1)}};
    frame.previous_dimension = 15;
  }
  NativePublication publication(0.2);
  const auto now = Clock::now();
  publication.Update(frame.nominal, {}, now);
  auto source = publication.Sample(200, now);
  source.set_publication_sequence(index + 1);
  source.set_recorded_input(true);
  return EncodeNativeGraph(frame, source, "synthetic-joint-reset");
}

NativeGlobalMapInput IndependentSyntheticMap(const NativeJointOdometryInput& source,
                                            uint64_t generation) {
  auto map = test::Alignment(test::Map(source.odometry()), generation);
  map.set_odometry_initialization(NativeGlobalGnssInput::ABSENT);
  map.set_odometry_deskew(NativeGlobalGnssInput::ABSENT);
  map.set_shared_local_scans(NativeGlobalGnssInput::ABSENT);
  map.set_gnss_map_construction(NativeGlobalGnssInput::ABSENT);
  auto* qualification = map.mutable_graph_qualification();
  qualification->set_uncertainty_model_id("independent-synthetic-pose");
  qualification->set_independence_evidence_id("synthetic-independent-generator");
  qualification->set_quality_valid(true);
  qualification->set_uncertainty_qualified(false);
  qualification->set_independent_of_local(true);
  qualification->set_independent_of_graph_sources(true);
  return map;
}

TEST_F(GlobalWorkerTest, StructuredHeadingAndPositionUseSelectedGraphWithoutFullPose) {
  auto config = test::Config(Journal());
  test::EnableTargetGraph(&config);
  test::EnableStructuredGnss(&config);
  auto* graph = config.mutable_target_graph();
  graph->set_motion_error_model_id("synthetic-joint-reset");
  graph->set_maximum_error_dimension(32);
  graph->set_history_duration_ms(3000);
  graph->set_maximum_observation_age_ms(2000);
  auto* structured = config.mutable_structured_gnss();
  structured->set_fixed_stability_ns(0);
  structured->mutable_heading()->set_fixed_stability_ns(0);
  structured->set_gnss_map_construction(NativeGlobalGnssInput::ABSENT);
  const auto qualify = [](NativeGraphSourceQualification* source, const char* model) {
    source->set_uncertainty_model_id(model);
    source->set_independence_evidence_id("synthetic-independent-generator");
    source->set_quality_valid(true);
    source->set_uncertainty_qualified(false);
    source->set_independent_of_local(true);
    source->set_independent_of_graph_sources(true);
  };
  qualify(structured->mutable_position_graph_qualification(), "synthetic-position");
  qualify(structured->mutable_heading_graph_qualification(), "synthetic-heading");
  worker = std::make_unique<NativeGlobalWorker>(config, [&](const auto& decision) {
    std::lock_guard<std::mutex> lock(mutex);
    decisions.push_back(decision);
    condition.notify_all();
  });
  for (int i = 0; i < 4; ++i) {
    const auto source = SyntheticJointOdometry(i);
    ASSERT_EQ(Send(source).status(), 0);
    auto heading = test::DualHeading(i);
    auto position = test::BestPose(i);
    const double seconds = 100 + 0.01 * i;
    heading.set_measurement_time(seconds);
    position.set_measurement_time(seconds);
    const auto receive = static_cast<int64_t>(std::llround(seconds * 1e9));
    Send(NativeGlobalWorker::HeadingPacket{heading, receive});
    Send(NativeGlobalWorker::BestPosePacket{position, receive});
  }
  const auto output = worker->Sample(200);
  ASSERT_TRUE(output.usable()) << output.reason() << ' ' << output.graph_assessment_reason();
  EXPECT_EQ(output.selected_owner(), "fixed_lag");
  EXPECT_GT(output.correction_id(), 0);
  EXPECT_FALSE(output.full_pose_available());
  EXPECT_FALSE(output.has_map_from_imu());
  EXPECT_FALSE(output.trusted());
  worker->Finish();
  std::ifstream file(Journal());
  const std::string ledger((std::istreambuf_iterator<char>(file)), {});
  EXPECT_NE(ledger.find("novatel-best-pose-horizontal"), std::string::npos);
  EXPECT_NE(ledger.find("novatel-dual-antenna-heading"), std::string::npos);
  std::ifstream innovations(Journal() + ".innovations.csv");
  const std::string innovation_ledger(
      (std::istreambuf_iterator<char>(innovations)), {});
  EXPECT_NE(innovation_ledger.find("pre_update_residual"), std::string::npos);
  EXPECT_NE(innovation_ledger.find("novatel-best-pose-horizontal"),
            std::string::npos);
  EXPECT_NE(innovation_ledger.find("novatel-dual-antenna-heading"),
            std::string::npos);
}

TEST_F(GlobalWorkerTest, SelectedGraphResetsReacquiresAndNeverFallsBackToDiagnosticOwner) {
  auto config = test::Config(Journal());
  test::EnableTargetGraph(&config);
  test::EnableReset(&config);
  config.set_map_topic("/openloc/diagnostic/target-graph-prepared-map");
  config.mutable_target_graph()->set_motion_error_model_id("synthetic-joint-reset");
  config.mutable_target_graph()->set_maximum_error_dimension(32);
  config.mutable_target_graph()->set_history_duration_ms(3000);
  config.mutable_target_graph()->set_maximum_observation_age_ms(2000);
  worker = std::make_unique<NativeGlobalWorker>(config, [&](const auto& decision) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      decisions.push_back(decision);
    }

    condition.notify_all();
  });
  EXPECT_THROW(worker->AddOdometry(test::Odometry(0), Clock::now()),
               std::invalid_argument);
  for (int index = 0; index < 3; ++index) {
    SCOPED_TRACE(index);
    const auto source = SyntheticJointOdometry(index);
    ASSERT_EQ(Send(source).status(), 0);
    Send(IndependentSyntheticMap(source, 1));
  }
  auto output = worker->Sample(200);
  ASSERT_TRUE(output.target_graph_available()) << output.reason();
  EXPECT_EQ(output.selected_owner(), "fixed_lag");
  EXPECT_TRUE(output.diagnostic_only());
  EXPECT_FALSE(output.trusted());
  auto legacy = IndependentSyntheticMap(SyntheticJointOdometry(2), 1);
  legacy.set_sequence(100);
  legacy.clear_graph_qualification();
  EXPECT_NE(Send(legacy).status(), 0);
  ASSERT_EQ(Send(test::Reset()).status(), 0);
  EXPECT_TRUE(worker->Sample(200).reset_pending());
  ASSERT_EQ(Send(SyntheticJointOdometry(3)).status(), 0);
  EXPECT_NE(Send(IndependentSyntheticMap(SyntheticJointOdometry(3), 2)).status(), 0);
  ASSERT_EQ(Send(test::ResetAck("planner")).status(), 0);
  EXPECT_FALSE(worker->Sample(200).target_graph_available());
  ASSERT_EQ(Send(test::ResetAck("observer")).status(), 0);
  for (int index = 4; index < 7; ++index) {
    const auto source = SyntheticJointOdometry(index);
    ASSERT_EQ(Send(source).status(), 0);
    Send(IndependentSyntheticMap(source, 2));
  }
  output = worker->Sample(200);
  ASSERT_TRUE(output.target_graph_available()) << output.reason();
  EXPECT_EQ(output.alignment_generation(), 2);
  EXPECT_EQ(output.generation(), 1);
  EXPECT_EQ(output.odometry_sequence(), 7);
  auto unavailable = SyntheticJointOdometry(7);
  unavailable.set_model_available(false);
  unavailable.set_model_detail("explicit_native_model_capacity_exceeded");
  EXPECT_NE(Send(unavailable).status(), 0);
  EXPECT_FALSE(worker->Sample(200).target_graph_available());
  EXPECT_NE(worker->Sample(200).reason().find("explicit_native_model_capacity_exceeded"),
            std::string::npos);
  EXPECT_NE(Send(IndependentSyntheticMap(unavailable, 2)).status(), 0);
  worker->Finish();
}

TEST_F(GlobalWorkerTest, SourceClockGraphRequiresRecordedProfileAndMatchingWireMode) {
  auto config = test::Config(Journal());
  test::EnableTargetGraph(&config);
  config.mutable_target_graph()->set_motion_error_model_id("synthetic-joint-reset");
  config.mutable_target_graph()->set_recorded_source_clock(true);
  config.set_recorded_input(false);
  EXPECT_THROW(NativeGlobalWorker{config}, std::invalid_argument);
  config.set_recorded_input(true);
  worker = std::make_unique<NativeGlobalWorker>(config);
  worker->AddGraphOdometry(SyntheticJointOdometry(0), Clock::now());
  EXPECT_THROW(worker->Finish(), std::invalid_argument);
}

TEST_F(GlobalWorkerTest, ResetRetiresAlignmentWaitsForEveryConsumerAndReacquires) {
  Start(true, false, true);
  Acquire(1);
  const auto before = worker->Sample(200);
  ASSERT_TRUE(before.usable());
  const auto source = test::Odometry(11);
  const auto original = source.SerializeAsString();
  EXPECT_EQ(Send(test::Reset()).status(), 0);
  auto output = worker->Sample(200);
  EXPECT_EQ(output.alignment_generation(), 2);
  EXPECT_EQ(output.reset_sequence(), 1);
  EXPECT_TRUE(output.reset_pending());
  EXPECT_EQ(output.pending_reset_consumers_size(), 2);
  EXPECT_EQ(output.reason(), "WAITING_FOR_RESET_CONSUMERS");
  EXPECT_FALSE(output.usable());
  EXPECT_FALSE(output.has_x());
  EXPECT_FALSE(output.has_map_from_imu());
  Send(test::Odometry(12));
  EXPECT_NE(Send(test::Alignment(test::Gnss(test::Odometry(12)), 2)).status(), 0);
  EXPECT_NE(Send(test::Alignment(test::Map(test::Odometry(12)), 2)).status(), 0);
  EXPECT_EQ(Send(test::ResetAck("planner")).status(), 0);
  EXPECT_EQ(Send(test::ResetAck("planner")).status(), 0);
  EXPECT_EQ(worker->Sample(200).pending_reset_consumers_size(), 1);
  EXPECT_NE(Send(test::ResetAck("unknown")).status(), 0);
  EXPECT_NE(Send(test::ResetAck("observer", 1, 1)).status(), 0);
  EXPECT_TRUE(worker->Sample(200).reset_pending());
  EXPECT_EQ(Send(test::ResetAck("observer")).status(), 0);
  output = worker->Sample(200);
  EXPECT_FALSE(output.reset_pending());
  EXPECT_FALSE(output.usable());
  EXPECT_NE(Send(test::Alignment(test::Gnss(test::Odometry(12)), 2)).status(), 0);
  for (int i = 13; i < 25; ++i) {
    Send(test::Odometry(i));
    EXPECT_NE(Send(test::Gnss(test::Odometry(i))).status(), 0);
    EXPECT_NE(Send(test::Alignment(test::Gnss(test::Odometry(i)), 1)).status(), 0);
    Send(test::Alignment(test::Gnss(test::Odometry(i)), 2));
  }
  output = worker->Sample(200);
  ASSERT_TRUE(output.usable()) << output.reason();
  EXPECT_EQ(output.alignment_generation(), 2);
  EXPECT_EQ(output.session(), before.session());
  EXPECT_EQ(output.generation(), before.generation());
  EXPECT_EQ(output.odometry_sequence(), 25);
  EXPECT_TRUE(output.new_measurement());
  EXPECT_FALSE(output.epoch_changed());
  EXPECT_NEAR(output.x(), 124, 1e-6);
  EXPECT_FALSE(output.trusted());
  EXPECT_FALSE(output.full_pose_available());
  EXPECT_EQ(source.SerializeAsString(), original);
  worker->Finish();
  std::ifstream journal(Journal());
  const std::string text((std::istreambuf_iterator<char>(journal)), {});
  EXPECT_NE(text.find("reset_consumer_retired:observer"), std::string::npos);
  EXPECT_NE(text.find("absolute_alignment_generation_mismatch"), std::string::npos);
}

TEST_F(GlobalWorkerTest, ResetRetriesCannotEraseNewEvidenceOrReleaseWrongSession) {
  Start(true, false, true);
  for (int i = 0; i < 3; ++i) {
    Send(test::Odometry(i));
    Send(test::Alignment(test::Map(test::Odometry(i)), 1));
  }
  ASSERT_TRUE(worker->Sample(200).full_pose_available());
  EXPECT_EQ(Send(test::Alignment(test::Map(test::Odometry(3)), 1)).status(),
            static_cast<int>(::openloc::localization::StatusCode::kPending));
  auto wrong = test::Reset();
  wrong.set_alignment_session("other-process");
  EXPECT_NE(Send(wrong).status(), 0);
  EXPECT_EQ(worker->Sample(200).alignment_generation(), 1);
  EXPECT_EQ(Send(test::Reset()).status(), 0);
  EXPECT_EQ(Send(test::Reset()).status(), 0);
  EXPECT_NE(Send(test::Reset(2, 2)).status(), 0);
  wrong = test::Reset();
  wrong.set_reason("conflicting-retry");
  EXPECT_NE(Send(wrong).status(), 0);
  auto ack = test::ResetAck("planner");
  ack.set_alignment_session("other-process");
  EXPECT_NE(Send(ack).status(), 0);
  EXPECT_EQ(worker->Sample(200).pending_reset_consumers_size(), 2);
  Send(test::ResetAck("planner"));
  Send(test::ResetAck("observer"));
  auto at_frontier = test::Alignment(test::Map(test::Odometry(2)), 2);
  EXPECT_NE(Send(at_frontier).status(), 0);
  for (int i = 3; i < 6; ++i) {
    Send(test::Odometry(i));
    if (i == 3) {
      EXPECT_FALSE(worker->Sample(200).full_pose_available());
    }
    Send(test::Alignment(test::Map(test::Odometry(i)), 2));
  }
  auto full = worker->Sample(200);
  ASSERT_TRUE(full.full_pose_available());
  EXPECT_EQ(Send(test::Reset()).status(), 0);
  EXPECT_EQ(Send(test::ResetAck("planner")).status(), 0);
  EXPECT_TRUE(worker->Sample(200).full_pose_available());
  EXPECT_EQ(Send(test::Reset(2, 2)).status(), 0);
  EXPECT_EQ(worker->Sample(200).alignment_generation(), 3);
  EXPECT_NE(Send(test::ResetAck("planner")).status(), 0);
  EXPECT_TRUE(worker->Sample(200).reset_pending());
  EXPECT_FALSE(worker->Sample(200).has_map_from_imu());
  worker->Finish();
}

TEST_F(GlobalWorkerTest, ResetClearsRawFragmentsAndRestartsReceiverDwell) {
  Start(false, true, true);
  for (int i = 0; i < 16; ++i) {
    Send(test::GgaOdometry(i));
    Send(NativeGlobalWorker::RawGgaPacket{
        test::Gga(i), (100LL + i) * 1000000000, static_cast<uint64_t>(i + 1)});
  }
  ASSERT_TRUE(worker->Sample(200).usable());
  Send(NativeGlobalWorker::RawGgaPacket{"$GNGGA,partial", 115000000000LL, 17});
  Send(test::Reset());
  Send(test::ResetAck("planner"));
  Send(test::GgaOdometry(16));
  EXPECT_NE(Send(NativeGlobalWorker::RawGgaPacket{
      test::Gga(16), 116000000000LL, 18}).status(), 0);
  Send(test::ResetAck("observer"));
  EXPECT_FALSE(worker->Sample(200).usable());
  for (int i = 17; i < 33; ++i) {
    Send(test::GgaOdometry(i));
    const auto outcome = Send(NativeGlobalWorker::RawGgaPacket{
        test::Gga(i), (100LL + i) * 1000000000, static_cast<uint64_t>(i + 2)});
    if (i < 19) {
      EXPECT_NE(outcome.status(), 0);
    }
  }
  const auto output = worker->Sample(200);
  ASSERT_TRUE(output.usable()) << output.reason();
  EXPECT_EQ(output.alignment_generation(), 2);
  EXPECT_EQ(output.generation(), 1);
  EXPECT_FALSE(output.full_pose_available());
  worker->Finish();
}

TEST_F(GlobalWorkerTest, ResetProfileRequiresExplicitBoundedConsumerSet) {
  auto config = test::Config(Journal());
  config.set_alignment_session("orphan");
  EXPECT_THROW(NativeGlobalWorker{config}, std::invalid_argument);
  config.clear_alignment_session();
  test::EnableReset(&config);
  config.add_reset_consumers("planner");
  EXPECT_THROW(NativeGlobalWorker{config}, std::invalid_argument);
  config.mutable_reset_consumers()->RemoveLast();
  config.set_reset_topic(config.odometry_topic());
  EXPECT_THROW(NativeGlobalWorker{config}, std::invalid_argument);
  Start();
  EXPECT_THROW(worker->AddReset(test::Reset(), Clock::now()), std::invalid_argument);
  EXPECT_THROW(worker->AddResetAck(test::ResetAck("planner"), Clock::now()),
               std::invalid_argument);
  worker->Finish();
}

TEST_F(GlobalWorkerTest, RawGgaAcquiresSingleOwnerAndRetiresOnOdometryEpoch) {
  Start(false, true);
  for (int i = 0; i < 16; ++i) {
    const auto source = test::GgaOdometry(i);
    const auto original = source.SerializeAsString();
    Send(source);
    const auto processed = Send(NativeGlobalWorker::RawGgaPacket{
        test::Gga(i), (100LL + i) * 1000000000, static_cast<uint64_t>(i + 1)});
    EXPECT_EQ(processed.kind(), "raw_gga");
    EXPECT_EQ(source.SerializeAsString(), original);
  }
  const auto output = worker->Sample(200);
  ASSERT_TRUE(output.usable()) << output.reason();
  EXPECT_GT(output.correction_id(), 0);
  EXPECT_NEAR(output.x(), 14.94, 0.1);
  EXPECT_FALSE(output.full_pose_available());
  EXPECT_FALSE(output.trusted());
  EXPECT_FALSE(output.target_graph_available());
  EXPECT_FALSE(output.has_map_from_imu());
  EXPECT_THROW(worker->AddGnss(test::Gnss(test::Odometry(17)), Clock::now()),
               std::invalid_argument);
  Send(test::GgaOdometry(16, 2));
  const auto reset = Send(NativeGlobalWorker::RawGgaPacket{
      test::Gga(16), 116000000000LL, 17});
  EXPECT_NE(reset.status(), 0);
  EXPECT_FALSE(worker->Sample(201).usable());
  worker->Finish();
  std::ifstream journal(Journal());
  const std::string text((std::istreambuf_iterator<char>(journal)), {});
  EXPECT_NE(text.find("\"raw-gga-horizontal\""), std::string::npos);
  EXPECT_NE(text.find("fixed_solution_not_stable"), std::string::npos);
}

TEST_F(GlobalWorkerTest, UnfinishedRawStreamFailsExplicitly) {
  Start(false, true);
  worker->AddRawGga({"$GNGGA,", 100000000000LL, 1}, Clock::now());
  EXPECT_THROW(worker->Finish(), std::runtime_error);
}

TEST_F(GlobalWorkerTest, RecordedRawPacketLossFailsInsteadOfJoiningFragments) {
  Start(false, true);
  Send(NativeGlobalWorker::RawGgaPacket{"$GNGGA,", 100000000000LL, 1});
  worker->AddRawGga({test::Gga(1), 101000000000LL, 3}, Clock::now());
  EXPECT_THROW(worker->Finish(), std::invalid_argument);
}

TEST_F(GlobalWorkerTest, AcquiresTracksHoldsAndExpiresWithoutFullPose) {
  Start();
  EXPECT_FALSE(worker->Sample(200).usable());
  Acquire();
  const auto source = test::Odometry(11);
  const auto first = worker->Sample(200);
  ASSERT_TRUE(first.usable()) << first.reason();
  EXPECT_NEAR(first.x(), 111, 1e-6);
  EXPECT_NEAR(first.y(), 20, 1e-6);
  EXPECT_GT(first.correction_id(), 0);
  EXPECT_TRUE(first.new_measurement());
  EXPECT_FALSE(first.trusted());
  EXPECT_FALSE(first.full_pose_available());
  EXPECT_FALSE(first.target_graph_available());
  auto held = source;
  held.set_publication_sequence(13);
  held.set_new_measurement(false);
  Send(held);
  const auto second = worker->Sample(201);
  EXPECT_TRUE(second.usable());
  EXPECT_FALSE(second.new_measurement());
  EXPECT_EQ(first.measurement_ns(), second.measurement_ns());
  const auto stale = worker->Sample(
      202, Clock::now() + std::chrono::milliseconds(210));
  EXPECT_FALSE(stale.usable());
  EXPECT_EQ(stale.reason(), "LOCAL_ODOMETRY_STALE");
  EXPECT_FALSE(stale.has_x());
  worker->Finish();
}

TEST_F(GlobalWorkerTest, HeldNativeErrorMustRetainTheExactSourceSnapshot) {
  Start();
  NativePublication publication(0.2);
  NativePublication::State state;
  state.stamp = {100, 100, 1, "unix"};
  state.epoch = {"native-error-test", 1};
  state.valid = true;
  state.covariance.setIdentity();
  state.covariance_model_valid = false;
  state.uncertainty_qualified = false;
  const auto arrival = Clock::now();
  publication.Update(state, {}, arrival);
  auto first = publication.Sample(101, arrival);
  first.set_recorded_input(true);
  EXPECT_EQ(Send(first).status(), 0);
  auto held = publication.Sample(102, arrival);
  held.set_recorded_input(true);
  EXPECT_EQ(Send(held).status(), 0);
  EXPECT_FALSE(worker->Sample(200).target_graph_available());
  held.set_publication_sequence(3);
  held.mutable_native_error_state()->set_covariance(0, 2);
  worker->AddOdometry(held, Clock::now());
  EXPECT_THROW(worker->Finish(), std::invalid_argument);
  EXPECT_THROW(worker->Sample(200), std::invalid_argument);
}

TEST_F(GlobalWorkerTest, MalformedOptionalNativeErrorFailsWithoutLegacyFallback) {
  NativePublication publication(0.2);
  NativePublication::State state;
  state.stamp = {100, 100, 1, "unix"};
  state.epoch = {"native-error-test", 1};
  state.valid = true;
  state.covariance.setIdentity();
  const auto arrival = Clock::now();
  publication.Update(state, {}, arrival);
  const auto original = publication.Sample(101, arrival);
  for (int kind = 0; kind < 5; ++kind) {
    auto source = original;
    source.set_recorded_input(true);
    auto* error = source.mutable_native_error_state();
    if (kind == 0) error->clear_covariance_model_valid();
    else if (kind == 1) error->mutable_covariance()->RemoveLast();
    else if (kind == 2)
      error->set_covariance(0, std::numeric_limits<double>::quiet_NaN());
    else if (kind == 3) error->mutable_gyro_bias()->clear_x();
    else source.set_usable(false);
    NativeGlobalWorker candidate(
        test::Config(Journal() + "." + std::to_string(kind)));
    candidate.AddOdometry(source, Clock::now());
    EXPECT_THROW(candidate.Finish(), std::invalid_argument);
  }
}

TEST_F(GlobalWorkerTest, MissingRejectedDuplicateAndRetiredEpochAreExplicit) {
  Start();
  const auto early = Send(test::Gnss(test::Odometry(0)));
  EXPECT_EQ(early.status(),
            static_cast<int>(::openloc::localization::StatusCode::kRejectedMeasurement));
  Acquire();
  ASSERT_TRUE(worker->Sample(200).usable());
  auto invalid = test::Gnss(test::Odometry(12));
  invalid.set_reference_id("wrong-site");
  EXPECT_EQ(Send(invalid).status(),
            static_cast<int>(::openloc::localization::StatusCode::kMapReferenceMismatch));
  invalid = test::Gnss(test::Odometry(12));
  invalid.set_receiver_admitted(false);
  EXPECT_NE(Send(invalid).status(), 0);
  EXPECT_NE(Send(test::Gnss(test::Odometry(11))).status(), 0);
  auto reset = test::Odometry(12, 2);
  Send(reset);
  const auto waiting = worker->Sample(200);
  EXPECT_FALSE(waiting.usable());
  EXPECT_EQ(waiting.generation(), 2);
  EXPECT_FALSE(waiting.has_x());
  EXPECT_NE(Send(test::Gnss(test::Odometry(12))).status(), 0);
  Send(test::Gnss(reset));
  for (int i = 13; i < 24; ++i) {
    const auto next = test::Odometry(i, 2);
    Send(next);
    Send(test::Gnss(next));
  }
  const auto reacquired = worker->Sample(200);
  ASSERT_TRUE(reacquired.usable()) << reacquired.reason();
  EXPECT_TRUE(reacquired.epoch_changed());
  EXPECT_EQ(reacquired.generation(), 2);
  worker->Finish();
  std::ifstream journal(Journal());
  const std::string text((std::istreambuf_iterator<char>(journal)), {});
  EXPECT_NE(text.find("owner,"), std::string::npos);
  EXPECT_NE(text.find("GNSS reference mismatch"), std::string::npos);
}

TEST_F(GlobalWorkerTest, UnavailableLocalCannotBeRevivedByGnssOrHeldOutput) {
  Start();
  Acquire();
  auto unavailable = test::Odometry(12);
  unavailable.set_usable(false);
  unavailable.clear_odom_from_imu();
  unavailable.clear_native_error_state();
  Send(unavailable);
  EXPECT_FALSE(worker->Sample(200).usable());
  EXPECT_NE(Send(test::Gnss(test::Odometry(12))).status(), 0);
  EXPECT_FALSE(worker->Sample(200).usable());
  auto held = test::Odometry(11);
  held.set_publication_sequence(14);
  held.set_new_measurement(false);
  Send(held);
  EXPECT_FALSE(worker->Sample(200).usable());
  worker->Finish();
}

TEST_F(GlobalWorkerTest, FreshLocalDoesNotRenewOldGnssEvidence) {
  Start();
  Acquire();
  auto fresh = test::Odometry(42);
  Send(fresh);
  const auto output = worker->Sample(200);
  EXPECT_EQ(output.reason(), "GNSS_HORIZONTAL_STALE");
  EXPECT_FALSE(output.usable());
  EXPECT_FALSE(output.has_x());
  worker->Finish();
}

TEST_F(GlobalWorkerTest, MalformedSourceAndWorkerErrorsAreNeverSwallowed) {
  Start();
  auto source = test::Odometry(0);
  source.mutable_odom_from_imu()->set_child_frame_id("wrong");
  worker->AddOdometry(source, Clock::now());
  EXPECT_THROW(worker->Finish(), std::invalid_argument);
  EXPECT_THROW(worker->Sample(200), std::invalid_argument);
}

TEST_F(GlobalWorkerTest, QueueOverflowFailsInsteadOfDroppingOrBlockingOdometry) {
  bool entered = false;
  bool release = false;
  NativeGlobalWorker blocked(test::Config(Journal()),
      [&](const NativeGlobalProcessed&) {
        std::unique_lock<std::mutex> lock(mutex);
        entered = true;
        condition.notify_all();
        condition.wait(lock, [&] { return release; });
      });
  blocked.AddOdometry(test::Odometry(0), Clock::now());
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5),
                                  [&] { return entered; }));
  }
  for (int i = 1; i < 256; ++i)
    blocked.AddOdometry(test::Odometry(i), Clock::now());
  EXPECT_THROW(blocked.AddOdometry(test::Odometry(256), Clock::now()),
               std::runtime_error);
  {
    std::lock_guard<std::mutex> lock(mutex);
    release = true;
  }
  condition.notify_all();
  EXPECT_THROW(blocked.Finish(), std::runtime_error);
}

TEST_F(GlobalWorkerTest, ExplicitDiagnosticAssumptionAndSeparateTopicsRequired) {
  auto config = test::Config(Journal());
  config.set_assume_gravity_aligned_odom(false);
  EXPECT_THROW(NativeGlobalWorker{config}, std::invalid_argument);
  config.set_assume_gravity_aligned_odom(true);
  config.set_output_topic("/apollo/localization/pose");
  EXPECT_THROW(NativeGlobalWorker{config}, std::invalid_argument);
}

TEST_F(GlobalWorkerTest, MapHandoverAndGnssShareRevisionWithoutTouchingOdometry) {
  Start(true);
  Acquire();
  const auto horizontal = worker->Sample(200);
  ASSERT_TRUE(horizontal.usable());
  EXPECT_FALSE(horizontal.full_pose_available());
  for (int i = 12; i < 15; ++i) {
    const auto source = test::Odometry(i);
    const auto bytes = source.SerializeAsString();
    Send(source);
    const auto processed = Send(test::Map(source));
    EXPECT_EQ(processed.kind(), "map");
    EXPECT_EQ(processed.status() == 0, i == 14);
    EXPECT_EQ(source.SerializeAsString(), bytes);
  }
  const auto full = worker->Sample(200);
  ASSERT_TRUE(full.usable()) << full.reason();
  ASSERT_TRUE(full.full_pose_available());
  ASSERT_TRUE(full.has_map_from_imu());
  EXPECT_NEAR(full.map_from_imu().x(), 114, 1e-6);
  EXPECT_NEAR(full.map_from_imu().z(), 3, 1e-6);
  EXPECT_NEAR(full.map_from_odom().x(), 100, 1e-6);
  EXPECT_EQ(full.correction_id(), horizontal.correction_id() + 1);
  EXPECT_FALSE(full.has_alignment_yaw());
  EXPECT_FALSE(full.trusted());
  EXPECT_FALSE(full.target_graph_available());
  const auto source = test::Odometry(15);
  Send(source);
  EXPECT_EQ(Send(test::Gnss(source)).status(), 0);
  const auto fused = worker->Sample(200);
  EXPECT_EQ(fused.correction_id(), full.correction_id() + 1);
  EXPECT_EQ(fused.last_full_pose_measurement_ns(),
            full.last_full_pose_measurement_ns());
  EXPECT_FALSE(worker->Sample(201).new_measurement());
  worker->Finish();
  std::ifstream journal(Journal());
  const std::string text((std::istreambuf_iterator<char>(journal)), {});
  EXPECT_NE(text.find("\"prepared-map\""), std::string::npos);
  EXPECT_NE(text.find("\"prepared-gnss-xy\""), std::string::npos);
}

TEST_F(GlobalWorkerTest, MapOnlyBootstrapsButPartialEvidenceCannotRenewFullPose) {
  Start(true);
  for (int i = 0; i < 3; ++i) {
    const auto source = test::Odometry(i);
    Send(source);
    Send(test::Map(source));
  }
  const auto full = worker->Sample(200);
  ASSERT_TRUE(full.full_pose_available());
  auto source = test::Odometry(3);
  auto projected = test::Map(source, true);
  projected.mutable_map_from_imu()->set_y(5000);
  EXPECT_EQ(Send(projected).status(),
            static_cast<int>(::openloc::localization::StatusCode::kPending));
  EXPECT_EQ(worker->Sample(200).correction_id(), full.correction_id());
  Send(source);
  const auto partial_update = worker->Sample(200);
  EXPECT_EQ(partial_update.correction_id(), full.correction_id() + 1);
  EXPECT_NEAR(partial_update.map_from_imu().y(), 20, 1e-6);
  EXPECT_EQ(partial_update.last_full_pose_measurement_ns(),
            full.last_full_pose_measurement_ns());
  source = test::Odometry(24);
  Send(source);
  EXPECT_EQ(Send(test::Gnss(source)).status(), 0);
  const auto stale = worker->Sample(200);
  EXPECT_FALSE(stale.usable());
  EXPECT_FALSE(stale.full_pose_available());
  EXPECT_FALSE(stale.has_map_from_imu());
  EXPECT_FALSE(stale.has_x());
  EXPECT_EQ(stale.reason(), "FULL_POSE_STALE");
  worker->Finish();
}

TEST_F(GlobalWorkerTest, MapRejectsMalformedAmbiguousAndRetiredInputsExplicitly) {
  Start(true);
  EXPECT_NE(Send(test::Map(test::Odometry(0))).status(), 0);
  Send(test::Odometry(0));
  auto map = test::Map(test::Odometry(0));
  map.set_ambiguous(true);
  EXPECT_NE(Send(map).status(), 0);
  EXPECT_FALSE(worker->Sample(200).usable());
  map = test::Map(test::Odometry(0));
  map.set_covariance(0, -1);
  EXPECT_NE(Send(map).status(), 0);
  map = test::Map(test::Odometry(0));
  map.clear_covariance();
  EXPECT_EQ(Send(map).status(),
            static_cast<int>(::openloc::localization::StatusCode::kInvalidArgument));
  map = test::Map(test::Odometry(0), true);
  map.set_rank(6);
  EXPECT_NE(Send(map).status(), 0);
  map = test::Map(test::Odometry(0));
  map.set_content_id("wrong-map");
  EXPECT_EQ(Send(map).status(),
            static_cast<int>(::openloc::localization::StatusCode::kMapReferenceMismatch));
  for (int i = 1; i < 4; ++i) {
    auto source = test::Odometry(i);
    Send(source);
    Send(test::Map(source));
  }
  ASSERT_TRUE(worker->Sample(200).full_pose_available());
  EXPECT_NE(Send(test::Map(test::Odometry(3))).status(), 0);
  Send(test::Odometry(4, 2));
  EXPECT_NE(Send(test::Map(test::Odometry(4))).status(), 0);
  const auto reset = worker->Sample(200);
  EXPECT_FALSE(reset.usable());
  EXPECT_FALSE(reset.has_map_from_imu());
  EXPECT_EQ(reset.generation(), 2);
  worker->Finish();
}

TEST_F(GlobalWorkerTest, ProjectedMapCannotInventFullPoseAndMapIsOptIn) {
  Start();
  EXPECT_THROW(worker->AddMap(test::Map(test::Odometry(0)), Clock::now()),
               std::invalid_argument);
  worker->Finish();
  auto config = test::Config(Journal() + ".enabled");
  config.set_map_topic(config.gnss_topic());
  EXPECT_THROW(NativeGlobalWorker{config}, std::invalid_argument);
  config.set_map_topic("/openloc/diagnostic/global-test/map");
  NativeGlobalWorker projected_only(config);
  for (int i = 0; i < 4; ++i) {
    const auto source = test::Odometry(i);
    projected_only.AddOdometry(source, Clock::now());
    projected_only.AddMap(test::Map(source, true), Clock::now());
  }
  projected_only.Finish();
  EXPECT_FALSE(projected_only.Sample(200).full_pose_available());
  EXPECT_FALSE(projected_only.Sample(200).has_map_from_imu());
}

}  // namespace
}  // namespace apollo::localization::openloc_diagnostic
