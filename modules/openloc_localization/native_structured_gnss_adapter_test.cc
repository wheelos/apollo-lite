// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <unistd.h>

#include <fstream>
#include <limits>

#include "gtest/gtest.h"
#include "native_global_test_input.h"

namespace apollo::localization::openloc_diagnostic {
namespace {
namespace loc = ::openloc::localization;
int64_t Receive(int index) { return (100LL + index) * 1000000000 + 1000000; }

TEST(StructuredGnssAdapterTest, FixedDwellProjectsReceiverUncertaintyWithoutInventingHeightOrHdop) {
  auto config = test::Config("");
  test::EnableStructuredGnss(&config);
  NativeStructuredGnssAdapter adapter(config);
  adapter.Reset("vehicle-test", 1);
  EXPECT_FALSE(adapter.Position(test::BestPose(0), Receive(0)).observation);
  EXPECT_FALSE(adapter.Position(test::BestPose(1), Receive(1)).observation);
  const auto admitted = adapter.Position(test::BestPose(2), Receive(2));
  ASSERT_TRUE(admitted.status.ok()) << admitted.status.message();
  ASSERT_TRUE(admitted.observation);
  const auto& input = *admitted.observation;
  EXPECT_NEAR(input.x(), 2.0, 0.01);
  EXPECT_NEAR(input.y(), 0, 0.01);
  EXPECT_NEAR(input.covariance_xx(), 0.03 * 0.03, 1e-6);
  EXPECT_NEAR(input.covariance_yy(), 0.02 * 0.02, 1e-6);
  EXPECT_EQ(input.measurement_ns(), 102000000000LL);
  EXPECT_EQ(input.receive_ns(), Receive(2));
  EXPECT_TRUE(input.diagnostic_only());
  EXPECT_EQ(input.covariance_source(), "receiver-enu-diagonal-projected-uncalibrated");
}

TEST(StructuredGnssAdapterTest, BadFixAndMissingAccuracyResetDwellBeforeRecovery) {
  auto config = test::Config("");
  test::EnableStructuredGnss(&config);
  NativeStructuredGnssAdapter adapter(config);
  adapter.Reset("vehicle-test", 1);
  adapter.Position(test::BestPose(0), Receive(0));
  adapter.Position(test::BestPose(1), Receive(1));
  auto bad = test::BestPose(2);
  bad.set_sol_type(drivers::gnss::PSRDIFF);
  EXPECT_FALSE(adapter.Position(bad, Receive(2)).status.ok());
  EXPECT_FALSE(adapter.Position(test::BestPose(3), Receive(3)).observation);
  EXPECT_FALSE(adapter.Position(test::BestPose(4), Receive(4)).observation);
  EXPECT_TRUE(adapter.Position(test::BestPose(5), Receive(5)).observation);
  auto missing = test::BestPose(6);
  missing.clear_latitude_std_dev();
  EXPECT_EQ(adapter.Position(missing, Receive(6)).status.message(), "incomplete_gnss_best_pose");
  EXPECT_FALSE(adapter.Position(test::BestPose(7), Receive(7)).observation);
}

TEST(StructuredGnssAdapterTest, ExplicitClockRejectsStaleWrongEpochAndMissingMeasurement) {
  auto config = test::Config("");
  test::EnableStructuredGnss(&config);
  config.mutable_structured_gnss()->set_measurement_to_unix_offset_s(315964782);
  NativeStructuredGnssAdapter adapter(config);
  adapter.Reset("vehicle-test", 1);
  const auto gps = adapter.Position(test::BestPose(0), Receive(0) + 315964782000000000LL);
  EXPECT_EQ(gps.measurement_ns, 315964882000000000LL);
  EXPECT_NE(gps.status.code(), loc::StatusCode::kInvalidArgument);
  EXPECT_EQ(adapter.Position(test::BestPose(1), Receive(1)).status.message(),
            "structured_gnss_stale_or_future_measurement");
  auto missing = test::BestPose(2);
  missing.clear_measurement_time();
  EXPECT_EQ(adapter.Position(missing, Receive(2)).status.message(), "incomplete_gnss_best_pose");
}

TEST(StructuredGnssAdapterTest, HeadingUsesBaselineAzimuthAndRejectsBadLengthAndDuplicates) {
  auto config = test::Config("");
  test::EnableStructuredGnss(&config);
  NativeStructuredGnssAdapter adapter(config);
  adapter.Reset("vehicle-test", 1);
  EXPECT_FALSE(adapter.Heading(test::DualHeading(0), Receive(0)).status.ok());
  EXPECT_FALSE(adapter.Heading(test::DualHeading(1), Receive(1)).status.ok());
  const auto heading = adapter.Heading(test::DualHeading(2), Receive(2));
  ASSERT_TRUE(heading.status.ok()) << heading.status.message();
  EXPECT_NEAR(heading.baseline_yaw_rad, 0, 1e-12);
  EXPECT_NEAR(std::sqrt(heading.variance_rad2), 0.5 * 3.141592653589793 / 180, 1e-12);
  EXPECT_EQ(adapter.Heading(test::DualHeading(2), Receive(2)).status.message(),
            "out_of_order_dual_antenna_heading");
  auto wrong = test::DualHeading(3);
  wrong.set_baseline_length(2);
  EXPECT_FALSE(adapter.Heading(wrong, Receive(3)).status.ok());
  EXPECT_FALSE(adapter.Heading(test::DualHeading(4), Receive(4)).status.ok());
  adapter.Reset("vehicle-test", 2, 105000000000LL);
  EXPECT_EQ(adapter.Heading(test::DualHeading(5), Receive(5)).status.message(),
            "structured_gnss_precedes_reset_frontier");
}

TEST(StructuredGnssAdapterTest, MissingFieldsInvalidProfileAndInsDerivedFixAreNotQualified) {
  auto config = test::Config("");
  test::EnableStructuredGnss(&config);
  auto invalid = config;
  invalid.mutable_structured_gnss()->mutable_heading()->set_baseline_x(0);
  EXPECT_THROW(NativeStructuredGnssAdapter{invalid}, std::invalid_argument);
  NativeStructuredGnssAdapter adapter(config);
  adapter.Reset("vehicle-test", 1);
  auto input = test::BestPose(0);
  input.set_sol_type(drivers::gnss::INS_RTKFIXED);
  EXPECT_FALSE(adapter.Position(input, Receive(0)).status.ok());
  input = test::BestPose(1);
  input.set_longitude_std_dev(std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(adapter.Position(input, Receive(1)).status.ok());
}

TEST(StructuredGnssAdapterTest, WorkerAcquiresHorizontalWithoutFullPoseAndJournalsSourceDecisions) {
  const auto journal = ::testing::TempDir() + "structured-worker-" +
      std::to_string(getpid()) + ".csv";
  auto config = test::Config(journal);
  test::EnableStructuredGnss(&config);
  std::mutex mutex;
  std::condition_variable condition;
  uint64_t completed = 0;
  NativeGlobalWorker worker(config, [&](const auto&) {
    std::lock_guard<std::mutex> lock(mutex);
    ++completed;
    condition.notify_all();
  });
  const auto send = [&](const auto& operation) {
    uint64_t target;
    {
      std::lock_guard<std::mutex> lock(mutex);
      target = completed + 1;
    }
    operation();
    std::unique_lock<std::mutex> lock(mutex);
    const bool ready = condition.wait_for(lock, std::chrono::seconds(5),
        [&] { return completed >= target; });
    lock.unlock();
    worker.Check();
    if (!ready) throw std::runtime_error("structured worker input completion timeout");
  };
  for (int i = 0; i < 5; ++i) {
    send([&] { worker.AddOdometry(test::GgaOdometry(i), NativeGlobalWorker::Clock::now()); });
    send([&] { worker.AddHeading({test::DualHeading(i), Receive(i)}, NativeGlobalWorker::Clock::now()); });
    send([&] { worker.AddBestPose({test::BestPose(i), Receive(i)}, NativeGlobalWorker::Clock::now()); });
  }
  auto output = worker.Sample(200);
  ASSERT_TRUE(output.usable()) << output.reason();
  EXPECT_GT(output.correction_id(), 0);
  EXPECT_EQ(output.reason(), "DIAGNOSTIC_HORIZONTAL_ONLY");
  EXPECT_FALSE(output.full_pose_available());
  EXPECT_FALSE(output.has_map_from_imu());
  EXPECT_FALSE(output.trusted());
  EXPECT_THROW(worker.AddGnss(test::Gnss(test::Odometry(0)), NativeGlobalWorker::Clock::now()),
               std::invalid_argument);
  send([&] { worker.AddOdometry(test::GgaOdometry(8), NativeGlobalWorker::Clock::now()); });
  EXPECT_FALSE(worker.Sample(200).usable());
  for (int i = 9; i <= 12; ++i) {
    send([&] { worker.AddOdometry(test::GgaOdometry(i), NativeGlobalWorker::Clock::now()); });
    send([&] { worker.AddHeading({test::DualHeading(i), Receive(i)}, NativeGlobalWorker::Clock::now()); });
    auto position = test::BestPose(i);
    if (i == 9) position.set_sol_type(drivers::gnss::PSRDIFF);
    send([&] { worker.AddBestPose({position, Receive(i)}, NativeGlobalWorker::Clock::now()); });
  }
  const auto recovered = worker.Sample(200);
  EXPECT_TRUE(recovered.usable()) << recovered.reason();
  EXPECT_GT(recovered.correction_id(), output.correction_id());
  EXPECT_EQ(recovered.generation(), output.generation());
  EXPECT_FALSE(recovered.full_pose_available());
  worker.Finish();
  std::ifstream file(journal);
  const std::string ledger((std::istreambuf_iterator<char>(file)), {});
  EXPECT_NE(ledger.find("best_pose_gate,"), std::string::npos);
  EXPECT_NE(ledger.find("heading_gate,"), std::string::npos);
  EXPECT_NE(ledger.find("novatel-dual-antenna-heading"), std::string::npos);
}
}  // namespace
}  // namespace apollo::localization::openloc_diagnostic
