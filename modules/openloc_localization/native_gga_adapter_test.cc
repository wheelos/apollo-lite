// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "gtest/gtest.h"
#include "native_global_test_input.h"

namespace apollo::localization::openloc_diagnostic {
namespace {
namespace loc = ::openloc::localization;

class GgaAdapterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto config = test::Config("");
    test::EnableRawGga(&config);
    config.mutable_raw_gga()->set_lever_x(0.1);
    config.mutable_raw_gga()->set_lever_y(-0.2);
    config.mutable_raw_gga()->set_lever_z(0.3);
    adapter = std::make_unique<NativeGgaAdapter>(config);
    adapter->Reset("test-session", 1);
  }
  void Feed(const std::string& bytes, int64_t receive) {
    adapter->Feed(bytes, receive,
        [&](const NativeGgaDecision& decision) { decisions.push_back(decision); });
  }
  void Gga(int i, const std::string& quality = "4") {
    Feed(test::Gga(i, quality), (100LL + i) * 1000000000);
  }
  std::unique_ptr<NativeGgaAdapter> adapter;
  std::vector<NativeGgaDecision> decisions;
};

TEST_F(GgaAdapterTest, FragmentsMultipleSentencesGateProjectionAndLeverContract) {
  const auto sentence = test::Gga(0);
  Feed(sentence.substr(0, 20), 100000000000LL);
  EXPECT_TRUE(decisions.empty());
  Feed(sentence.substr(20) + test::Nmea("GNRMC,ignored"), 100000000000LL);
  ASSERT_EQ(decisions.size(), 1);
  EXPECT_EQ(decisions.back().status.message(), "fixed_solution_not_stable");
  Feed(test::Gga(1) + test::Gga(2), 102000000000LL);
  ASSERT_EQ(decisions.size(), 3);
  ASSERT_TRUE(decisions.back().observation) << decisions.back().status.message();
  const auto& input = *decisions.back().observation;
  EXPECT_EQ(input.measurement_ns(), 102000000000LL);
  EXPECT_EQ(input.receive_ns(), 102000000000LL);
  EXPECT_EQ(input.session(), "test-session");
  EXPECT_EQ(input.generation(), 1);
  EXPECT_TRUE(input.diagnostic_only());
  EXPECT_EQ(input.gnss_map_construction(), NativeGlobalGnssInput::UNKNOWN);
  EXPECT_NEAR(input.x(), 1.991, 0.003);
  EXPECT_NEAR(input.y(), 0, 0.003);
  EXPECT_EQ(input.covariance_xx(), 1);
  EXPECT_EQ(input.covariance_source(), "explicit-uncalibrated-gga-sigma");
  EXPECT_DOUBLE_EQ(input.lever_x(), 0.1);
  EXPECT_DOUBLE_EQ(input.lever_y(), -0.2);
  EXPECT_DOUBLE_EQ(input.lever_z(), 0.3);
  EXPECT_NO_THROW(adapter->Finish());
}

TEST_F(GgaAdapterTest, ChecksumAndMissingQualityResetDwellWithoutFakeAccuracy) {
  Gga(0);
  Gga(1);
  auto corrupt = test::Gga(2);
  corrupt[corrupt.size() - 3] = corrupt[corrupt.size() - 3] == '0' ? '1' : '0';
  Feed(corrupt, 102000000000LL);
  EXPECT_EQ(decisions.back().status.message(), "nmea_checksum_mismatch");
  Gga(3);
  EXPECT_FALSE(decisions.back().observation);
  Feed(test::Nmea(
      "GNGGA,000144.000,4000,N,11700,E,4,12,0.8,30,M,0,M,,0001"),
      104000000000LL);
  EXPECT_EQ(decisions.back().status.code(), loc::StatusCode::kInvalidArgument);
  Gga(5);
  Gga(6);
  EXPECT_FALSE(decisions.back().observation);
  Gga(7);
  EXPECT_TRUE(decisions.back().observation);
}

TEST_F(GgaAdapterTest, FloatDuplicateGapAndEpochRequireNewStability) {
  Gga(0);
  Gga(1, "5");
  EXPECT_EQ(decisions.back().status.message(), "carrier_solution_not_fixed");
  Gga(2);
  Gga(3);
  Gga(4);
  ASSERT_TRUE(decisions.back().observation);
  Gga(4);
  EXPECT_EQ(decisions.back().status.code(), loc::StatusCode::kDuplicateTimestamp);
  Gga(8);
  EXPECT_FALSE(decisions.back().observation);
  adapter->Reset("test-session", 2);
  Gga(9);
  Gga(10);
  EXPECT_FALSE(decisions.back().observation);
  Gga(11);
  ASSERT_TRUE(decisions.back().observation);
  EXPECT_EQ(decisions.back().observation->generation(), 2);
}

TEST_F(GgaAdapterTest, ResetFrontierCannotContributeOldDwellOrObservation) {
  Gga(0);
  Gga(1);
  Gga(2);
  ASSERT_TRUE(decisions.back().observation);
  adapter->Reset("test-session", 1, 102000000000LL);
  Gga(2);
  EXPECT_EQ(decisions.back().status.message(),
            "gga_measurement_precedes_reset_frontier");
  Gga(3);
  Gga(4);
  EXPECT_FALSE(decisions.back().observation);
  Gga(5);
  EXPECT_TRUE(decisions.back().observation);
}

TEST_F(GgaAdapterTest, UtcMidnightKeepsOriginalNanosecondsRejectsStaleAndLeap) {
  constexpr int64_t day = 86400000000000LL;
  Feed(test::Nmea(
      "GNGGA,235959.999999999,4000,N,11700,E,4,12,0.8,30,M,0,M,0.5,0001"),
      day + 100000000);
  ASSERT_EQ(decisions.size(), 1);
  EXPECT_EQ(decisions.back().measurement_ns, day - 1);
  Feed(test::Nmea(
      "GNGGA,000000.000000001,4000,N,11700,E,4,12,0.8,30,M,0,M,0.5,0001"),
      day + 200000000);
  EXPECT_EQ(decisions.back().measurement_ns, day + 1);
  Feed(test::Nmea(
      "GNGGA,235960.000,4000,N,11700,E,4,12,0.8,30,M,0,M,0.5,0001"),
      day + 200000000);
  EXPECT_EQ(decisions.back().status.message(), "invalid_gga_utc_time");
  Feed(test::Gga(0), day + 104000000000LL);
  EXPECT_EQ(decisions.back().status.message(), "gga_stale_or_future_measurement");
}

TEST_F(GgaAdapterTest, InvalidGeometryFramingBoundsAndConfigFailExplicitly) {
  Feed(test::Nmea(
      "GNGGA,000140,4060,N,11700,E,4,12,0.8,30,M,0,M,0.5,0001"),
      100000000000LL);
  EXPECT_EQ(decisions.back().status.message(), "invalid_gga_coordinate");
  Feed(test::Nmea(
      "GNGGA,000140,4000,N,12000.0001,E,4,12,0.8,30,M,0,M,0.5,0001"),
      100000000000LL);
  EXPECT_EQ(decisions.back().status.message(), "gga_outside_declared_utm_zone");
  Feed("$GNGGA,partial", 100000000000LL);
  Feed(test::Gga(1), 101000000000LL);
  ASSERT_GE(decisions.size(), 4);
  EXPECT_EQ(decisions[2].status.message(), "truncated_nmea_sentence");
  EXPECT_THROW(Feed("$" + std::string(256, 'X'), 101000000000LL),
               std::runtime_error);
  EXPECT_THROW(adapter->Finish(), std::runtime_error);
  auto config = test::Config("");
  test::EnableRawGga(&config);
  config.mutable_raw_gga()->set_utm_zone(0);
  EXPECT_THROW(NativeGgaAdapter{config}, std::invalid_argument);
}

}  // namespace
}  // namespace apollo::localization::openloc_diagnostic
