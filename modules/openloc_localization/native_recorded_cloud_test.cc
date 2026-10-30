// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_recorded_cloud.h"

#include <unistd.h>

#include <future>

#include "gtest/gtest.h"

namespace apollo::localization::openloc_diagnostic::recorded {
namespace {

class RecordedCloudTest : public ::testing::Test {
 protected:
  void SetUp() override {
    directory_ = std::filesystem::temp_directory_path() /
                 ("openloc_cloud_" + std::to_string(getpid()));
    ASSERT_TRUE(std::filesystem::create_directory(directory_));
  }
  void TearDown() override {
    std::filesystem::remove(directory_ / "points.csv");
    std::filesystem::remove(directory_);
  }
  std::filesystem::path directory_;
  const std::vector<std::string> source_ = {
      "1700000000000000002", "1700000000001000000", "points.csv"};
};

TEST_F(RecordedCloudTest, AsyncDecodingPreservesOriginalPayload) {
  std::ofstream(directory_ / "points.csv")
      << "timestamp_ns,x,y,z\n"
         "1700000000000000001,0,0,0\n"
         "1700000000000000002,1.25,-2.5,3\n";
  const auto decode = [&] {
    return ReadCloud(directory_, source_, "livox_frame", 7);
  };
  auto pending = std::async(std::launch::async, decode);
  const auto synchronous = decode();
  const auto cloud = pending.get();
  EXPECT_EQ(cloud.SerializeAsString(), synchronous.SerializeAsString());
  EXPECT_EQ(cloud.header().sequence_num(), 7);
  EXPECT_DOUBLE_EQ(cloud.header().timestamp_sec(), Time(source_[1]) / 1e9);
  EXPECT_DOUBLE_EQ(cloud.measurement_time(), Time(source_[0]) / 1e9);
  ASSERT_EQ(cloud.point_size(), 2);
  EXPECT_EQ(cloud.point(0).timestamp(), 1700000000000000001ULL);
  EXPECT_EQ(cloud.point(1).timestamp(), 1700000000000000002ULL);
  EXPECT_FLOAT_EQ(cloud.point(0).x(), 0);
  EXPECT_FLOAT_EQ(cloud.point(1).x(), 1.25);
  EXPECT_FLOAT_EQ(cloud.point(1).y(), -2.5);
  EXPECT_FLOAT_EQ(cloud.point(1).z(), 3);
}

TEST_F(RecordedCloudTest, DecodeFailureSurfacesThroughFuture) {
  for (const auto& contents :
       {"timestamp_ns,x,y,z\n1,nan,0,0\n", "wrong_header\n1,0,0,0\n"}) {
    std::ofstream(directory_ / "points.csv") << contents;
    auto pending = std::async(std::launch::async, [&] {
      return ReadCloud(directory_, source_, "livox_frame", 1);
    });
    EXPECT_THROW(pending.get(), std::runtime_error);
  }
  for (const auto& path : {"/points.csv", "../points.csv"}) {
    auto source = source_;
    source[2] = path;
    EXPECT_THROW(ReadCloud(directory_, source, "livox_frame", 1),
                 std::runtime_error);
  }
}

}  // namespace
}  // namespace apollo::localization::openloc_diagnostic::recorded
