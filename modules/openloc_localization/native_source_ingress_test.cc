// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_source_ingress.h"

#include <filesystem>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

namespace apollo::localization::openloc_diagnostic {
namespace {
using Stream = NativeSourceIngress::Stream;

class IngressTest : public ::testing::Test {
 protected:
  void SetUp() override {
    path = ::testing::TempDir() +
        ::testing::UnitTest::GetInstance()->current_test_info()->name() + ".csv";
    std::filesystem::remove(path);
  }
  void TearDown() override { std::filesystem::remove(path); }
  std::string Read() {
    std::ifstream input(path);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  }
  std::string path;
};

TEST_F(IngressTest, LiveStreamsUseIndependentBaselinesAndUint32Rollover) {
  NativeSourceIngress monitor(path, true);
  int admitted = 0;
  const auto enqueue = [&] { ++admitted; };
  monitor.Admit(Stream::kImu, 0xffffffffU, 1, enqueue);
  monitor.Admit(Stream::kImu, 0, 1.005, enqueue);
  monitor.Admit(Stream::kImu, 1, 1.01, enqueue);
  monitor.Admit(Stream::kLidar, 100, 1.01, enqueue);
  monitor.Admit(Stream::kLidar, 101, 1.02, enqueue);
  EXPECT_EQ(admitted, 5);
  EXPECT_NE(Read().find("imu,1,4294967295,1,1,baseline_admitted"), std::string::npos);
  EXPECT_NE(Read().find("lidar,1.01,100,1,1,baseline_admitted"), std::string::npos);
}

TEST_F(IngressTest, MissingGapDuplicateAndRegressionFailBeforeEnqueue) {
  NativeSourceIngress monitor(path, true);
  int admitted = 0;
  const auto enqueue = [&] { ++admitted; };
  EXPECT_THROW(monitor.Admit(Stream::kImu, std::nullopt, 1, enqueue),
               std::invalid_argument);
  monitor.Admit(Stream::kImu, 7, 1, enqueue);
  EXPECT_THROW(monitor.Admit(Stream::kImu, 7, 1.01, enqueue), std::runtime_error);
  EXPECT_THROW(monitor.Admit(Stream::kImu, 6, 1.01, enqueue), std::runtime_error);
  try {
    monitor.Admit(Stream::kImu, 10, 1.01, enqueue);
    FAIL() << "Skipped source identity was admitted";
  } catch (const std::runtime_error&) {
    monitor.Failure(Stream::kImu, 1.01, std::current_exception());
  }
  EXPECT_EQ(admitted, 1);
  const auto journal = Read();
  EXPECT_NE(journal.find(",1,1,fatal,"), std::string::npos);
  EXPECT_NE(journal.find("previous=7 received=10 missing=2"), std::string::npos);
}

TEST_F(IngressTest, DefaultHeaderlessModeAndQueueFailureRemainExplicit) {
  NativeSourceIngress monitor(path, false);
  int admitted = 0;
  monitor.Admit(Stream::kImu, std::nullopt, 1, [&] { ++admitted; });
  monitor.Admit(Stream::kImu, 400, 1.01, [&] { ++admitted; });
  EXPECT_THROW(monitor.Admit(Stream::kImu, 401, 0, [] {}),
               std::invalid_argument);
  EXPECT_THROW(monitor.Admit(Stream::kImu, 401,
                            std::numeric_limits<double>::quiet_NaN(), [] {}),
               std::invalid_argument);
  try {
    monitor.Admit(Stream::kImu, 401, 1.02, [] {
      throw std::runtime_error("native LIO input queue overflow");
    });
    FAIL() << "Enqueue failure was hidden";
  } catch (const std::runtime_error&) {
    monitor.Failure(Stream::kImu, 1.02, std::current_exception());
  }
  EXPECT_EQ(admitted, 2);
  EXPECT_NE(Read().find(",2,0,fatal,\"native LIO input queue overflow\""),
            std::string::npos);
  EXPECT_THROW(NativeSourceIngress(path, false), std::invalid_argument);
}

}  // namespace
}  // namespace apollo::localization::openloc_diagnostic
