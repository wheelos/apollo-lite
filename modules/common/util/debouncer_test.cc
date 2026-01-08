// Copyright 2025 WheelOS. All Rights Reserved.
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

//  Created Date: 2026-01-03
//  Author: daohu527

#include "modules/common/util/debouncer.h"

#include <gtest/gtest.h>

namespace apollo {
namespace control {

/**
 * @class CounterDebouncerTest
 * @brief Test suite class for managing the CounterDebouncer test context.
 */
class CounterDebouncerTest : public ::testing::Test {
 protected:
  // Setup logic can be added here if needed
  virtual void SetUp() {}
  virtual void TearDown() {}
};

/**
 * @test Verify if the initial state is correct after object construction.
 */
TEST_F(CounterDebouncerTest, Initialization) {
  uint32_t threshold = 5;
  CounterDebouncer debouncer(threshold);

  EXPECT_EQ(debouncer.count(), 0u);
  EXPECT_FALSE(debouncer.IsActive());
}

/**
 * @test Verify the normal debouncing process until the threshold is reached.
 */
TEST_F(CounterDebouncerTest, NormalDebounceProcess) {
  uint32_t threshold = 3;
  CounterDebouncer debouncer(threshold);

  // 1st fault sample: fault not yet confirmed
  EXPECT_FALSE(debouncer.Update(true));
  EXPECT_EQ(debouncer.count(), 1u);

  // 2nd fault sample: fault not yet confirmed
  EXPECT_FALSE(debouncer.Update(true));
  EXPECT_EQ(debouncer.count(), 2u);

  // 3rd fault sample: threshold reached, fault confirmed
  EXPECT_TRUE(debouncer.Update(true));
  EXPECT_EQ(debouncer.count(), 3u);
  EXPECT_TRUE(debouncer.IsActive());

  // 4th fault sample: counter should stay capped at threshold and remain active
  EXPECT_TRUE(debouncer.Update(true));
  EXPECT_EQ(debouncer.count(), 3u);
}

/**
 * @test Verify the Immediate Reset Strategy.
 * Even if the fault is close to being confirmed, a single normal signal
 * must reset the counter to zero immediately.
 */
TEST_F(CounterDebouncerTest, ImmediateResetOnNormalSignal) {
  CounterDebouncer debouncer(10);

  // Simulate 9 consecutive fault samples
  for (int i = 0; i < 9; ++i) {
    debouncer.Update(true);
  }
  EXPECT_FALSE(debouncer.IsActive());
  EXPECT_EQ(debouncer.count(), 9u);

  // Receive one normal signal
  EXPECT_FALSE(debouncer.Update(false));

  // Counter must reset to zero immediately
  EXPECT_EQ(debouncer.count(), 0u);
  EXPECT_FALSE(debouncer.IsActive());
}

/**
 * @test Verify the recovery logic after a fault has been confirmed.
 */
TEST_F(CounterDebouncerTest, RecoveryAfterConfirmation) {
  CounterDebouncer debouncer(2);

  // Confirm the fault
  debouncer.Update(true);
  debouncer.Update(true);
  ASSERT_TRUE(debouncer.IsActive());

  // Receive a normal signal; fault state should recover immediately
  EXPECT_FALSE(debouncer.Update(false));
  EXPECT_FALSE(debouncer.IsActive());
  EXPECT_EQ(debouncer.count(), 0u);
}

/**
 * @test Verify the manual Reset function.
 */
TEST_F(CounterDebouncerTest, ManualReset) {
  CounterDebouncer debouncer(5);
  debouncer.Update(true);
  debouncer.Update(true);

  debouncer.Reset();

  EXPECT_EQ(debouncer.count(), 0u);
  EXPECT_FALSE(debouncer.IsActive());
}

/**
 * @test Boundary condition: threshold is 0.
 * Based on the logic (threshold > 0), IsActive should always be false.
 */
TEST_F(CounterDebouncerTest, ZeroThresholdBoundary) {
  CounterDebouncer debouncer(0);

  EXPECT_FALSE(debouncer.IsActive());
  EXPECT_FALSE(debouncer.Update(true));
  EXPECT_FALSE(debouncer.IsActive());
}

/**
 * @test Boundary condition: threshold is 1.
 * Should trigger/confirm fault on the very first fault sample.
 */
TEST_F(CounterDebouncerTest, SingleSampleThreshold) {
  CounterDebouncer debouncer(1);

  EXPECT_TRUE(debouncer.Update(true));
  EXPECT_TRUE(debouncer.IsActive());
  EXPECT_FALSE(debouncer.Update(false));
}

}  // namespace control
}  // namespace apollo
