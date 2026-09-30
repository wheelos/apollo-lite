// Copyright 2026 WheelOS All Rights Reserved.
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

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "modules/local_planning/planning_context/input_gate.h"
#include "modules/local_planning/scenario.h"

namespace apollo {
namespace local_planning {
namespace {

InputPolicy Policy() {
  InputPolicy policy;
  policy.odom_frame = "odom";
  policy.clock_id = "test_clock";
  policy.max_odom_age = 1.0;
  policy.max_graph_age = 2.0;
  policy.max_prediction_age = 1.0;
  policy.stationary_speed_limit = 0.01;
  return policy;
}

CycleInput Input() {
  CycleInput input;
  input.planning_time = 10.0;
  auto& stamp = input.odometry.stamp;
  stamp.frame_id = "odom";
  stamp.clock_id = "test_clock";
  stamp.epoch = {"session-a", 1};
  stamp.sequence = 1;
  stamp.measurement_time = 9.75;
  stamp.publication_time = 9.875;
  stamp.valid_until = 12.0;
  stamp.health = InputHealth::HEALTHY;
  input.graph = stamp;
  input.prediction.stamp = stamp;
  input.prediction.graph_sequence = input.graph.sequence;
  return input;
}

void ExpectRejected(const common::Status& status) {
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(common::ErrorCode::PLANNING_ERROR, status.code());
  EXPECT_FALSE(status.error_message().empty());
}

TEST(InputGateTest, RequiresExplicitStationaryEpoch) {
  InputGate gate(Policy());
  auto input = Input();
  ExpectRejected(gate.Admit(input));
  input.odometry.speed_mps = 0.02;
  ExpectRejected(gate.BeginEpoch(input.odometry, 9.875));
  input.odometry.speed_mps = -0.02;
  ExpectRejected(gate.BeginEpoch(input.odometry, 9.875));
  input.odometry.speed_mps = 0.0;
  ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
  EXPECT_TRUE(gate.Admit(input).ok());
}

TEST(InputGateTest, RejectsInvalidPolicies) {
  std::vector<std::function<void(InputPolicy*)>> mutations = {
      [](InputPolicy* p) { p->odom_frame.clear(); },
      [](InputPolicy* p) { p->clock_id.clear(); },
      [](InputPolicy* p) { p->max_odom_age = 0.0; },
      [](InputPolicy* p) { p->max_graph_age = -1.0; },
      [](InputPolicy* p) { p->max_prediction_age = 0.0; },
      [](InputPolicy* p) { p->stationary_speed_limit = -1.0; },
      [](InputPolicy* p) {
        p->max_odom_age = std::numeric_limits<double>::infinity();
      },
      [](InputPolicy* p) {
        p->stationary_speed_limit = std::numeric_limits<double>::quiet_NaN();
      }};
  for (size_t i = 0; i < mutations.size(); ++i) {
    SCOPED_TRACE(i);
    auto policy = Policy();
    mutations[i](&policy);
    InputGate gate(policy);
    ExpectRejected(gate.BeginEpoch(Input().odometry, 9.875));
    ExpectRejected(gate.Admit(Input()));
  }
}

TEST(InputGateTest, RejectsBadMetadataOnEverySource) {
  std::vector<std::function<void(SourceStamp*)>> mutations = {
      [](SourceStamp* s) { s->frame_id = "map"; },
      [](SourceStamp* s) { s->clock_id = "other_clock"; },
      [](SourceStamp* s) { s->epoch.producer_session = "session-b"; },
      [](SourceStamp* s) { s->epoch.producer_session.clear(); },
      [](SourceStamp* s) { s->epoch.generation = 0; },
      [](SourceStamp* s) { s->epoch.generation = 2; },
      [](SourceStamp* s) { s->sequence = 0; },
      [](SourceStamp* s) { s->health = InputHealth::UNKNOWN; },
      [](SourceStamp* s) { s->health = InputHealth::DEGRADED; },
      [](SourceStamp* s) { s->health = InputHealth::INVALID; },
      [](SourceStamp* s) { s->measurement_time = -1.0; },
      [](SourceStamp* s) { s->measurement_time = 7.0; },
      [](SourceStamp* s) { s->measurement_time = 11.0; },
      [](SourceStamp* s) { s->publication_time = 9.0; },
      [](SourceStamp* s) { s->publication_time = 11.0; },
      [](SourceStamp* s) { s->valid_until = 10.0; }};
  for (int source = 0; source < 3; ++source) {
    for (size_t i = 0; i < mutations.size(); ++i) {
      SCOPED_TRACE(source);
      SCOPED_TRACE(i);
      auto input = Input();
      InputGate gate(Policy());
      ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
      SourceStamp* stamps[] = {&input.odometry.stamp, &input.graph,
                               &input.prediction.stamp};
      mutations[i](stamps[source]);
      ExpectRejected(gate.Admit(input));
      EXPECT_TRUE(gate.Admit(Input()).ok());
    }
  }
}

TEST(InputGateTest, RejectsNonfiniteTimesAndPose) {
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity()}) {
    for (int field = 0; field < 14; ++field) {
      SCOPED_TRACE(field);
      auto input = Input();
      InputGate gate(Policy());
      ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
      double* fields[] = {&input.planning_time,
                          &input.odometry.x,
                          &input.odometry.y,
                          &input.odometry.heading,
                          &input.odometry.speed_mps,
                          &input.odometry.stamp.measurement_time,
                          &input.odometry.stamp.publication_time,
                          &input.odometry.stamp.valid_until,
                          &input.graph.measurement_time,
                          &input.graph.publication_time,
                          &input.graph.valid_until,
                          &input.prediction.stamp.measurement_time,
                          &input.prediction.stamp.publication_time,
                          &input.prediction.stamp.valid_until};
      *fields[field] = bad;
      ExpectRejected(gate.Admit(input));
    }
  }
}

TEST(InputGateTest, AgeLimitInclusiveAndDeadlineExclusive) {
  auto input = Input();
  InputGate gate(Policy());
  ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
  input.planning_time = 10.75;
  ASSERT_TRUE(gate.Admit(input).ok());
  input.planning_time = 10.875;
  ExpectRejected(gate.Admit(input));

  input = Input();
  input.graph.valid_until = 10.125;
  InputGate deadline_gate(Policy());
  ASSERT_TRUE(deadline_gate.BeginEpoch(input.odometry, 9.875).ok());
  ASSERT_TRUE(deadline_gate.Admit(input).ok());
  input.planning_time = 10.125;
  ExpectRejected(deadline_gate.Admit(input));
}

TEST(InputGateTest, ReusesImmutableSourcesAtIncreasingPlanningTime) {
  auto input = Input();
  InputGate gate(Policy());
  ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
  ASSERT_TRUE(gate.Admit(input).ok());
  ExpectRejected(gate.Admit(input));
  input.planning_time = 10.125;
  EXPECT_TRUE(gate.Admit(input).ok());
  input.planning_time = 9.875;
  ExpectRejected(gate.Admit(input));
}

TEST(InputGateTest, ReusesOldGraphWithCurrentOdometryAndPrediction) {
  auto input = Input();
  InputGate gate(Policy());
  ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
  ASSERT_TRUE(gate.Admit(input).ok());
  input.planning_time = 10.25;
  input.odometry.stamp.sequence = 2;
  input.odometry.stamp.measurement_time = 10.125;
  input.odometry.stamp.publication_time = 10.25;
  input.odometry.x = 1.0;
  input.odometry.speed_mps = 1.0;
  input.prediction.stamp = input.odometry.stamp;
  EXPECT_TRUE(gate.Admit(input).ok());
}

TEST(InputGateTest, RejectsMutationWithoutAdvancingFrontier) {
  for (int source = 0; source < 3; ++source) {
    SCOPED_TRACE(source);
    auto input = Input();
    InputGate gate(Policy());
    ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
    ASSERT_TRUE(gate.Admit(input).ok());
    input.planning_time = 10.125;
    auto mutated = input;
    SourceStamp* stamps[] = {&mutated.odometry.stamp, &mutated.graph,
                             &mutated.prediction.stamp};
    stamps[source]->valid_until = 13.0;
    ExpectRejected(gate.Admit(mutated));
    EXPECT_TRUE(gate.Admit(input).ok());
  }
}

TEST(InputGateTest, RejectsPoseMutationAndDuplicateMeasurement) {
  auto input = Input();
  InputGate gate(Policy());
  ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
  auto bad = input;
  bad.odometry.x = 1.0;
  ExpectRejected(gate.Admit(bad));
  ASSERT_TRUE(gate.Admit(input).ok());
  input.planning_time = 10.125;
  bad = input;
  bad.odometry.stamp.sequence = 2;
  ExpectRejected(gate.Admit(bad));
  bad.odometry.stamp.measurement_time = 9.5;
  ExpectRejected(gate.Admit(bad));
  EXPECT_TRUE(gate.Admit(input).ok());
}

TEST(InputGateTest, NewGraphRequiresNewPredictionVersion) {
  auto input = Input();
  InputGate gate(Policy());
  ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
  ASSERT_TRUE(gate.Admit(input).ok());
  input.planning_time = 10.125;
  input.graph.sequence = 2;
  ExpectRejected(gate.Admit(input));
  input.prediction.graph_sequence = 2;
  ExpectRejected(gate.Admit(input));
  input.prediction.stamp.sequence = 2;
  EXPECT_TRUE(gate.Admit(input).ok());
}

TEST(InputGateTest, RejectsSequenceRegressionOnEverySource) {
  for (int source = 0; source < 3; ++source) {
    auto input = Input();
    input.odometry.stamp.sequence = 2;
    input.graph.sequence = 2;
    input.prediction.stamp.sequence = 2;
    input.prediction.graph_sequence = 2;
    InputGate gate(Policy());
    ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
    ASSERT_TRUE(gate.Admit(input).ok());
    input.planning_time = 10.125;
    SourceStamp* stamps[] = {&input.odometry.stamp, &input.graph,
                             &input.prediction.stamp};
    stamps[source]->sequence = 1;
    if (source == 1) {
      input.prediction.graph_sequence = 1;
    }
    ExpectRejected(gate.Admit(input));
  }
}

TEST(InputGateTest, EpochTransitionRejectsOldMessages) {
  auto input = Input();
  InputGate gate(Policy());
  ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
  ASSERT_TRUE(gate.Admit(input).ok());
  auto next = input;
  next.planning_time = 10.125;
  next.odometry.stamp.epoch = {"session-b", 2};
  next.graph.epoch = next.odometry.stamp.epoch;
  next.prediction.stamp.epoch = next.odometry.stamp.epoch;
  ExpectRejected(gate.Admit(next));
  ASSERT_TRUE(gate.BeginEpoch(next.odometry, 10.125).ok());
  input.planning_time = 10.125;
  ExpectRejected(gate.Admit(input));
  EXPECT_TRUE(gate.Admit(next).ok());
  ExpectRejected(gate.BeginEpoch(input.odometry, 10.25));
  next.planning_time = 10.25;
  ExpectRejected(gate.Admit(next));
}

TEST(InputGateTest, FailedMovingResetDisarmsGate) {
  auto input = Input();
  InputGate gate(Policy());
  ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
  ASSERT_TRUE(gate.Admit(input).ok());
  auto reset = input.odometry;
  reset.stamp.epoch.generation = 2;
  reset.speed_mps = 2.0;
  ExpectRejected(gate.BeginEpoch(reset, 10.125));
  input.planning_time = 10.125;
  ExpectRejected(gate.Admit(input));
}

TEST(InputGateTest, EpochTransitionCannotRewindClock) {
  auto input = Input();
  InputGate gate(Policy());
  ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
  ASSERT_TRUE(gate.Admit(input).ok());
  input.odometry.stamp.epoch.generation = 2;
  ExpectRejected(gate.BeginEpoch(input.odometry, 9.875));
}

TEST(InputGateTest, NewEpochMayReuseSequenceWithDifferentPayload) {
  auto input = Input();
  InputGate gate(Policy());
  ASSERT_TRUE(gate.BeginEpoch(input.odometry, 9.875).ok());
  input.corridor.lane_id = 1;
  ASSERT_TRUE(gate.Admit(input).ok());
  const auto old = input;
  input.planning_time = 10.125;
  input.odometry.stamp.epoch = {"restarted-producer", 2};
  input.graph.epoch = input.odometry.stamp.epoch;
  input.prediction.stamp.epoch = input.odometry.stamp.epoch;
  input.corridor.lane_id = 2;
  input.occupancy.horizon = 10.0;
  ASSERT_TRUE(gate.BeginEpoch(input.odometry, 10.125).ok());
  ASSERT_TRUE(gate.Admit(input).ok());
  auto delayed = old;
  delayed.planning_time = 10.25;
  ExpectRejected(gate.Admit(delayed));
  input.planning_time = 10.25;
  EXPECT_TRUE(gate.Admit(input).ok());
}

struct TaskTrace {
  std::vector<std::string> calls;
  int resets = 0;
  bool fail = false;
};

class RecordingTask : public Task {
 public:
  RecordingTask(std::string name, TaskTrace* trace)
      : name_(std::move(name)), trace_(trace) {}
  std::string Name() const override { return name_; }
  common::Status Execute(const CycleInput& input) override {
    trace_->calls.push_back(name_);
    if (trace_->fail) {
      return common::Status(common::ErrorCode::PLANNING_ERROR, "test failure");
    }
    EXPECT_GT(input.planning_time, 0.0);
    return common::Status::OK();
  }
  void Reset() override { ++trace_->resets; }

 private:
  std::string name_;
  TaskTrace* trace_;
};

TEST(ScenarioTest, TasksRunInOrderOnlyAfterAdmission) {
  TaskTrace trace;
  std::vector<std::unique_ptr<Task>> tasks;
  tasks.emplace_back(new RecordingTask("first", &trace));
  tasks.emplace_back(new RecordingTask("second", &trace));
  Scenario scenario(Policy(), std::move(tasks));
  EXPECT_EQ(ScenarioState::IDLE, scenario.state());
  ExpectRejected(scenario.Process(Input()));
  EXPECT_TRUE(trace.calls.empty());
  EXPECT_EQ(ScenarioState::INVALID, scenario.state());
  ASSERT_TRUE(scenario.BeginEpoch(Input().odometry, 9.875).ok());
  ASSERT_TRUE(scenario.Process(Input()).ok());
  EXPECT_EQ((std::vector<std::string>{"first", "second"}), trace.calls);
  EXPECT_EQ(ScenarioState::TASKS_COMPLETED, scenario.state());
  const int resets = trace.resets;
  auto invalid = Input();
  invalid.planning_time = 10.125;
  invalid.graph.frame_id = "map";
  ExpectRejected(scenario.Process(invalid));
  EXPECT_EQ(ScenarioState::INVALID, scenario.state());
  EXPECT_EQ(resets + 2, trace.resets);
  EXPECT_EQ(2U, trace.calls.size());
}

TEST(ScenarioTest, TaskFailureShortCircuitsAndResetsEntireStage) {
  TaskTrace first;
  TaskTrace second;
  first.fail = true;
  std::vector<std::unique_ptr<Task>> tasks;
  tasks.emplace_back(new RecordingTask("first", &first));
  tasks.emplace_back(new RecordingTask("second", &second));
  Scenario scenario(Policy(), std::move(tasks));
  ASSERT_TRUE(scenario.BeginEpoch(Input().odometry, 9.875).ok());
  auto status = scenario.Process(Input());
  ExpectRejected(status);
  EXPECT_NE(std::string::npos, status.error_message().find("first"));
  EXPECT_EQ(1U, first.calls.size());
  EXPECT_TRUE(second.calls.empty());
  EXPECT_EQ(2, first.resets);
  EXPECT_EQ(2, second.resets);
  EXPECT_EQ(ScenarioState::INVALID, scenario.state());
  first.fail = false;
  auto next = Input();
  next.planning_time = 10.125;
  EXPECT_TRUE(scenario.Process(next).ok());
  EXPECT_EQ(ScenarioState::TASKS_COMPLETED, scenario.state());
}

TEST(ScenarioTest, EpochTransitionResetsTaskHistory) {
  TaskTrace trace;
  std::vector<std::unique_ptr<Task>> tasks;
  tasks.emplace_back(new RecordingTask("task", &trace));
  Scenario scenario(Policy(), std::move(tasks));
  auto input = Input();
  ASSERT_TRUE(scenario.BeginEpoch(input.odometry, 9.875).ok());
  ASSERT_TRUE(scenario.Process(input).ok());
  auto reset = input.odometry;
  reset.stamp.epoch.generation = 2;
  ASSERT_TRUE(scenario.BeginEpoch(reset, 10.125).ok());
  EXPECT_EQ(2, trace.resets);
  EXPECT_EQ(ScenarioState::IDLE, scenario.state());
  reset.stamp.epoch.generation = 3;
  reset.speed_mps = 1.0;
  ExpectRejected(scenario.BeginEpoch(reset, 10.25));
  EXPECT_EQ(3, trace.resets);
  EXPECT_EQ(ScenarioState::INVALID, scenario.state());
}

TEST(ScenarioTest, RejectsEmptyNullUnnamedAndDuplicateTasks) {
  TaskTrace trace;
  for (int variant = 0; variant < 4; ++variant) {
    SCOPED_TRACE(variant);
    std::vector<std::unique_ptr<Task>> tasks;
    if (variant == 1) {
      tasks.emplace_back(nullptr);
    } else if (variant == 2) {
      tasks.emplace_back(new RecordingTask("", &trace));
    } else if (variant == 3) {
      tasks.emplace_back(new RecordingTask("same", &trace));
      tasks.emplace_back(new RecordingTask("same", &trace));
    }
    Scenario scenario(Policy(), std::move(tasks));
    ExpectRejected(scenario.BeginEpoch(Input().odometry, 9.875));
    ExpectRejected(scenario.Process(Input()));
    EXPECT_EQ(ScenarioState::INVALID, scenario.state());
    EXPECT_TRUE(trace.calls.empty());
  }
}

}  // namespace
}  // namespace local_planning
}  // namespace apollo
