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

// Created Date: 2026-04-15
// Author: daohu527

#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_slack_osqp_interface.h"

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/init.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace {

std::string PlannerConfigPath() {
  constexpr char kConfigPath[] =
      "modules/planning/testdata/conf/open_space_standard_parking_lot.pb.txt";
  const char* test_srcdir = std::getenv("TEST_SRCDIR");
  const char* test_workspace = std::getenv("TEST_WORKSPACE");
  if (test_srcdir != nullptr && test_workspace != nullptr) {
    return std::string(test_srcdir) + "/" + test_workspace + "/" + kConfigPath;
  }
  return kConfigPath;
}

void ExpectUpperTriangularSorted(const std::vector<OSQPInt>& indptr,
                                 const std::vector<OSQPInt>& indices,
                                 const OSQPInt num_cols) {
  ASSERT_EQ(indptr.size(), static_cast<size_t>(num_cols + 1));
  for (OSQPInt col = 0; col < num_cols; ++col) {
    ASSERT_LE(indptr[col], indptr[col + 1]);
    for (OSQPInt idx = indptr[col]; idx < indptr[col + 1]; ++idx) {
      EXPECT_LE(indices[idx], col);
      if (idx + 1 < indptr[col + 1]) {
        EXPECT_LT(indices[idx], indices[idx + 1]);
      }
    }
  }
}

bool HasOffDiagonalEntry(const std::vector<OSQPInt>& indptr,
                         const std::vector<OSQPInt>& indices,
                         const OSQPInt num_cols) {
  for (OSQPInt col = 0; col < num_cols; ++col) {
    for (OSQPInt idx = indptr[col]; idx < indptr[col + 1]; ++idx) {
      if (indices[idx] < col) {
        return true;
      }
    }
  }
  return false;
}

class DualVariableWarmStartSlackOSQPInterfaceTest : public ::testing::Test {
 public:
  void SetUp() override {
    FLAGS_planner_open_space_config_filename = PlannerConfigPath();

    ACHECK(apollo::cyber::common::GetProtoFromFile(
        FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
        << "Failed to load open space config file "
        << FLAGS_planner_open_space_config_filename;

    ProblemSetup();
  }

 protected:
  void ProblemSetup() {
    obstacles_edges_num_ = Eigen::MatrixXi(obstacles_num_, 1);
    obstacles_edges_num_ << 2, 1, 2, 1;
    Eigen::MatrixXd xWS = Eigen::MatrixXd::Ones(4, horizon_ + 1);
    ptop_.reset(new DualVariableWarmStartSlackOSQPInterface(
        horizon_, ts_, ego_, obstacles_edges_num_, obstacles_num_, obstacles_A_,
        obstacles_b_, xWS, planner_open_space_config_));
  }

  size_t horizon_ = 5;
  size_t obstacles_num_ = 4;
  double ts_ = 0.1;
  Eigen::MatrixXd ego_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXi obstacles_edges_num_;
  Eigen::MatrixXd obstacles_A_ = Eigen::MatrixXd::Ones(10, 2);
  Eigen::MatrixXd obstacles_b_ = Eigen::MatrixXd::Ones(10, 1);
  std::unique_ptr<DualVariableWarmStartSlackOSQPInterface> ptop_ = nullptr;
  PlannerOpenSpaceConfig planner_open_space_config_;
};

}  // namespace

TEST_F(DualVariableWarmStartSlackOSQPInterfaceTest, Initialization) {
  EXPECT_NE(ptop_, nullptr);
}

TEST_F(DualVariableWarmStartSlackOSQPInterfaceTest,
       AssemblePBuildsUpperTriangularSortedCsc) {
  std::vector<OSQPFloat> P_data;
  std::vector<OSQPInt> P_indices;
  std::vector<OSQPInt> P_indptr;

  ptop_->assembleP(&P_data, &P_indices, &P_indptr);

  const OSQPInt num_cols = static_cast<OSQPInt>(P_indptr.size() - 1);
  ExpectUpperTriangularSorted(P_indptr, P_indices, num_cols);
  EXPECT_TRUE(HasOffDiagonalEntry(P_indptr, P_indices, num_cols));
}

TEST_F(DualVariableWarmStartSlackOSQPInterfaceTest, OptimizeSucceeds) {
  ASSERT_TRUE(ptop_->optimize());

  Eigen::MatrixXd l_warm_up;
  Eigen::MatrixXd n_warm_up;
  Eigen::MatrixXd slacks;
  ptop_->get_optimization_results(&l_warm_up, &n_warm_up, &slacks);

  EXPECT_EQ(l_warm_up.rows(), obstacles_edges_num_.sum());
  EXPECT_EQ(l_warm_up.cols(), static_cast<int>(horizon_) + 1);
  EXPECT_EQ(n_warm_up.rows(), 4 * static_cast<int>(obstacles_num_));
  EXPECT_EQ(n_warm_up.cols(), static_cast<int>(horizon_) + 1);
  EXPECT_EQ(slacks.rows(), static_cast<int>(obstacles_num_));
  EXPECT_EQ(slacks.cols(), static_cast<int>(horizon_) + 1);
}

}  // namespace planning
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  const int result = RUN_ALL_TESTS();
  apollo::cyber::Clear();
  std::fflush(nullptr);
  std::_Exit(result);
}
