// Copyright 2026 WheelOS. All Rights Reserved.
#include <array>
#include <cstddef>
#include <sstream>
#include <string>

#include "gtest/gtest.h"

#include "modules/common/vehicle_calibration/test_bundle.h"
#include "modules/perception/common/io/io_util.h"

namespace apollo {
namespace perception {
namespace common {
namespace {

std::string IntrinsicYaml(const std::array<double, 9>& matrix) {
  std::ostringstream yaml;
  yaml << "width: 640\nheight: 480\nK: [";
  for (size_t i = 0; i < matrix.size(); ++i) {
    yaml << (i == 0 ? "" : ", ") << matrix[i];
  }
  yaml << "]\nD: [0, 0, 0, 0, 0]\n";
  return yaml.str();
}

TEST(CalibrationInputTest, OfflineExtrinsicsUseOnlyExplicitInputDirectory) {
  EXPECT_EQ(ResolveExtrinsicPath("/offline/params", "camera.yaml"),
            "/offline/params/camera.yaml");
  EXPECT_EQ(ResolveExtrinsicPath("", "camera.yaml"), "camera.yaml");
}

TEST(BrownCameraIntrinsicTest, RejectsUnsupportedMatrixEntries) {
  apollo::common::vehicle_calibration::TestBundle bundle;
  const std::array<double, 9> canonical{400, 0, 320, 0, 400, 240, 0, 0, 1};
  for (const int index : {1, 3, 6, 7, 8}) {
    auto matrix = canonical;
    matrix[index] += 0.01;
    bundle.Write("intrinsic.yaml", IntrinsicYaml(matrix));
    base::BrownCameraDistortionModel model;
    EXPECT_FALSE(
        LoadBrownCameraIntrinsic(bundle.path("intrinsic.yaml"), &model))
        << index;
  }
}

TEST(BrownCameraIntrinsicTest,
     CanonicalizesRoundingAndKeepsProjectionConsistent) {
  apollo::common::vehicle_calibration::TestBundle bundle;
  bundle.Write("intrinsic.yaml", IntrinsicYaml({400, 1e-7, 320, -1e-7, 400, 240,
                                                1e-7, -1e-7, 1}));
  base::BrownCameraDistortionModel model;
  ASSERT_TRUE(LoadBrownCameraIntrinsic(bundle.path("intrinsic.yaml"), &model));
  Eigen::Matrix3f expected;
  expected << 400, 0, 320, 0, 400, 240, 0, 0, 1;
  EXPECT_TRUE(model.get_intrinsic_params().isApprox(expected, 0));
  const Eigen::Vector3f point(0, 1, 1);
  EXPECT_TRUE(model.Project(point).isApprox(Eigen::Vector2f(320, 640)));
  EXPECT_TRUE(
      model.get_camera_model()->Project(point).isApprox(model.Project(point)));
}

}  // namespace
}  // namespace common
}  // namespace perception
}  // namespace apollo
