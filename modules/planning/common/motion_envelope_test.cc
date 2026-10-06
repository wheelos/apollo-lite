#include "modules/planning/common/motion_envelope.h"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {
namespace {

TEST(MotionEnvelopeTest, MissingPlannerBoundaryDoesNotInventTrackBox) {
  Frame frame(1);
  MotionSpatialEnvelope envelope;
  envelope.add_boundary()->set_x(99.0);
  const auto original = envelope.SerializeAsString();
  std::string reason;
  EXPECT_FALSE(BuildMotionEnvelope(frame, &envelope, &reason));
  EXPECT_FALSE(reason.empty());
  EXPECT_EQ(envelope.SerializeAsString(), original);
}

TEST(MotionEnvelopeTest, OpenSpaceUsesWorldTransformedPolygon) {
  Frame frame(1);
  auto* info = frame.mutable_open_space_info();
  info->set_is_on_open_space_trajectory(true);
  info->set_origin_heading(std::acos(-1.0) * 0.5);
  info->mutable_origin_point()->set_x(10.0);
  info->mutable_origin_point()->set_y(20.0);
  auto* polygon = info->mutable_roi_parking_boundary_polygon();
  polygon->emplace_back(0.0, 0.0);
  polygon->emplace_back(2.0, 0.0);
  polygon->emplace_back(0.0, 3.0);
  MotionSpatialEnvelope envelope;
  std::string reason;
  ASSERT_TRUE(BuildMotionEnvelope(frame, &envelope, &reason)) << reason;
  ASSERT_EQ(envelope.boundary_size(), 3);
  EXPECT_NEAR(envelope.boundary(0).x(), 10.0, 1e-10);
  EXPECT_NEAR(envelope.boundary(0).y(), 20.0, 1e-10);
  EXPECT_NEAR(envelope.boundary(1).x(), 10.0, 1e-10);
  EXPECT_NEAR(envelope.boundary(1).y(), 22.0, 1e-10);
  EXPECT_NEAR(envelope.boundary(2).x(), 7.0, 1e-10);
  EXPECT_NEAR(envelope.boundary(2).y(), 20.0, 1e-10);
  (*polygon)[1].set_x(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(BuildMotionEnvelope(frame, &envelope, &reason));
}

}  // namespace
}  // namespace planning
}  // namespace apollo
