#include "modules/planning/common/motion_envelope.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {

bool BuildMotionEnvelope(const Frame& frame, MotionSpatialEnvelope* envelope,
                         std::string* reason) {
  if (envelope == nullptr || reason == nullptr) {
    AERROR << "Motion envelope requires output and diagnostic storage";
    return false;
  }
  MotionSpatialEnvelope candidate;
  const auto& open_space = frame.open_space_info();
  if (open_space.is_on_open_space_trajectory()) {
    const auto& polygon = open_space.roi_parking_boundary_polygon();
    if (polygon.size() < 3) {
      *reason = "open-space planner did not provide an authorized ROI polygon";
      return false;
    }
    for (const auto& point : polygon) {
      auto world = point;
      world.SelfRotate(open_space.origin_heading());
      world += open_space.origin_point();
      if (!std::isfinite(world.x()) || !std::isfinite(world.y())) {
        *reason = "open-space ROI contains nonfinite coordinates";
        return false;
      }
      auto* output = candidate.add_boundary();
      output->set_x(world.x());
      output->set_y(world.y());
    }
  } else {
    const auto* reference = frame.DriveReferenceLineInfo();
    if (reference == nullptr) {
      *reason = "no selected reference line for motion authorization";
      return false;
    }
    const PathBoundary* selected = nullptr;
    for (const auto& boundary : reference->GetCandidatePathBoundaries()) {
      if (boundary.label() == reference->path_data().path_label()) {
        selected = &boundary;
        break;
      }
    }
    if (selected == nullptr || selected->boundary().size() < 2 ||
        !std::isfinite(selected->delta_s()) || selected->delta_s() <= 0.0) {
      *reason = "selected path has no valid matching admissible boundary";
      return false;
    }
    double half_width = std::numeric_limits<double>::infinity();
    const auto count = selected->boundary().size();
    for (size_t vertex = 0; vertex < 2 * count; ++vertex) {
      const size_t index = vertex < count ? vertex : 2 * count - 1 - vertex;
      const auto& bounds = selected->boundary()[index];
      if (!std::isfinite(bounds.first) || !std::isfinite(bounds.second) ||
          bounds.first >= bounds.second) {
        *reason = "selected path boundary is empty or nonfinite";
        return false;
      }
      half_width = std::min(half_width, (bounds.second - bounds.first) * 0.5);
      common::SLPoint sl;
      sl.set_s(selected->start_s() + index * selected->delta_s());
      sl.set_l(vertex < count ? bounds.first : bounds.second);
      common::math::Vec2d xy;
      if (!reference->reference_line().SLToXY(sl, &xy) ||
          !std::isfinite(xy.x()) || !std::isfinite(xy.y())) {
        *reason = "cannot project selected path boundary into world frame";
        return false;
      }
      auto* output = candidate.add_boundary();
      output->set_x(xy.x());
      output->set_y(xy.y());
    }
    candidate.set_max_lateral_deviation_m(half_width);
  }
  envelope->Swap(&candidate);
  reason->clear();
  return true;
}

}  // namespace planning
}  // namespace apollo
