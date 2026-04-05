#include "modules/planning/integration_tests/scenario_manager/scenario_builder.h"

#include "modules/planning/common/reference_line_info.h"

namespace apollo {
namespace planning {
namespace scenario {

void ScenarioBuilder::Build(const EnvInput& input, DependencyInjector* injector,
                            LocalView* local_view,
                            common::VehicleState* vehicle_state) {
  // 1. clock_mode_ mock time mode

  // 2. Injection speed and status
  vehicle_state->set_linear_velocity(input.speed);
  vehicle_state->set_timestamp(input.timestamp);

  // 3. Inject PadMessage into LocalView
  if (input.pad_message != "NONE") {
    auto pad_msg = std::make_shared<planning::PadMessage>();
    if (input.pad_message == "STOP")
      pad_msg->set_action(PadMessage::STOP);
    else if (input.pad_message == "PULL_OVER")
      pad_msg->set_action(PadMessage::PULL_OVER);
    local_view->pad_msg = pad_msg;
  }

  // 4. Inject Routing / Safety Status
  if (input.has_routing) {
    auto routing = std::make_shared<routing::RoutingResponse>();
    auto* waypoint = routing->mutable_routing_request()->add_waypoint();
    waypoint->set_id("test_lane");
    waypoint->set_s(0.0);
    local_view->routing = routing;
  }

  auto* blocking_status = injector->history()->mutable_blocking_status();
  if (input.history_stuck_time > 0.0) {
    blocking_status->set_start_stuck_time(input.history_stuck_time);
  } else {
    blocking_status->clear_start_stuck_time();
  }

  if (input.internal_emergency) {
    auto* emergency_stop = injector->planning_context()
                               ->mutable_planning_status()
                               ->mutable_emergency_stop();
    emergency_stop->mutable_stop_fence_point()->set_x(0.0);
    emergency_stop->mutable_stop_fence_point()->set_y(0.0);
  }
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
