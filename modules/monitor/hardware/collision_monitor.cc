#include "modules/monitor/hardware/collision_monitor.h"

#include "absl/strings/str_cat.h"

#include "modules/common_msgs/perception_msgs/collision_warning.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/map_util.h"
#include "modules/monitor/common/monitor_manager.h"

namespace apollo {
namespace monitor {

DEFINE_string(collision_monitor_name, "CollisionMonitor",
              "Name of the collision monitor.");
DEFINE_double(collision_monitor_interval, 0.1,
              "Collision status checking interval (s).");
DEFINE_string(collision_component_name, "Collision",
              "Collision component name.");

CollisionMonitor::CollisionMonitor()
    : RecurrentRunner(FLAGS_collision_monitor_name,
                      FLAGS_collision_monitor_interval) {}

void CollisionMonitor::RunOnce(const double current_time) {
  auto manager = MonitorManager::Instance();
  Component* component = apollo::common::util::FindOrNull(
      *manager->GetStatus()->mutable_components(),
      FLAGS_collision_component_name);
  if (component == nullptr) {
    return;
  }
  ComponentStatus* component_status = component->mutable_other_status();
  component_status->clear_status();

  static auto collision_warning_reader =
      manager->CreateReader<apollo::perception::CollisionWarning>(
          FLAGS_collision_warning_topic);
  collision_warning_reader->Observe();
  const auto collision_warning_status =
      collision_warning_reader->GetLatestObserved();

  auto* system_status = MonitorManager::Instance()->GetStatus();

  if (collision_warning_status == nullptr ||
      !collision_warning_status->is_collision()) {
    system_status->clear_passenger_msg();
    system_status->clear_safety_mode_trigger_time();
    system_status->clear_require_emergency_stop();
  } else {
    system_status->set_passenger_msg("EMERGENCY BRAKING! Imminent Collision!");
    system_status->set_safety_mode_trigger_time(current_time);
    system_status->set_require_emergency_stop(true);
  }
}

}  // namespace monitor
}  // namespace apollo
