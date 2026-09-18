#include "modules/dreamview/backend/simulator/entity/env/traffic_light_manager.h"

#include "modules/common/util/message_util.h"

using apollo::common::util::FillHeader;

namespace apollo {
namespace dreamview {

TrafficLightManager::TrafficLightManager(std::shared_ptr<cyber::Node> node)
    : node_(node) {}

void TrafficLightManager::LoadFromScenario(const Scenario& scenario) {
  std::lock_guard<std::mutex> lock(mutex_);
  lights_.clear();
  for (const auto& t : scenario.traffic_lights) {
    SimTrafficLight lt;
    lt.id = (t.id != 0) ? t.id : 0;
    lt.x = t.x;
    lt.y = t.y;
    lt.state = t.state;
    lt.trigger_time = t.trigger_time;
    lt.is_active = false;
    lights_.push_back(lt);
  }
  // Create writer for TrafficLightDetection; topic name follows convention.
  if (node_) {
    tl_writer_ = node_->CreateWriter<apollo::perception::TrafficLightDetection>(
        "/perception/traffic_light_detection");
  }
}

bool TrafficLightManager::Init() { return true; }

void TrafficLightManager::Step(double dt) override {
  std::lock_guard<std::mutex> lock(mutex_);
  elapsed_time_sec_ += dt;
  for (auto& light : lights_) {
    if (!light.is_active && elapsed_time_sec_ >= light.trigger_time) {
      light.is_active = true;
    }
    // 这里可以增加简单的红绿黄切换逻辑
  }
}

void TrafficLightManager::Publish() {
  std::lock_guard<std::mutex> lock(mutex_);
  bool has_active = false;
  for (auto& lt : lights_) {
    if (lt.is_active) has_active = true;
  }

  // Publish traffic light detection if there is any active light
  if (tl_writer_ && has_active) {
    auto msg = std::make_shared<apollo::perception::TrafficLightDetection>();
    FillHeader("TrafficLightManager", msg.get());
    msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
    for (const auto& lt : lights_) {
      if (!lt.is_active) continue;
      auto* out = msg->add_traffic_light();
      out->set_id(lt.id);
      // Map internal state to proto enum: 1=red,2=yellow,3=green
      switch (lt.state) {
        case 1:
          out->set_color(apollo::perception::TrafficLight::RED);
          break;
        case 2:
          out->set_color(apollo::perception::TrafficLight::YELLOW);
          break;
        case 3:
          out->set_color(apollo::perception::TrafficLight::GREEN);
          break;
        default:
          out->set_color(apollo::perception::TrafficLight::UNKNOWN);
      }
      out->set_position_x(lt.x);
      out->set_position_y(lt.y);
    }
    tl_writer_->Write(msg);
  }
}

void TrafficLightManager::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  lights_.clear();
  elapsed_time_sec_ = 0.0;
}

}  // namespace dreamview
}  // namespace apollo
