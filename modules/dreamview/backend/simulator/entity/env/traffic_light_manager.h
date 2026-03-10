#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"

#include "cyber/cyber.h"
#include "modules/dreamview/backend/simulator/base/entity.h"
#include "modules/dreamview/backend/simulator/scenario/scenario.h"

namespace apollo {
namespace dreamview {

class TrafficLightManager : public SimEntity {
 public:
  explicit TrafficLightManager(std::shared_ptr<cyber::Node> node);
  ~TrafficLightManager() = default;

  void LoadFromScenario(const Scenario& scenario);
  bool Init() override;
  void Step(double dt) override;
  void Reset() override;
  void Publish() override;

  std::vector<TrafficLightStatus> GetTrafficLights() const;

 private:
  std::mutex mutex_;

  struct SimTrafficLight {
    int id = 0;
    double x = 0.0;
    double y = 0.0;
    int state = 0;
    double trigger_time = 0.0;
    bool is_active = false;
  };

  std::vector<SimTrafficLight> lights_;
  std::shared_ptr<cyber::Node> node_;
  double elapsed_time_sec_ = 0.0;
  std::shared_ptr<cyber::Writer<apollo::perception::TrafficLightDetection>>
      tl_writer_;
};

}  // namespace dreamview
}  // namespace apollo
