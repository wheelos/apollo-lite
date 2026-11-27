#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

#include "cyber/cyber.h"
#include "modules/common/math/vec2d.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/backend/simulator/base/entity.h"

namespace apollo {
namespace dreamview {

class EgoModel : public SimEntity {
 public:
  EgoModel(std::shared_ptr<cyber::Node> node, MapService* map_service);
  virtual ~EgoModel() = default;

  // Set the vehicle pose directly (x, y, heading) and optional velocity.
  virtual void Teleport(double x, double y, double heading,
                        double velocity = 0.0) = 0;

  virtual EgoState GetState() const = 0;

  virtual void RequestRouting(
      const std::vector<apollo::common::math::Vec2d>& waypoints) = 0;

  bool Init() override = 0;
  void Step(double dt) override = 0;
  void Reset() override = 0;
  void Publish() override = 0;

 protected:
  std::shared_ptr<cyber::Node> node_;
  MapService* map_service_;

  std::shared_ptr<cyber::Writer<apollo::localization::LocalizationEstimate>>
      localization_writer_;
  std::shared_ptr<cyber::Writer<apollo::canbus::Chassis>> chassis_writer_;
  std::shared_ptr<cyber::Writer<apollo::routing::RoutingRequest>>
      routing_writer_;
};

}  // namespace dreamview
}  // namespace apollo
