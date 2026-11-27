#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "nlohmann/json.hpp"

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"

#include "cyber/cyber.h"
#include "modules/common/math/vec2d.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/backend/simulator/base/entity.h"
#include "modules/dreamview/backend/simulator/scenario/scenario.h"

namespace apollo {
namespace dreamview {

struct SimStaticObstacle {
  int id;
  double x;
  double y;
  double heading;
  double width;
  double length;
};

struct SimDynamicObstacle {
  int id;
  // Use Cartesian coordinates (x,y) and velocities in XY frame (vx, vy).
  double x;
  double y;
  double vx;
  double vy;
  double width;
  double length;
  double trigger_time;
  bool is_active;
};

class ObstacleManager : public SimEntity {
 public:
  ObstacleManager(std::shared_ptr<cyber::Node> node, MapService* map_service);
  ~ObstacleManager() = default;

  void LoadFromScenario(const apollo::dreamview::Scenario& scenario);

  bool Init() override;
  void Step(double dt) override;
  void Reset() override;
  void Publish() override;

  std::vector<ObstacleStatus> GetObstacleStatuses() const;

 private:
  std::shared_ptr<cyber::Node> node_;
  MapService* map_service_;
  std::shared_ptr<cyber::Writer<apollo::perception::PerceptionObstacles>>
      perception_writer_;

  std::mutex mutex_;
  std::vector<SimStaticObstacle> static_obstacles_;
  std::vector<SimDynamicObstacle> dynamic_obstacles_;

  double start_time_ = 0.0;
  int next_obstacle_id_ = 1000;
};

}  // namespace dreamview
}  // namespace apollo
