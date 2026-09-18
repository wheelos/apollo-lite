#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "modules/common_msgs/planning_msgs/planning.pb.h"

#include "modules/dreamview/backend/simulator/base/frame_context.h"
#include "modules/dreamview/backend/simulator/entity/ego/ego_model.h"

namespace apollo {
namespace dreamview {

class EgoPerfectModel : public EgoModel {
 public:
  EgoPerfectModel(std::shared_ptr<cyber::Node> node, MapService* map_service);
  ~EgoPerfectModel() override = default;

  bool Init() override;

  void SetPose(double x, double y, double heading,
               double velocity = 0.0) override;
  void RequestRouting(
      const std::vector<apollo::common::math::Vec2d>& waypoints) override;
  void Step(double dt) override;
  void Reset() override;
  void Publish() override;

  // Return current ego snapshot for FrameContext
  EgoState GetState() const override;

 private:
  void OnPlanning(
      const std::shared_ptr<apollo::planning::ADCTrajectory>& trajectory);
  void PublishLocalization(const apollo::common::TrajectoryPoint& point,
                           double absolute_time);
  void PublishChassis(const apollo::common::TrajectoryPoint& point,
                      double absolute_time);

  std::shared_ptr<cyber::Reader<apollo::planning::ADCTrajectory>>
      planning_reader_;

  std::mutex mutex_;
  std::shared_ptr<apollo::planning::ADCTrajectory> current_trajectory_;

  // Ego status
  apollo::common::TrajectoryPoint target_point_;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_z_ = 0.0;
  double current_heading_ = 0.0;
  double current_velocity_ = 0.0;
  double current_acceleration_ = 0.0;
  double current_time_sec_ = 0.0;
  // Last simulation time notified by engine (seconds)
  double last_sim_time_sec_ = 0.0;
};

}  // namespace dreamview
}  // namespace apollo
