#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "Eigen/Geometry"

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/component/timer_component.h"
#include "modules/collision_guardian/core/occupancy_detector.h"
#include "modules/collision_guardian/core/risk_filter.h"
#include "modules/collision_guardian/proto/collision_guardian.pb.h"
#include "modules/collision_guardian/proto/collision_guardian_config.pb.h"
#include "modules/transform/transform_query.h"

namespace apollo {
namespace collision_guardian {

class CollisionGuardianComponent final : public cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  bool ValidateConfig() const;
  bool AppendPointCloud(const drivers::PointCloud& pointcloud,
                        std::vector<Eigen::Vector3d>* points,
                        double* timestamp_sec);
  bool AppendOccupancyGrid(const OccupancyGrid& grid,
                           std::vector<Eigen::Vector3d>* points,
                           double* timestamp_sec);
  bool QueryTransform(double timestamp_sec, const std::string& source_frame,
                      Eigen::Affine3d* transform);
  void PublishRisk(const RiskFilterResult& filter_result,
                   const OccupancyResult& occupancy_result, bool input_valid,
                   double source_timestamp_sec, uint32_t source_mask);
  static RiskState ToProtoState(FilterState state);
  static RiskZone DetermineZone(const OccupancyResult& result);

  CollisionGuardianConfig config_;
  std::unique_ptr<OccupancyDetector> detector_;
  std::unique_ptr<RiskFilter> risk_filter_;
  transform::TransformQuery transform_query_;

  std::shared_ptr<cyber::Reader<drivers::PointCloud>> pointcloud_reader_;
  std::shared_ptr<cyber::Reader<OccupancyGrid>> occupancy_grid_reader_;
  std::shared_ptr<cyber::Writer<CollisionRisk>> risk_writer_;

  std::mutex mutex_;
  std::shared_ptr<drivers::PointCloud> latest_pointcloud_;
  std::shared_ptr<OccupancyGrid> latest_occupancy_grid_;
  double last_pointcloud_timestamp_sec_ = 0.0;
  double last_grid_timestamp_sec_ = 0.0;
  double last_valid_input_time_sec_ = 0.0;
  bool fault_published_ = false;
};

CYBER_REGISTER_COMPONENT(CollisionGuardianComponent)

}  // namespace collision_guardian
}  // namespace apollo
