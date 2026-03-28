#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Geometry"

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/proto/lidar_unified_component_config.pb.h"

#include "modules/transform/buffer_interface.h"

namespace apollo {
namespace drivers {
namespace lidar {

/// @brief Point cloud storage type (CPU Host or GPU Device)
enum class MemoryDeviceType { kHost = 0, kDevice = 1 };

/// @brief Point cloud abstraction supporting Zero-Copy & CUDA memory.
struct PointCloudBuffer {
  void* data_ptr = nullptr;  // Raw memory ptr to array of points
  size_t capacity = 0;       // Total allocated capacity (point count)
  size_t valid_count = 0;    // Number of effectively populated points
  size_t item_size = 0;      // sizeof(PointT) e.g., sizeof(PointXYZIT)
  MemoryDeviceType device_type = MemoryDeviceType::kHost;
  int device_id = -1;  // -1: Host, 0-N: CUDA device index
};

/// @brief Context carrying all necessary data for a single sensor frame.
struct SensorFrameContext {
  std::string sensor_id;
  std::shared_ptr<const PointCloud> point_cloud;
  bool is_primary = false;
  double min_timestamp_sec = 0.0;
  double max_timestamp_sec = 0.0;
};

// ============================================================================
// 1. Motion Compensation & Deskew Policy Interface (Generates TFs)
// ============================================================================
class LidarDeskewPolicy {
 public:
  virtual ~LidarDeskewPolicy() = default;

  virtual bool Init(const LidarUnifiedComponentConfig& config,
                    apollo::transform::BufferInterface* tf_buffer) = 0;

  virtual bool ComputeMotionCompensationPoses(
      const SensorFrameContext& frame_context,
      std::vector<double>* sample_times,
      std::vector<Eigen::Affine3d>* poses) = 0;
};

// ============================================================================
// 2. Spatial Fusion Policy Interface (Transforms & Merges Points)
// ============================================================================
class LidarFusionPolicy {
 public:
  virtual ~LidarFusionPolicy() = default;

  virtual bool Init(const LidarUnifiedComponentConfig& config,
                    apollo::transform::BufferInterface* tf_buffer) = 0;

  /// @brief Fuses multiple deskewed point clouds into a single base_link target
  /// cloud taking into account the primary sensor's reference time.
  /// @note Can leverage Host or Device buffers.
  virtual bool FuseToBaseLink(
      double reference_timestamp_sec,
      const std::vector<SensorFrameContext>& frames,
      const std::vector<std::vector<Eigen::Affine3d>>& frames_motion_poses,
      const std::vector<std::vector<double>>& frames_motion_times,
      PointCloudBuffer* output_buffer) = 0;
};

// ============================================================================
// 3. Post-Processing Filter Policy Interface (Ego Car, Voxel)
// ============================================================================
class LidarFilterPolicy {
 public:
  virtual ~LidarFilterPolicy() = default;

  virtual bool Init(const LidarUnifiedComponentConfig& config) = 0;

  /// @brief Apply filtering passes (Ego box, Voxel) directly on the buffer
  /// memory.
  /// @return number of remaining valid points.
  virtual size_t ApplyFilters(PointCloudBuffer* io_buffer,
                              size_t* ego_filtered_count,
                              size_t* voxel_filtered_count) = 0;
};

// ============================================================================
// Policy Factory Interface
// ============================================================================
class LidarPolicyFactory {
 public:
  static std::unique_ptr<LidarDeskewPolicy> CreateDeskewPolicy(
      const std::string& mode);
  static std::unique_ptr<LidarFusionPolicy> CreateFusionPolicy(
      const std::string& mode);
  static std::unique_ptr<LidarFilterPolicy> CreateFilterPolicy(
      const std::string& mode);
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
