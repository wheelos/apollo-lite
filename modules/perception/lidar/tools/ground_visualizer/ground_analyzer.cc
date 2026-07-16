/******************************************************************************
 * Copyright 2026 The WheelOS Team. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "modules/perception/common/i_lib/pc/i_ground.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace apollo {
namespace perception {
namespace lidar {
namespace tools {

struct GroundAnalyzerConfig {
  uint32_t grid_size = 32;
  float ground_thres = 3.0f;
  float roi_rad_x = 120.0f;
  float roi_rad_y = 120.0f;
  float roi_rad_z = 100.0f;
  uint32_t nr_smooth_iter = 6;
  bool use_roi = true;
  bool use_ground_service = true;
  float sample_region_z_lower = -3.0f;
  float sample_region_z_upper = -1.0f;
  float roi_near_rad = 32.0f;
  float planefit_orien_threshold = 30.0f;
  uint32_t big_grid_size = 512;
  uint32_t small_grid_size = 64;
  float z_compare_thres = 0.3f;
  float smooth_z_thres = 2.0f;
  float planefit_dist_thres_near = 1.00f;
  float planefit_dist_thres_far = 1.50f;
  uint32_t inliers_min_threshold = 3;
  float near_range_dist = 10.0f;
  float near_range_ground_thres = 3.0f;
  float middle_range_dist = 20.0f;
  float middle_range_ground_thres = 3.0f;
  bool enable_debug_non_ground_cloud = false;
  std::string debug_cloud_channel = "/apollo/sensor/perception/nonground/PointCloud2";

  // Vehicle pose for coordinate transform (world -> vehicle)
  float vehicle_x = 0.0f;
  float vehicle_y = 0.0f;
  float vehicle_z = 0.0f;

  void Print() const {
    std::cout << "Ground Analyzer Configuration:" << std::endl;
    std::cout << "  grid_size: " << grid_size << std::endl;
    std::cout << "  small_grid_size: " << small_grid_size << std::endl;
    std::cout << "  big_grid_size: " << big_grid_size << std::endl;
    std::cout << "  roi_rad_x: " << roi_rad_x << std::endl;
    std::cout << "  planefit_dist_thres_near: " << planefit_dist_thres_near << std::endl;
    std::cout << "  planefit_dist_thres_far: " << planefit_dist_thres_far << std::endl;
    std::cout << "  planefit_orien_threshold: " << planefit_orien_threshold << std::endl;
    std::cout << "  ground_thres: " << ground_thres << std::endl;
    std::cout << "  nr_smooth_iter: " << nr_smooth_iter << std::endl;
  }
};

// Debug information structure
struct DetectionDebugInfo {
  unsigned int grid_dim_x = 0;
  unsigned int grid_dim_y = 0;
  int valid_planes = 0;
  int failed_cells = 0;
  int total_support = 0;
  int ground_count = 0;
  int nonground_count = 0;
  float min_height = 0;
  float max_height = 0;
  int cells_with_points = 0;
  int empty_cells = 0;
  int max_points_in_cell = 0;

  struct PlaneInfo {
    unsigned int r, c;
    float nx, ny, nz, d;
    int support;
    float angle_to_x;
  };
  std::vector<PlaneInfo> planes;  // First few planes for display

  void Print() const {
    std::cout << "\n===== Ground Detection Debug Info =====" << std::endl;
    std::cout << "Grid dimension: " << grid_dim_x << " x " << grid_dim_y << std::endl;
    std::cout << "Grid point distribution:" << std::endl;
    std::cout << "  Cells with points: " << cells_with_points << " / " << (grid_dim_x * grid_dim_y) << std::endl;
    std::cout << "  Empty cells: " << empty_cells << std::endl;
    std::cout << "  Max points in one cell: " << max_points_in_cell << std::endl;
    std::cout << "Valid planes: " << valid_planes << " / " << (grid_dim_x * grid_dim_y) << std::endl;
    std::cout << "Failed cells: " << failed_cells << std::endl;
    std::cout << "Total support points: " << total_support << std::endl;
    std::cout << "\nResults: ground=" << ground_count << " (" << (100.0*ground_count/(ground_count+nonground_count)) << "%), "
              << "nonground=" << nonground_count << " (" << (100.0*nonground_count/(ground_count+nonground_count)) << "%)" << std::endl;
    std::cout << "Height range: [" << min_height << ", " << max_height << "]" << std::endl;
    std::cout << "===== End Debug Info =====\n" << std::endl;
  }

  void ToJson(std::ostream& out) const {
    out << "\"debug\": {";
    out << "\"grid_dim_x\": " << grid_dim_x << ",";
    out << "\"grid_dim_y\": " << grid_dim_y << ",";
    out << "\"cells_with_points\": " << cells_with_points << ",";
    out << "\"empty_cells\": " << empty_cells << ",";
    out << "\"max_points_in_cell\": " << max_points_in_cell << ",";
    out << "\"valid_planes\": " << valid_planes << ",";
    out << "\"failed_cells\": " << failed_cells << ",";
    out << "\"total_support\": " << total_support << ",";
    out << "\"ground_count\": " << ground_count << ",";
    out << "\"nonground_count\": " << nonground_count << ",";
    out << "\"min_height\": " << min_height << ",";
    out << "\"max_height\": " << max_height << ",";
    out << "\"planes\": [";
    for (size_t i = 0; i < planes.size(); ++i) {
      const auto& p = planes[i];
      out << "{\"r\":" << p.r << ",\"c\":" << p.c << ",";
      out << "\"nx\":" << p.nx << ",\"ny\":" << p.ny << ",\"nz\":" << p.nz << ",\"d\":" << p.d << ",";
      out << "\"support\":" << p.support << ",\"angle_to_x\":" << p.angle_to_x << "}";
      if (i < planes.size() - 1) out << ",";
    }
    out << "]}";
  }
};

class GroundAnalyzer {
 public:
  explicit GroundAnalyzer(const GroundAnalyzerConfig& config) : config_(config) {
    // Map from proto field names to PlaneFitGroundDetectorParam field names
    // Matches spatio_temporal_ground_detector.cc Init() function exactly
    param_.nr_grids_coarse = config.small_grid_size;
    param_.nr_grids_fine = config.big_grid_size;
    param_.roi_region_rad_x = config.roi_rad_x;
    param_.roi_region_rad_y = config.roi_rad_y;
    param_.roi_region_rad_z = config.roi_rad_z;
    param_.nr_smooth_iter = config.nr_smooth_iter;
    param_.sample_region_z_lower = config.sample_region_z_lower;
    param_.sample_region_z_upper = config.sample_region_z_upper;
    param_.roi_near_rad = config.roi_near_rad;
    param_.planefit_orien_threshold = config.planefit_orien_threshold;
    param_.planefit_dist_threshold_near = config.planefit_dist_thres_near;
    param_.planefit_dist_threshold_far = config.planefit_dist_thres_far;
    param_.planefit_filter_threshold = config.z_compare_thres;
    param_.candidate_filter_threshold = config.smooth_z_thres;
    param_.nr_inliers_min_threshold = config.inliers_min_threshold;
    param_.nr_points_max = 500000;

    // Create detector
    detector_ = std::make_unique<common::PlaneFitGroundDetector>(param_);
    detector_->Init();
  }

  bool AnalyzeAndOutputJSON(const std::string& pcd_path, std::ostream& out,
                           const std::string& save_pcd_path = "") {
    // Load PCD file
    pcl::PointCloud<PCLPointXYZIT> cloud;
    if (pcl::io::loadPCDFile<PCLPointXYZIT>(pcd_path, cloud) == -1) {
      out << "{\"error\": \"Failed to load PCD file\"}";
      return false;
    }

    // Prepare data - use SAME center logic as system
    std::vector<float> points;
    std::vector<int> indices;
    points.reserve(cloud.size() * 3);
    indices.reserve(cloud.size());

    float cx, cy, cz;
    // Use system's center (vehicle pose) if provided, same as system's lidar2world_pose.translation()
    if (config_.vehicle_x != 0.0f || config_.vehicle_y != 0.0f || config_.vehicle_z != 0.0f) {
      cx = config_.vehicle_x;
      cy = config_.vehicle_y;
      cz = config_.vehicle_z;
    } else {
      // Fallback: calculate average
      cx = cy = cz = 0;
      for (const auto& pt : cloud) {
        cx += pt.x;
        cy += pt.y;
        cz += pt.z;
      }
      cx /= cloud.size();
      cy /= cloud.size();
      cz /= cloud.size();
    }

    for (size_t i = 0; i < cloud.size(); ++i) {
      const auto& pt = cloud[i];
      points.push_back(pt.x - cx);
      points.push_back(pt.y - cy);
      points.push_back(pt.z - cz);
      indices.push_back(static_cast<int>(i));
    }

    // Run ground detection
    std::vector<float> heights(cloud.size());

    bool success = detector_->Detect(points.data(), heights.data(),
                                     cloud.size(), 3);

    if (!success) {
      out << "{\"error\": \"Ground detection failed\"}";
      return false;
    }

    // Collect debug information
    DetectionDebugInfo debug_info = CollectDetectionDebug(cloud, heights);

    // Save PCD files if requested (before JSON output)
    if (!save_pcd_path.empty()) {
      SaveResultPCD(cloud, heights, save_pcd_path);
    }

    // Output JSON
    out << "{";
    out << "\"points\": [";

    for (size_t i = 0; i < cloud.size(); ++i) {
      const auto& pt = cloud[i];
      float h = heights[i];
      int label = (h > config_.ground_thres) ? 0 : 1;

      // Output centered coordinates - must match what detector actually processes
      out << "[" << (pt.x - cx) << "," << (pt.y - cy) << "," << (pt.z - cz) << "," << label << "]";
      if (i < cloud.size() - 1) out << ",";
    }

    out << "],";
    out << "\"config\": {";
    out << "\"small_grid_size\": " << config_.small_grid_size << ",";
    out << "\"roi_rad_x\": " << config_.roi_rad_x << ",";
    out << "\"planefit_dist_thres_near\": " << config_.planefit_dist_thres_near << ",";
    out << "\"planefit_dist_thres_far\": " << config_.planefit_dist_thres_far << ",";
    out << "\"ground_thres\": " << config_.ground_thres;
    out << "},";
    out << "\"num_points\": " << cloud.size() << ",";
    debug_info.ToJson(out);  // Add debug info to JSON
    out << "}";

    return true;
  }

  bool Analyze(const std::string& pcd_path, const std::string& output_path,
               const std::string& save_pcd_path = "") {
    // Load PCD file
    pcl::PointCloud<PCLPointXYZIT> cloud;
    if (pcl::io::loadPCDFile<PCLPointXYZIT>(pcd_path, cloud) == -1) {
      std::cerr << "Failed to load PCD file: " << pcd_path << std::endl;
      return false;
    }

    std::cout << "Loaded " << cloud.size() << " points from " << pcd_path << std::endl;

    // Prepare data for detector
    std::vector<float> points;
    std::vector<int> indices;
    points.reserve(cloud.size() * 3);
    indices.reserve(cloud.size());

    // Calculate bounding box
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    for (const auto& pt : cloud) {
      min_x = std::min(min_x, pt.x);
      max_x = std::max(max_x, pt.x);
      min_y = std::min(min_y, pt.y);
      max_y = std::max(max_y, pt.y);
      min_z = std::min(min_z, pt.z);
      max_z = std::max(max_z, pt.z);
    }

    std::cout << "Bounding Box: X=[" << min_x << ", " << max_x << "]"
              << " Y=[" << min_y << ", " << max_y << "]"
              << " Z=[" << min_z << ", " << max_z << "]" << std::endl;

    // Use system's center (lidar2world_pose.translation) if provided
    // This must match what the system used for ground detection!
    float center_x, center_y, center_z;
    if (config_.vehicle_x != 0.0f || config_.vehicle_y != 0.0f || config_.vehicle_z != 0.0f) {
      // Use system's center (vehicle pose)
      center_x = config_.vehicle_x;
      center_y = config_.vehicle_y;
      center_z = config_.vehicle_z;
      std::cout << "Using system center (from vehicle pose): [" << center_x << ", " << center_y << ", " << center_z << "]" << std::endl;
    } else {
      // Fallback: calculate average as center
      float sum_x = 0, sum_y = 0, sum_z = 0;
      for (const auto& pt : cloud) {
        sum_x += pt.x;
        sum_y += pt.y;
        sum_z += pt.z;
      }
      center_x = sum_x / cloud.size();
      center_y = sum_y / cloud.size();
      center_z = sum_z / cloud.size();
      std::cout << "Using calculated center (average): [" << center_x << ", " << center_y << ", " << center_z << "]" << std::endl;
    }

    // Estimate slope
    float xy_range = std::sqrt(std::pow(max_x - min_x, 2) + std::pow(max_y - min_y, 2));
    float z_range = max_z - min_z;
    float slope_deg = std::atan2(z_range, xy_range) * 180.0f / M_PI;
    std::cout << "Estimated slope: " << slope_deg << " degrees" << std::endl;

    // Center points and prepare data (using SAME center as system!)
    for (size_t i = 0; i < cloud.size(); ++i) {
      const auto& pt = cloud[i];
      points.push_back(pt.x - center_x);
      points.push_back(pt.y - center_y);
      points.push_back(pt.z - center_z);
      indices.push_back(static_cast<int>(i));
    }

    // Run ground detection
    std::vector<float> heights(cloud.size());
    bool success = detector_->Detect(points.data(), heights.data(),
                                     cloud.size(), 3);

    if (!success) {
      std::cerr << "Ground detection failed!" << std::endl;
      return false;
    }

    // Collect and print debug information
    DetectionDebugInfo debug_info = CollectDetectionDebug(cloud, heights);
    debug_info.Print();

    // Additional analysis for non-JSON mode
    float grid_size = config_.roi_rad_x * 2.0f / config_.small_grid_size;
    std::cout << "\nGround Fitting Analysis:" << std::endl;
    std::cout << "  Grid cell size: " << grid_size << "m x " << grid_size << "m" << std::endl;

    std::cout << "\n  Height variation in one grid for different slopes:" << std::endl;
    for (int slope : {5, 10, 15, 20}) {
      float h_var = grid_size * std::tan(slope * M_PI / 180.0f);
      std::cout << "    " << slope << "° slope: " << h_var << "m";
      if (h_var > config_.planefit_dist_thres_near) {
        std::cout << " ❌ EXCEEDS threshold (" << config_.planefit_dist_thres_near << "m)";
      } else {
        std::cout << " ✓ OK";
      }
      std::cout << std::endl;
    }

    // Generate visualization data
    GenerateVisualization(cloud, heights, center_x, center_y, center_z,
                         slope_deg, grid_size, output_path);

    // Save PCD files if requested
    if (!save_pcd_path.empty()) {
      SaveResultPCD(cloud, heights, save_pcd_path);
    }

    return true;
  }

  bool SaveResultPCD(const pcl::PointCloud<PCLPointXYZIT>& cloud,
                     const std::vector<float>& heights,
                     const std::string& output_prefix) {
    // Separate ground and non-ground points using ADAPTIVE threshold
    // Input cloud is already CENTERED (pt - center), same as system uses!
    // Apollo coordinate: X=right, Y=forward, Z=up
    pcl::PointCloud<PCLPointXYZIT> ground_cloud;
    pcl::PointCloud<PCLPointXYZIT> nonground_cloud;
    pcl::PointCloud<PCLPointXYZIT> labeled_cloud;

    ground_cloud.reserve(cloud.size());
    nonground_cloud.reserve(cloud.size());
    labeled_cloud.reserve(cloud.size());

    int ground_count = 0;

    for (size_t i = 0; i < cloud.size(); ++i) {
      const auto& pt = cloud[i];
      float h = heights[i];

      // Input is already centered (relative coordinates), use Y directly for forward distance
      // Apollo: Y is forward
      float threshold = config_.ground_thres;
      const float forward_dist = pt.y;

      if (forward_dist > 0.0f && forward_dist < config_.near_range_dist) {
        threshold = config_.near_range_ground_thres;
      } else if (forward_dist >= config_.near_range_dist &&
                 forward_dist < config_.middle_range_dist) {
        threshold = config_.middle_range_ground_thres;
      }

      bool is_ground = h <= threshold;

      // Add to labeled cloud with intensity indicating ground/non-ground
      PCLPointXYZIT labeled_pt = pt;
      labeled_pt.intensity = is_ground ? 1 : 255;
      labeled_cloud.push_back(labeled_pt);

      // Add to respective cloud
      if (is_ground) {
        ground_cloud.push_back(pt);
        ground_count++;
      } else {
        nonground_cloud.push_back(pt);
      }
    }

    // Save PCD files
    pcl::PCDWriter writer;

    std::string ground_path = output_prefix + "_ground.pcd";
    writer.writeBinary(ground_path, ground_cloud);
    std::cout << "Saved ground points: " << ground_path
              << " (" << ground_cloud.size() << " points)" << std::endl;

    std::string nonground_path = output_prefix + "_nonground.pcd";
    writer.writeBinary(nonground_path, nonground_cloud);
    std::cout << "Saved non-ground points: " << nonground_path
              << " (" << nonground_cloud.size() << " points)" << std::endl;

    std::string labeled_path = output_prefix + "_labeled.pcd";
    writer.writeBinary(labeled_path, labeled_cloud);
    std::cout << "Saved labeled points: " << labeled_path
              << " (" << labeled_cloud.size() << " points)" << std::endl;

    return true;
  }

 private:
  // Collect detailed detection debug information
  DetectionDebugInfo CollectDetectionDebug(const pcl::PointCloud<PCLPointXYZIT>& cloud,
                                          const std::vector<float>& heights) {
    DetectionDebugInfo info;

    // Get grid information
    info.grid_dim_x = detector_->GetGridDimX();
    info.grid_dim_y = detector_->GetGridDimY();

    // Get grid to check point distribution
    // Note: GetGrid() returns const pointer but GetVoxels() is non-const
    // We need to cast away const to access voxels
    common::VoxelGridXY<float>* grid = const_cast<common::VoxelGridXY<float>*>(detector_->GetGrid());
    std::vector<common::Voxel<float>>& voxels = grid->GetVoxels();

    for (unsigned int r = 0; r < info.grid_dim_y; ++r) {
      for (unsigned int c = 0; c < info.grid_dim_x; ++c) {
        unsigned int idx = r * info.grid_dim_x + c;
        const common::Voxel<float>& voxel = voxels[idx];
        int nr_points = voxel.NrPoints();
        if (nr_points > 0) {
          info.cells_with_points++;
          if (nr_points > info.max_points_in_cell) info.max_points_in_cell = nr_points;
        } else {
          info.empty_cells++;
        }
      }
    }

    // Analyze ground planes
    for (unsigned int r = 0; r < info.grid_dim_y; ++r) {
      for (unsigned int c = 0; c < info.grid_dim_x; ++c) {
        const common::GroundPlaneLiDAR* plane = detector_->GetGroundPlane(r, c);
        if (plane && plane->IsValid()) {
          info.valid_planes++;
          info.total_support += plane->GetNrSupport();
          // Collect first few planes for display
          if (info.planes.size() < 10) {
            DetectionDebugInfo::PlaneInfo pi;
            pi.r = r;
            pi.c = c;
            pi.nx = plane->params[0];
            pi.ny = plane->params[1];
            pi.nz = plane->params[2];
            pi.d = plane->params[3];
            pi.support = plane->GetNrSupport();
            pi.angle_to_x = plane->GetDegreeNormalToX();
            info.planes.push_back(pi);
          }
        } else {
          info.failed_cells++;
        }
      }
    }

    // Count ground vs non-ground
    for (size_t i = 0; i < cloud.size(); ++i) {
      if (heights[i] > config_.ground_thres) {
        info.nonground_count++;
      } else {
        info.ground_count++;
      }
    }

    // Height statistics
    info.min_height = heights[0];
    info.max_height = heights[0];
    for (float h : heights) {
      if (h < info.min_height) info.min_height = h;
      if (h > info.max_height) info.max_height = h;
    }

    return info;
  }

  void GenerateVisualization(const pcl::PointCloud<PCLPointXYZIT>& cloud,
                            const std::vector<float>& heights,
                            float center_x, float center_y, float center_z,
                            float slope_deg, float grid_size,
                            const std::string& output_path) {
    // Create JSON data for visualization
    std::stringstream json;
    json << "{\n";
    json << "  \"points\": [\n";

    for (size_t i = 0; i < cloud.size(); ++i) {
      const auto& pt = cloud[i];
      float h = heights[i];
      bool is_ground = h <= config_.ground_thres;

      json << "    [" << pt.x << "," << pt.y << "," << pt.z << ","
           << (is_ground ? "1" : "0") << "]";
      if (i < cloud.size() - 1) json << ",";
      json << "\n";
    }

    json << "  ],\n";
    json << "  \"config\": {\n";
    json << "    \"small_grid_size\": " << config_.small_grid_size << ",\n";
    json << "    \"roi_rad_x\": " << config_.roi_rad_x << ",\n";
    json << "    \"planefit_dist_thres_near\": " << config_.planefit_dist_thres_near << ",\n";
    json << "    \"planefit_dist_thres_far\": " << config_.planefit_dist_thres_far << ",\n";
    json << "    \"ground_thres\": " << config_.ground_thres << ",\n";
    json << "    \"planefit_orien_threshold\": " << config_.planefit_orien_threshold << "\n";
    json << "  },\n";
    json << "  \"analysis\": {\n";
    json << "    \"slope_degrees\": " << slope_deg << ",\n";
    json << "    \"grid_size\": " << grid_size << ",\n";
    json << "    \"total_points\": " << cloud.size() << ",\n";

    int ground_count = 0;
    for (float h : heights) {
      if (h <= config_.ground_thres) ground_count++;
    }
    json << "    \"ground_points\": " << ground_count << ",\n";
    json << "    \"nonground_points\": " << (cloud.size() - ground_count) << "\n";
    json << "  }\n";
    json << "}\n";

    // Write JSON file
    std::string json_path = output_path + ".json";
    std::ofstream json_file(json_path);
    json_file << json.str();
    json_file.close();
    std::cout << "Saved visualization data to: " << json_path << std::endl;

    // HTML generation removed - using web interface instead
  }


  GroundAnalyzerConfig config_;
  common::PlaneFitGroundDetectorParam param_;
  std::unique_ptr<common::PlaneFitGroundDetector> detector_;
};

}  // namespace tools
}  // namespace lidar
}  // namespace perception
}  // namespace apollo

int main(int argc, char** argv) {
  // Check for help first
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " <pcd_file> [options]" << std::endl;
      std::cout << "\nOptions:" << std::endl;
      std::cout << "  --output <prefix>       Output prefix for JSON and HTML files" << std::endl;
      std::cout << "  --json                  Output JSON to stdout (for web API)" << std::endl;
      std::cout << "  --save_pcd <prefix>     Save ground/nonground/labeled PCD files" << std::endl;
      std::cout << "  --small_grid_size <N>   Number of coarse grids (default: 32)" << std::endl;
      std::cout << "  --big_grid_size <N>     Number of fine grids (default: 256)" << std::endl;
      std::cout << "  --nr_smooth_iter <N>      Number of smooth iterations (default: 5)" << std::endl;
      std::cout << "  --roi_rad_x <F>      ROI radius X (default: 120.0)" << std::endl;
      std::cout << "  --roi_rad_y <F>      ROI radius Y (default: 120.0)" << std::endl;
      std::cout << "  --roi_rad_z <F>      ROI radius Z (default: 100.0)" << std::endl;
      std::cout << "  --roi_near_rad <F>   ROI near radius (default: 32.0)" << std::endl;
      std::cout << "  --planefit_dist_thres_near <F>   Near distance threshold (default: 0.10)" << std::endl;
      std::cout << "  --planefit_dist_thres_far <F>    Far distance threshold (default: 0.20)" << std::endl;
      std::cout << "  --planefit_orien_threshold <F>   Orientation threshold (default: 5.0)" << std::endl;
      std::cout << "  --z_compare_thres <F>     Z compare threshold (default: 0.1)" << std::endl;
      std::cout << "  --smooth_z_thres <F>      Smooth Z threshold (default: 1.0)" << std::endl;
      std::cout << "  --ground_thres <F>        Ground height threshold (default: 3.0)" << std::endl;
      std::cout << "  --vehicle_x <F>           Vehicle position X in world coords (default: 0.0)" << std::endl;
      std::cout << "  --vehicle_y <F>           Vehicle position Y in world coords (default: 0.0)" << std::endl;
      std::cout << "  --vehicle_z <F>           Vehicle position Z in world coords (default: 0.0)" << std::endl;
      std::cout << "  --vehicle_pose_file <F>   Load vehicle pose from file (e.g., 000006_vehicle_*.txt)" << std::endl;
      std::cout << "\nExample:" << std::endl;
      std::cout << "  " << argv[0] << " input.pcd --json --small_grid_size 64 --planefit_dist_thres_near 0.25" << std::endl;
      std::cout << "  " << argv[0] << " input.pcd --save_pcd result --small_grid_size 64" << std::endl;
      return 0;
    }
  }

  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <pcd_file> [options]" << std::endl;
    std::cout << "Use --help for more information" << std::endl;
    return 1;
  }

  // First argument should be PCD file (skip options)
  int pcd_file_idx = -1;
  std::string output_prefix = "ground_analysis";
  std::string save_pcd_prefix = "";
  bool json_mode = false;

  // Parse command line options
  apollo::perception::lidar::tools::GroundAnalyzerConfig config;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--json") {
      json_mode = true;
    } else if (arg == "--output" && i + 1 < argc) {
      output_prefix = argv[++i];
    } else if (arg == "--save_pcd" && i + 1 < argc) {
      save_pcd_prefix = argv[++i];
    } else if (arg == "--small_grid_size" && i + 1 < argc) {
      config.small_grid_size = std::stoul(argv[++i]);
    } else if (arg == "--big_grid_size" && i + 1 < argc) {
      config.big_grid_size = std::stoul(argv[++i]);
    } else if (arg == "--grid_size" && i + 1 < argc) {
      config.grid_size = std::stoul(argv[++i]);
    } else if (arg == "--nr_smooth_iter" && i + 1 < argc) {
      config.nr_smooth_iter = std::stoul(argv[++i]);
    } else if (arg == "--roi_rad_x" && i + 1 < argc) {
      config.roi_rad_x = std::stof(argv[++i]);
    } else if (arg == "--roi_rad_y" && i + 1 < argc) {
      config.roi_rad_y = std::stof(argv[++i]);
    } else if (arg == "--roi_rad_z" && i + 1 < argc) {
      config.roi_rad_z = std::stof(argv[++i]);
    } else if (arg == "--roi_near_rad" && i + 1 < argc) {
      config.roi_near_rad = std::stof(argv[++i]);
    } else if (arg == "--sample_region_z_lower" && i + 1 < argc) {
      config.sample_region_z_lower = std::stof(argv[++i]);
    } else if (arg == "--sample_region_z_upper" && i + 1 < argc) {
      config.sample_region_z_upper = std::stof(argv[++i]);
    } else if (arg == "--planefit_dist_thres_near" && i + 1 < argc) {
      config.planefit_dist_thres_near = std::stof(argv[++i]);
    } else if (arg == "--planefit_dist_thres_far" && i + 1 < argc) {
      config.planefit_dist_thres_far = std::stof(argv[++i]);
    } else if (arg == "--planefit_orien_threshold" && i + 1 < argc) {
      config.planefit_orien_threshold = std::stof(argv[++i]);
    } else if (arg == "--inliers_min_threshold" && i + 1 < argc) {
      config.inliers_min_threshold = std::stoul(argv[++i]);
    } else if (arg == "--z_compare_thres" && i + 1 < argc) {
      config.z_compare_thres = std::stof(argv[++i]);
    } else if (arg == "--smooth_z_thres" && i + 1 < argc) {
      config.smooth_z_thres = std::stof(argv[++i]);
    } else if (arg == "--ground_thres" && i + 1 < argc) {
      config.ground_thres = std::stof(argv[++i]);
    } else if (arg == "--near_range_dist" && i + 1 < argc) {
      config.near_range_dist = std::stof(argv[++i]);
    } else if (arg == "--near_range_ground_thres" && i + 1 < argc) {
      config.near_range_ground_thres = std::stof(argv[++i]);
    } else if (arg == "--middle_range_dist" && i + 1 < argc) {
      config.middle_range_dist = std::stof(argv[++i]);
    } else if (arg == "--middle_range_ground_thres" && i + 1 < argc) {
      config.middle_range_ground_thres = std::stof(argv[++i]);
    } else if (arg == "--vehicle_x" && i + 1 < argc) {
      config.vehicle_x = std::stof(argv[++i]);
    } else if (arg == "--vehicle_y" && i + 1 < argc) {
      config.vehicle_y = std::stof(argv[++i]);
    } else if (arg == "--vehicle_z" && i + 1 < argc) {
      config.vehicle_z = std::stof(argv[++i]);
    } else if (arg == "--vehicle_pose_file" && i + 1 < argc) {
      // Read vehicle pose from file (generated by system)
      std::string pose_file = argv[++i];
      std::ifstream pf(pose_file);
      if (pf.is_open()) {
        std::string line;
        while (std::getline(pf, line)) {
          if (line.empty() || line[0] == '#') continue;  // Skip comments
          std::istringstream iss(line);
          if (iss >> config.vehicle_x >> config.vehicle_y >> config.vehicle_z) {
            break;  // Read first data line
          }
        }
        pf.close();
        std::cout << "Loaded vehicle pose from file: "
                  << config.vehicle_x << ", " << config.vehicle_y << ", " << config.vehicle_z << std::endl;
      } else {
        std::cerr << "Warning: Failed to open vehicle pose file: " << pose_file << std::endl;
      }
    } else if (arg[0] != '-' && pcd_file_idx < 0) {
      // This is the PCD file (first non-option argument)
      pcd_file_idx = i;
    }
  }

  if (pcd_file_idx < 1) {
    std::cerr << "Error: No PCD file specified" << std::endl;
    return 1;
  }

  std::string pcd_file = argv[pcd_file_idx];

  apollo::perception::lidar::tools::GroundAnalyzer analyzer(config);

  if (json_mode) {
    // Run in JSON mode (output to stdout for web API)
    if (!analyzer.AnalyzeAndOutputJSON(pcd_file, std::cout, save_pcd_prefix)) {
      std::cerr << "Analysis failed!" << std::endl;
      return 1;
    }
  } else {
    // Run in file output mode
    config.Print();
    if (!analyzer.Analyze(pcd_file, output_prefix, save_pcd_prefix)) {
      std::cerr << "Analysis failed!" << std::endl;
      return 1;
    }
    std::cout << "\nOpen " << output_prefix << ".html in your browser to view the visualization." << std::endl;
  }

  return 0;
}
