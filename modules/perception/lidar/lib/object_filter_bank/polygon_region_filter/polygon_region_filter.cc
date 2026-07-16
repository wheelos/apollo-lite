/******************************************************************************
 * Copyright 2026 The WheelOS Authors. All Rights Reserved.
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
#include "modules/perception/lidar/lib/object_filter_bank/polygon_region_filter/polygon_region_filter.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/pipeline/proto/plugin/polygon_region_filter_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using PolygonRegionFilterConfig =
    apollo::perception::lidar::PolygonRegionFilterConfig;

bool PolygonRegionObjectFilter::Init(const ObjectFilterInitOptions& options) {
  // Legacy init path - load from config manager
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));
  const std::string config_dir =
      apollo::cyber::common::GetAbsolutePath(work_root, root_path);
  std::string config_file = apollo::cyber::common::GetAbsolutePath(
      config_dir, "polygon_region_filter.conf");
  PolygonRegionFilterConfig config;
  ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &config));
  return LoadPolygonRegions(config);
}

bool PolygonRegionObjectFilter::Filter(const ObjectFilterOptions& options,
                                       LidarFrame* frame) {
  if (!frame) {
    AINFO << "Lidar frame is nullptr.";
    return false;
  }

  if (exclude_regions_.empty() && include_regions_.empty()) {
    // No regions configured, pass through all objects
    return true;
  }

  const Eigen::Affine3d& pose = frame->lidar2world_pose;
  auto& objects = frame->segmented_objects;

  std::vector<bool> objects_valid_flag(objects.size(), true);

  for (size_t i = 0; i < objects.size(); ++i) {
    const auto& obj = objects[i];

    // Check if object is in exclude regions
    if (!exclude_regions_.empty()) {
      bool in_exclude_region =
          IsObjectInPolygon(obj, exclude_regions_, pose);
      if (in_exclude_region) {
        objects_valid_flag[i] = false;
        ADEBUG << "Polygon region filter: object " << obj->id
               << " filtered out (in exclude region), center "
               << obj->center.head<2>().transpose();
        continue;
      }
    }

    // Check if object is in include regions (if configured)
    if (!include_regions_.empty()) {
      bool in_include_region =
          IsObjectInPolygon(obj, include_regions_, pose);
      if (!in_include_region) {
        objects_valid_flag[i] = false;
        ADEBUG << "Polygon region filter: object " << obj->id
               << " filtered out (not in include region), center "
               << obj->center.head<2>().transpose();
      }
    }
  }

  // Remove filtered objects
  size_t count = 0;
  for (size_t i = 0; i < objects.size(); ++i) {
    if (objects_valid_flag[i]) {
      if (count != i) {
        objects[count] = objects[i];
      }
      ++count;
    }
  }
  objects.resize(count);

  AINFO << "Polygon region filter, filtered objects size: from "
        << objects_valid_flag.size() << " to " << count;
  return true;
}

bool PolygonRegionObjectFilter::Init(const PluginConfig& plugin_config) {
  PolygonRegionFilterConfig config =
      plugin_config.polygon_region_filter_config();
  return LoadPolygonRegions(config);
}

bool PolygonRegionObjectFilter::IsPointInPolygon(
    const Eigen::Vector2d& point, const Polygon2D& polygon) const {
  if (polygon.points.size() < 3) {
    return false;
  }

  // Ray casting algorithm
  bool inside = false;
  size_t n = polygon.points.size();

  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const Eigen::Vector2d& pi = polygon.points[i];
    const Eigen::Vector2d& pj = polygon.points[j];

    // Check if point is on vertex
    if ((point - pi).norm() < 1e-6 || (point - pj).norm() < 1e-6) {
      return true;
    }

    // Check if point is on edge
    double cross = (pi.y() - pj.y()) * point.x() +
                   (pj.x() - pi.x()) * point.y() + pi.x() * pj.y() -
                   pj.x() * pi.y();
    if (std::abs(cross) < 1e-9) {
      // Point is on the line containing the edge
      double dot = (point - pi).dot(point - pj);
      if (dot <= 0) {
        return true;  // Point is on the edge
      }
    }

    // Ray casting: count intersections with ray to the right
    if ((pi.y() > point.y()) != (pj.y() > point.y())) {
      double intersect_x =
          (pj.x() - pi.x()) * (point.y() - pi.y()) / (pj.y() - pi.y()) +
          pi.x();
      if (point.x() < intersect_x) {
        inside = !inside;
      }
    }
  }

  return inside;
}

bool PolygonRegionObjectFilter::IsObjectInPolygon(
    const base::ObjectPtr& object, const std::vector<Polygon2D>& polygons,
    const Eigen::Affine3d& lidar2world) const {
  for (const auto& polygon : polygons) {
    if (IsObjectPolygonInPolygon(object, polygon, lidar2world)) {
      return true;
    }
  }
  return false;
}

bool PolygonRegionObjectFilter::IsObjectPolygonInPolygon(
    const base::ObjectPtr& object, const Polygon2D& polygon,
    const Eigen::Affine3d& lidar2world) const {
  if (use_center_point_only_) {
    return IsObjectCenterInPolygon(object, polygon, lidar2world);
  }

  // Check all polygon points of the object
  for (const auto& point : object->polygon) {
    Eigen::Vector3d local_point(point.x, point.y, point.z);
    Eigen::Vector3d world_point = lidar2world * local_point;

    if (use_world_coordinate_) {
      Eigen::Vector2d point_2d(world_point.x(), world_point.y());
      if (IsPointInPolygon(point_2d, polygon)) {
        return true;
      }
    } else {
      Eigen::Vector2d point_2d(point.x, point.y);
      if (IsPointInPolygon(point_2d, polygon)) {
        return true;
      }
    }
  }

  // Also check center point as fallback
  return IsObjectCenterInPolygon(object, polygon, lidar2world);
}

bool PolygonRegionObjectFilter::IsObjectCenterInPolygon(
    const base::ObjectPtr& object, const Polygon2D& polygon,
    const Eigen::Affine3d& lidar2world) const {
  Eigen::Vector3d center_point;
  if (use_world_coordinate_) {
    center_point = lidar2world * object->center;
  } else {
    center_point = object->center;
  }

  Eigen::Vector2d center_2d(center_point.x(), center_point.y());
  return IsPointInPolygon(center_2d, polygon);
}

bool PolygonRegionObjectFilter::LoadPolygonRegions(
    const PolygonRegionFilterConfig& config) {
  use_center_point_only_ = config.use_center_point_only();
  use_world_coordinate_ = (config.coordinate_system() == "world");

  // Load exclude regions
  exclude_regions_.clear();
  for (const auto& region_proto : config.exclude_regions()) {
    Polygon2D region;
    region.name = region_proto.name();
    for (const auto& point : region_proto.points()) {
      if (point.has_x() && point.has_y()) {
        region.points.push_back(Eigen::Vector2d(point.x(), point.y()));
      }
    }
    if (region.points.size() >= 3) {
      exclude_regions_.push_back(region);
      AINFO << "Loaded exclude region: " << region.name << " with "
            << region.points.size() << " points";
    } else {
      AWARN << "Skip exclude region '" << region.name
            << "' with less than 3 points";
    }
  }

  // Load include regions
  include_regions_.clear();
  for (const auto& region_proto : config.include_regions()) {
    Polygon2D region;
    region.name = region_proto.name();
    for (const auto& point : region_proto.points()) {
      if (point.has_x() && point.has_y()) {
        region.points.push_back(Eigen::Vector2d(point.x(), point.y()));
      }
    }
    if (region.points.size() >= 3) {
      include_regions_.push_back(region);
      AINFO << "Loaded include region: " << region.name << " with "
            << region.points.size() << " points";
    } else {
      AWARN << "Skip include region '" << region.name
            << "' with less than 3 points";
    }
  }

  return true;
}

PERCEPTION_REGISTER_OBJECTFILTER(PolygonRegionObjectFilter);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
