/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/**
 * @file camera_util.cc
 **/

#include "modules/perception/fusion/common/camera_util.h"

#include <limits>
#include <algorithm>
#include <cmath>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace fusion {

using apollo::common::EigenVector;

bool ProjectedBoxSimilarity(
    const base::ObjectConstPtr& object, const base::BBox2DF& camera_box,
    const Eigen::Affine3d& camera_to_world,
    const base::BaseCameraModelPtr& camera_model, double* similarity) {
  if (!object || !camera_model || !similarity ||
      !camera_to_world.matrix().allFinite() ||
      !object->direction.allFinite() ||
      object->direction.head<2>().squaredNorm() <= 0) {
    AERROR << "Invalid object-box camera projection.";
    return false;
  }
  *similarity = 0;
  EigenVector<Eigen::Vector3d> vertices;
  GetObjectEightVertices(object, &vertices);
  const Eigen::Matrix4d world_to_camera = camera_to_world.inverse().matrix();
  base::BBox2DF projected;
  projected.xmin = projected.ymin = std::numeric_limits<float>::max();
  projected.xmax = projected.ymax = std::numeric_limits<float>::lowest();
  bool visible = false;
  for (const auto& vertex : vertices) {
    Eigen::Vector2d point;
    if (!Pt3dToCamera2d(vertex, world_to_camera, camera_model, &point)) continue;
    if (!point.allFinite()) {
      AERROR << "Non-finite projected object box.";
      return false;
    }
    visible = true;
    projected.xmin = std::min(projected.xmin, static_cast<float>(point.x()));
    projected.xmax = std::max(projected.xmax, static_cast<float>(point.x()));
    projected.ymin = std::min(projected.ymin, static_cast<float>(point.y()));
    projected.ymax = std::max(projected.ymax, static_cast<float>(point.y()));
  }
  if (!visible) return true;
  const float width = static_cast<float>(camera_model->get_width());
  const float height = static_cast<float>(camera_model->get_height());
  projected.xmin = std::max(0.0f, projected.xmin);
  projected.xmax = std::min(width, projected.xmax);
  projected.ymin = std::max(0.0f, projected.ymin);
  projected.ymax = std::min(height, projected.ymax);
  if (projected.xmax <= projected.xmin || projected.ymax <= projected.ymin) {
    return true;
  }
  *similarity = common::CalculateIOUBBox(projected, camera_box);
  if (!std::isfinite(*similarity) || *similarity < 0 || *similarity > 1) {
    AERROR << "Invalid camera projected-box IoU.";
    return false;
  }
  return true;
}

void GetObjectEightVertices(std::shared_ptr<const base::Object> obj,
                            EigenVector<Eigen::Vector3d>* vertices) {
  vertices->clear();
  vertices->resize(8);
  Eigen::Vector3d center = obj->center;
  Eigen::Vector2d dir = obj->direction.head(2).cast<double>();
  dir.normalize();
  Eigen::Vector2d orth_dir(-dir.y(), dir.x());
  Eigen::Vector2d delta_x = dir * obj->size(0) * 0.5;
  Eigen::Vector2d delta_y = orth_dir * obj->size(1) * 0.5;

  // lower four points
  center.z() -= obj->size(2) * 0.5;
  (*vertices)[0] = center, (*vertices)[0].head(2) += (-delta_x - delta_y);
  (*vertices)[1] = center, (*vertices)[1].head(2) += (-delta_x + delta_y);
  (*vertices)[2] = center, (*vertices)[2].head(2) += (delta_x + delta_y);
  (*vertices)[3] = center, (*vertices)[3].head(2) += (delta_x - delta_y);
  // upper four points
  (*vertices)[4] = (*vertices)[0], (*vertices)[4].z() += obj->size(2);
  (*vertices)[5] = (*vertices)[1], (*vertices)[5].z() += obj->size(2);
  (*vertices)[6] = (*vertices)[2], (*vertices)[6].z() += obj->size(2);
  (*vertices)[7] = (*vertices)[3], (*vertices)[7].z() += obj->size(2);
}

bool Pt3dToCamera2d(const Eigen::Vector3d& pt3d,
                    const Eigen::Matrix4d& world2camera_pose,
                    base::BaseCameraModelPtr camera_model,
                    Eigen::Vector2d* pt2d) {
  Eigen::Vector4d local_pt = static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
      world2camera_pose * Eigen::Vector4d(pt3d(0), pt3d(1), pt3d(2), 1));
  if (local_pt[2] > 0) {
    *pt2d =
        (camera_model->Project(Eigen::Vector3f(
             static_cast<float>(local_pt[0]), static_cast<float>(local_pt[1]),
             static_cast<float>(local_pt[2]))))
            .cast<double>();
    return true;
  }
  return false;
}

bool IsObjectEightVerticesAllBehindCamera(
    const std::shared_ptr<const base::Object>& obj,
    const Eigen::Matrix4d& world2camera_pose,
    base::BaseCameraModelPtr camera_model) {
  EigenVector<Eigen::Vector3d> vertices(8);
  GetObjectEightVertices(obj, &vertices);
  Eigen::Vector2d pt2d;
  for (const auto& vertice : vertices) {
    if ((Pt3dToCamera2d(vertice, world2camera_pose, camera_model, &pt2d))) {
      return false;
    }
  }
  return true;
}

float ObjectInCameraView(SensorObjectConstPtr sensor_object,
                         base::BaseCameraModelPtr camera_model,
                         const Eigen::Affine3d& camera_sensor2world_pose,
                         double camera_ts, double camera_max_dist,
                         bool motion_compensation, bool all_in) {
  constexpr float kFloatEpsilon = std::numeric_limits<float>::epsilon();
  float in_view_ratio = 0.0f;
  Eigen::Matrix4d world2sensor_pose =
      camera_sensor2world_pose.matrix().inverse();
  if (!world2sensor_pose.allFinite()) {
    return in_view_ratio;
  }
  double width = static_cast<double>(camera_model->get_width());
  double height = static_cast<double>(camera_model->get_height());

  double time_diff =
      camera_ts - sensor_object->GetBaseObject()->latest_tracked_time;
  Eigen::Vector3f offset =
      sensor_object->GetBaseObject()->velocity * static_cast<float>(time_diff);
  if (!motion_compensation) {
    offset.setZero();
  }
  // 2.compute distance
  const auto& cloud =
      sensor_object->GetBaseObject()->lidar_supplement.cloud_world;
  Eigen::Vector3d center = sensor_object->GetBaseObject()->center;
  Eigen::Vector2d center2d;
  if (!Pt3dToCamera2d(center, world2sensor_pose, camera_model, &center2d)) {
    return in_view_ratio;
  }
  double obj_length = sensor_object->GetBaseObject()->size(0);
  double obj_width = sensor_object->GetBaseObject()->size(1);
  double obj_height = sensor_object->GetBaseObject()->size(2);
  if (cloud.size() > 0) {
    // use point cloud
    int point_num = static_cast<int>(cloud.size());
    int in_view_point_num = 0;
    for (int i = 0; i < point_num; ++i) {
      const auto& pt = cloud.at(i);
      Eigen::Vector3d pt3d(pt.x + offset[0], pt.y + offset[1],
                           pt.z + offset[2]);
      Eigen::Vector2d pt2d;
      bool flag = Pt3dToCamera2d(pt3d, world2sensor_pose, camera_model, &pt2d);
      if (flag && IsPtInFrustum(pt2d, width, height)) {
        ++in_view_point_num;
      }
    }
    in_view_ratio = static_cast<float>(in_view_point_num / point_num);
    if (all_in) {
      in_view_ratio = (in_view_point_num == point_num) ? 1.0 : 0.0;
    }
  } else if (obj_width > kFloatEpsilon && obj_height > kFloatEpsilon &&
             obj_length > kFloatEpsilon) {
    // use object box
    EigenVector<Eigen::Vector3d> box3d_vs(8);
    GetObjectEightVertices(sensor_object->GetBaseObject(), &box3d_vs);
    EigenVector<Eigen::Vector2f> box2d_vs;
    box2d_vs.reserve(box3d_vs.size());
    for (const auto& box3d_v : box3d_vs) {
      Eigen::Vector2d pt2d;
      bool flag = Pt3dToCamera2d(box3d_v + offset.cast<double>(),
                                 world2sensor_pose, camera_model, &pt2d);
      if (!flag) {
        return in_view_ratio;
      }
      box2d_vs.push_back(pt2d.cast<float>());
    }
    Eigen::Vector2d top_left;
    Eigen::Vector2d bottom_right;
    top_left.setConstant(std::numeric_limits<double>::max());
    bottom_right = -top_left;
    for (const auto& box2d_v : box2d_vs) {
      top_left = top_left.cwiseMin(box2d_v.cast<double>());
      bottom_right = bottom_right.cwiseMax(box2d_v.cast<double>());
    }
    Eigen::Vector2d box_size = bottom_right - top_left;
    Eigen::Vector2d bound_top_left =
        top_left.cwiseMax(Eigen::Vector2d(0.0, 0.0));
    Eigen::Vector2d bound_bottom_right =
        bottom_right.cwiseMin(Eigen::Vector2d(width, height));
    Eigen::Vector2d bound_box_size = bound_bottom_right - bound_top_left;
    if ((bound_box_size.array() > 0.0).all()) {
      in_view_ratio =
          static_cast<float>(bound_box_size.prod() / box_size.prod());
    } else {
      in_view_ratio = 0.0;
    }
    if (all_in) {
      in_view_ratio = std::abs(1.0 - in_view_ratio) <= 1e-6 ? 1.0 : 0.0;
    }
  } else {  // use center point
    if (!((center2d.array() > 0.0).all())) {
      in_view_ratio = 0.0;
    } else if (center2d.x() > width || center2d.y() > height) {
      in_view_ratio = 0.0;
    } else {
      in_view_ratio = 1.0;
    }
  }
  // compute distance score, when object is too far from camera
  // the camera is hard to detect object
  // TODO(yuantingrong):
  // maximum camera detection distance parameters, hard code
  const double max_dist = camera_max_dist;
  // 1 nearly 2m buffer
  const double dist_slope = 0.25;
  auto sigmoid_like_fun = [max_dist, dist_slope](double obj_dist) {
    double x = obj_dist - max_dist;
    return 0.5 - 0.5 * x * dist_slope /
                     std::sqrt(1 + x * x * dist_slope * dist_slope);
  };
  Eigen::Vector4d center3d_local = world2sensor_pose * center.homogeneous();
  double dist_to_camera = center3d_local.z();
  return static_cast<float>(in_view_ratio * sigmoid_like_fun(dist_to_camera));
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
