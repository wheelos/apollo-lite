/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar/lib/detector/graph_segmentation/graph_segmentation.h"

namespace apollo {
namespace perception {
namespace lidar {

bool GraphSegmentation::Init(const LidarDetectorInitOptions& options) {
  // Use default config path under this module; runtime can override if needed.
  return true;
}

bool GraphSegmentation::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }

  stage_conf_ = stage_config;

  // get config
  resolution_ = stage_conf_.graph_segmentation_config().resolution();
  threshold_ = stage_conf_.graph_segmentation_config().threshold();
  min_pt_number_ = stage_conf_.graph_segmentation_config().min_pt_number();
  search_radius_ = stage_conf_.graph_segmentation_config().search_radius();
  height_threshold_ =
      stage_conf_.graph_segmentation_config().height_threshold();
  semantic_cost_ = stage_conf_.graph_segmentation_config().semantic_cost();

  // initialize graph segmentor
  graph_segmentor_.Init(threshold_);

  ACHECK(bg_map_.Init(
      stage_conf_.graph_segmentation_config().xmin(),
      stage_conf_.graph_segmentation_config().xmax(),
      stage_conf_.graph_segmentation_config().ymin(),
      stage_conf_.graph_segmentation_config().ymax(), resolution_,
      height_threshold_, stage_conf_.graph_segmentation_config().car_xmin(),
      stage_conf_.graph_segmentation_config().car_xmax(),
      stage_conf_.graph_segmentation_config().car_ymin(),
      stage_conf_.graph_segmentation_config().car_ymax(),
      stage_conf_.graph_segmentation_config().car_zmax(),
      stage_conf_.graph_segmentation_config().min_radius(),
      stage_conf_.graph_segmentation_config().z_min_from_ground()));
  grid_width_ =
      static_cast<int>((stage_conf_.graph_segmentation_config().ymax() -
                        stage_conf_.graph_segmentation_config().ymin()) *
                       1.0f / resolution_);
  grid_height_ =
      static_cast<int>((stage_conf_.graph_segmentation_config().xmax() -
                        stage_conf_.graph_segmentation_config().xmin()) *
                       1.0f / resolution_);

  // clear background objects
  bg_objects_.clear();

  // Fall back to default Init without external config path.
  LidarDetectorInitOptions init_options;
  return Init(init_options);
}

bool GraphSegmentation::Process(DataFrame* data_frame) {
  if (data_frame == nullptr || data_frame->lidar_frame == nullptr) {
    return false;
  }
  LidarDetectorOptions options;
  return Detect(options, data_frame->lidar_frame);
}

bool GraphSegmentation::Detect(const LidarDetectorOptions& options,
                               LidarFrame* frame) {
  // clear background objects buffer
  bg_objects_.clear();

  // Input sizes for analysis
  AINFO << "GraphSeg Detect: cloud: " << frame->cloud->size()
        << ", non_ground: " << frame->non_ground_indices.indices.size()
        << ", roi: " << frame->roi_indices.indices.size()
        << ", secondary: " << frame->secondary_indices.indices.size();

  // reset background map
  if (!bg_map_.Reset(frame->cloud->size())) {
    AERROR << "bg_map_ Reset error.";
    return false;
  }
  // update map
  if (!bg_map_.UpdateMap(frame)) {
    AERROR << "bg_map_ Update error.";
    return false;
  }
  // cluster
  if (!Segment(frame)) {
    AERROR << "Graph Segment error.";
    return false;
  }
  // split
  if (!Split(frame)) {
    AERROR << "Split object error.";
    return false;
  }
  // set default value and id for bg_objects_
  if (!SetBgObjectDefaultVal(frame)) {
    AERROR << "SetBgObjectDefaultVal error.";
    return false;
  }

  // get all background objects
  frame->segmented_objects.insert(frame->segmented_objects.end(),
                                  bg_objects_.begin(), bg_objects_.end());
  AINFO << "Graph segment, background objects number: " << bg_objects_.size();
  return true;
}

bool GraphSegmentation::Segment(LidarFrame* frame) {
  std::vector<common::Edge> edge_set;
  edge_set.clear();
  common::Edge edge;
  // Log valid nodes before building edges
  AINFO << "Segment: valid_nodes(before edges): " << bg_map_.id()
        << ", grid(h,w): (" << grid_height_ << "," << grid_width_ << ")"
        << ", resolution: " << resolution_
        << ", search_radius: " << search_radius_;
  for (int x = 0; x < grid_height_; ++x) {
    for (int y = 0; y < grid_width_; ++y) {
      int grid_idx = x * grid_width_ + y;
      if (!bg_map_.bg_nodes_.at(grid_idx).is_valid) {
        continue;
      }

      edge.a = bg_map_.bg_nodes_.at(grid_idx).id;
      BgNode n1 = bg_map_.bg_nodes_.at(grid_idx);

      // Only connect to 8-neighborhood, but prevent diagonal-only corner
      // bridges. Precompute orthogonal neighbor validity.
      bool right_valid = false;
      bool left_valid = false;
      bool bottom_valid = false;

      // right neighbor (x, y+1)
      if (y + 1 < grid_width_) {
        int grid_ny = x * grid_width_ + (y + 1);
        if (bg_map_.bg_nodes_.at(grid_ny).is_valid) {
          right_valid = true;
          edge.b = bg_map_.bg_nodes_.at(grid_ny).id;
          BgNode n2 = bg_map_.bg_nodes_.at(grid_ny);
          edge.w = NodeDistance(n1, x, y, n2, x, y + 1);
          edge_set.push_back(edge);
        }
      }
      // left neighbor (x, y-1) - only for diagonal gating decision
      if (y - 1 >= 0) {
        int grid_ly = x * grid_width_ + (y - 1);
        if (bg_map_.bg_nodes_.at(grid_ly).is_valid) {
          left_valid = true;
        }
      }
      // bottom (x+1, y)
      if (x + 1 < grid_height_) {
        int grid_nxny = (x + 1) * grid_width_ + y;
        if (bg_map_.bg_nodes_.at(grid_nxny).is_valid) {
          bottom_valid = true;
          edge.b = bg_map_.bg_nodes_.at(grid_nxny).id;
          BgNode n2 = bg_map_.bg_nodes_.at(grid_nxny);
          edge.w = NodeDistance(n1, x, y, n2, x + 1, y);
          edge_set.push_back(edge);
        }
      }
      // bottom-left (x+1, y-1) - require at least one orth neighbor valid
      if (x + 1 < grid_height_ && y - 1 >= 0) {
        int grid_nxny = (x + 1) * grid_width_ + (y - 1);
        if (bg_map_.bg_nodes_.at(grid_nxny).is_valid &&
            (bottom_valid || left_valid)) {
          edge.b = bg_map_.bg_nodes_.at(grid_nxny).id;
          BgNode n2 = bg_map_.bg_nodes_.at(grid_nxny);
          edge.w = NodeDistance(n1, x, y, n2, x + 1, y - 1);
          edge_set.push_back(edge);
        }
      }
      // bottom-right (x+1, y+1) - require at least one orth neighbor valid
      if (x + 1 < grid_height_ && y + 1 < grid_width_) {
        int grid_nxny = (x + 1) * grid_width_ + (y + 1);
        if (bg_map_.bg_nodes_.at(grid_nxny).is_valid &&
            (bottom_valid || right_valid)) {
          edge.b = bg_map_.bg_nodes_.at(grid_nxny).id;
          BgNode n2 = bg_map_.bg_nodes_.at(grid_nxny);
          edge.w = NodeDistance(n1, x, y, n2, x + 1, y + 1);
          edge_set.push_back(edge);
        }
      }
    }
  }

  if (edge_set.size() == 0) {
    AINFO << "Graph segmentation, edges are empty.";
    return true;
  }
  AINFO << "Segment: built edges: " << edge_set.size();

  // graph segment
  graph_segmentor_.SegmentGraph(bg_map_.id(), edge_set.size(), edge_set.data());
  common::Universe* universe = graph_segmentor_.mutable_universe();
  std::vector<size_t> mapping(bg_map_.id(), 0);
  std::map<size_t, size_t> label_map;
  size_t label = 0;
  for (int i = 0; i < bg_map_.id(); ++i) {
    mapping[i] = universe->Find(i);
    auto iter = label_map.find(mapping[i]);
    if (iter == label_map.end()) {
      // key:value --> father:label
      label_map.emplace(mapping[i], label);
      mapping[i] = label;
      label++;
    } else {
      mapping[i] = iter->second;
    }
  }
  AINFO << "Segment: raw_clusters(before filtering by min_pt): "
        << label_map.size();

  // get point clusters
  auto original_cloud = frame->cloud;
  // SppClusterList clusters;
  clusters_.Reset();
  clusters_.resize(label_map.size());
  for (size_t i = 0; i < bg_map_.point_mask_.size(); ++i) {
    if (!bg_map_.point_mask_.at(i)) {
      continue;
    }
    size_t index = bg_map_.point_idx_.at(i);
    size_t cluster_label = mapping.at(bg_map_.bg_nodes_.at(index).id);
    clusters_.AddPointSample(cluster_label, original_cloud->at(i),
                             original_cloud->points_height(i), i);
  }
  GetObjectsFromClusters(frame);

  for (int i = 0; i <= clusters_.size() - 1; i++) {
    clusters_[i]->clear();
  }
  clusters_.Reset();

  return true;
}

float GraphSegmentation::NodeDistance(BgNode n1, int x1, int y1, BgNode n2,
                                      int x2, int y2) {
  float x_diff = static_cast<float>(x1 - x2) * resolution_;
  float y_diff = static_cast<float>(y1 - y2) * resolution_;
  float mean_height_diff = n1.mean_height - n2.mean_height;
  float max_height_diff = n1.max_height - n2.max_height;
  float intensity_diff = (n1.mean_intensity - n2.mean_intensity) / 255.0f;

  // Hard gate on vertical discontinuity to prevent merges across object-ground
  // boundaries when ground removal is disabled. Use semantic_cost_ as gate in
  // meters.
  float h_gate = static_cast<float>(semantic_cost_);
  if (h_gate <= 0.0f) {
    h_gate = 0.35f;  // default gate if not set in config
  }
  if (std::fabs(max_height_diff) > h_gate) {
    // Return a very large cost to avoid merging these nodes.
    return 1e6f;
  }

  // Weighted cost: emphasize height terms moderately; keep intensity weak
  const float w_h_mean = 2.0f;
  const float w_h_max = 3.0f;
  const float w_inten = 0.5f;
  float total_cost = std::sqrt(x_diff * x_diff + y_diff * y_diff +
                               w_h_mean * mean_height_diff * mean_height_diff +
                               w_h_max * max_height_diff * max_height_diff +
                               w_inten * intensity_diff * intensity_diff);

  // Add density-based penalty to suppress merges in very sparse regions
  int min_pts = std::min(n1.point_number, n2.point_number);
  if (min_pts <= 1) {
    total_cost += 1.0f;
  } else if (min_pts <= 2) {
    total_cost += 0.6f;
  } else if (min_pts <= 4) {
    total_cost += 0.3f;
  }
  return total_cost;
}

void GraphSegmentation::GetObjectsFromClusters(LidarFrame* frame) {
  // point cloud
  auto original_cloud_ = frame->cloud;
  auto original_world_cloud_ = frame->world_cloud;
  // clear background objects
  bg_objects_.clear();
  // background object
  for (size_t i = 0; i < clusters_.size(); ++i) {
    auto cluster = clusters_[i];
    if (cluster->point_ids.size() < min_pt_number_) {
      continue;
    }
    base::Object object;
    object.confidence = cluster->confidence;
    object.lidar_supplement.num_points_in_roi = cluster->point_ids.size();
    object.lidar_supplement.cloud.CopyPointCloud(*original_cloud_,
                                                 cluster->point_ids);
    object.lidar_supplement.cloud_world.CopyPointCloud(*original_world_cloud_,
                                                       cluster->point_ids);
    // copy to background objects
    std::shared_ptr<base::Object> obj(new base::Object);
    *obj = object;
    bg_objects_.push_back(std::move(obj));
  }
  // background object builder
  BgObjectBuilder(&bg_objects_, frame->lidar2novatel_extrinsics);
}

bool GraphSegmentation::Split(LidarFrame* frame) {
  std::vector<std::shared_ptr<base::Object>> split_objects;
  // record object number
  int before_split_num = bg_objects_.size();

  int valid_number = 0;
  for (size_t i = 0; i < bg_objects_.size(); i++) {
    std::shared_ptr<base::Object> obj = bg_objects_.at(i);
    if (NeedSplit(obj)) {
      std::vector<base::ObjectPtr> this_split;
      SplitObject(obj, &this_split,
                  stage_conf_.graph_segmentation_config().split_distance());
      BgObjectBuilder(&this_split, frame->lidar2novatel_extrinsics);
      split_objects.insert(split_objects.end(), this_split.begin(),
                           this_split.end());
      continue;
    }
    bg_objects_.at(valid_number) = obj;
    valid_number += 1;
  }
  int split_number = bg_objects_.size() - valid_number;
  bg_objects_.resize(valid_number);

  // put all split object to bg_objects_
  bg_objects_.insert(bg_objects_.end(), split_objects.begin(),
                     split_objects.end());
  int after_split_num = bg_objects_.size();

  AINFO << "Before split object number: " << before_split_num
        << ", split number: " << split_number
        << ", after split number: " << after_split_num;
  return true;
}

bool GraphSegmentation::NeedSplit(std::shared_ptr<base::Object> object) {
  // get length and width
  float length, width;
  if (object->size(0) > object->size(1)) {
    length = object->size(0);
    width = object->size(1);
  } else {
    length = object->size(1);
    width = object->size(0);
  }
  // additional guards: enough points and sufficiently long to split
  const size_t pts = object->lidar_supplement.cloud.size();
  const float split_dist =
      stage_conf_.graph_segmentation_config().split_distance();
  bool aspect_ok =
      (width > 1e-6f) &&
      (length / (width + 1e-6f) >
       stage_conf_.graph_segmentation_config().split_aspect_ratio());
  bool long_enough = (length >= 2.0f * split_dist);
  bool enough_points = (pts >= std::max<size_t>(min_pt_number_, 48));
  if (aspect_ok && long_enough && enough_points) {
    return true;
  }
  return false;
}

bool GraphSegmentation::SetBgObjectDefaultVal(LidarFrame* frame) {
  int start_index = frame->segmented_objects.size();
  for (size_t i = 0; i < bg_objects_.size(); i++) {
    std::shared_ptr<base::Object> object = bg_objects_.at(i);
    object->id = start_index++;
    object->lidar_supplement.is_in_roi = true;
    object->lidar_supplement.is_background = true;
    // mark as background and in-roi only
    // classification
    object->type = base::ObjectType::UNKNOWN;
    object->lidar_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object->lidar_supplement.raw_probs.back()[static_cast<int>(object->type)] =
        1.0f;
    object->lidar_supplement.raw_classification_methods.push_back(Name());
    // copy to type
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(),
                              object->lidar_supplement.raw_probs.back().end());
    // no detections field in 8.0 LidarObjectSupplement
  }
  return true;
}

PERCEPTION_REGISTER_LIDARDETECTOR(GraphSegmentation);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
