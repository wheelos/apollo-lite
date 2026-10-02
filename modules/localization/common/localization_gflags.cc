/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/common/localization_gflags.h"

DEFINE_string(local_map_name, "local_map", "The path of localization map.");
DEFINE_string(lidar_height_file,
              "/apollo/modules/ndt_localization/conf/"
              "velodyne64_height.yaml",
              "LiDAR height configuration path.");
DEFINE_double(lidar_height_default, 1.80,
              "The height from the center of velodyne to ground.");
DEFINE_bool(if_utm_zone_id_from_folder, true,
            "load utm zone id from local map folder");
DEFINE_bool(enable_lidar_localization, true,
            "Enable lidar-based localization.");
DEFINE_string(lidar_topic, "/apollo/sensor/lidar128/compensator/PointCloud2",
              "lidar pointcloud topic");
DEFINE_string(broadcast_tf_frame_id, "map", "global frame id in tf");
DEFINE_string(broadcast_tf_child_frame_id, "base_link",
              "vehicle pose frame id in tf");
DEFINE_string(ndt_map_dir, "ndt_map", "subdirectory for ndt map");
DEFINE_bool(ndt_debug_log_flag, false, "NDT Localization log switch");
DEFINE_double(online_resolution, 2.0, "NDT online pointcloud resolution");
DEFINE_int32(ndt_max_iterations, 10, "maximum iterations for NDT matching");
DEFINE_double(ndt_target_resolution, 1.0,
              "target resolution for ndt localization");
DEFINE_double(ndt_line_search_step_size, 0.1,
              "line search step size for ndt matching");
DEFINE_double(ndt_transformation_epsilon, 0.01,
              "iteration convergence condition on transformation");
DEFINE_int32(ndt_filter_size_x, 48, "x size for ndt searching area");
DEFINE_int32(ndt_filter_size_y, 48, "y size for ndt searching area");
DEFINE_int32(ndt_bad_score_count_threshold, 10,
             "count for continuous bad ndt fitness score");
DEFINE_double(ndt_warnning_ndt_score, 1.0,
              "warnning ndt fitness score threshold");
DEFINE_double(ndt_error_ndt_score, 2.0, "error ndt fitness score threshold");
