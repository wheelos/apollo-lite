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

#pragma once

#include "gflags/gflags.h"

DECLARE_string(local_map_name);
DECLARE_string(lidar_height_file);
DECLARE_double(lidar_height_default);
DECLARE_bool(if_utm_zone_id_from_folder);
DECLARE_string(lidar_topic);
DECLARE_string(broadcast_tf_frame_id);
DECLARE_string(broadcast_tf_child_frame_id);

DECLARE_string(ndt_map_dir);
DECLARE_bool(ndt_debug_log_flag);
DECLARE_double(online_resolution);
DECLARE_int32(ndt_max_iterations);
DECLARE_double(ndt_target_resolution);
DECLARE_double(ndt_line_search_step_size);
DECLARE_double(ndt_transformation_epsilon);
DECLARE_int32(ndt_filter_size_x);
DECLARE_int32(ndt_filter_size_y);
DECLARE_int32(ndt_bad_score_count_threshold);
DECLARE_double(ndt_warnning_ndt_score);
DECLARE_double(ndt_error_ndt_score);
