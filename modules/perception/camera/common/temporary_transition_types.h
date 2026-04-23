/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace apollo {
namespace perception {
namespace camera {

namespace yolo {

struct StBBoxStatistics {
  std::vector<float> mean;
  std::vector<float> std;
};

struct StDimensionStatistics {
  float mean_h = 0.0;
  float mean_w = 0.0;
  float mean_l = 0.0;
  float std_h = 1.0;
  float std_w = 1.0;
  float std_l = 1.0;
};

struct StModelParam {
  std::string model_name = "yolo-2d";
  std::string proto_file = "caffe.pt";
  std::string weight_file = "caffe.model";
  std::string stage1_pt = "caffe.pt";
  std::string stage1_md = "caffe.model";
  std::string stage2_pt = "caffe.pt";
  std::string stage2_md = "caffe.model";
  std::string anchors_file = "anchors.txt";
  std::string types_file = "types.txt";
  std::string feature_file = "feature.pt";

  float offset_ratio = 0.288889;
  float cropped_ratio = 0.711111;
  int32_t resized_width = 1440;
  int32_t aligned_pixel = 32;
  float confidence_threshold = 0.1;
  float light_vis_conf_threshold = 0.5;
  float light_swt_conf_threshold = 0.5;
  float roi_conf_threshold = 0.1;
  float box_conf_threshold = 0.5;
  float stage2_nms_threshold = 0.4;
  float min_2d_height = 0.0;
  float min_3d_height = 0.0;
  float min_3d_width = 0.0;
  float min_3d_length = 0.0;
  std::string calibratetable_root = "./";
  std::string model_type = "CaffeNet";
  int32_t ori_cycle = 1;
  bool per_cls_reg = false;
  std::vector<StDimensionStatistics> dimension_statistics;
  std::vector<StBBoxStatistics> bbox_statistics;
  std::string expand_file = "expand.txt";
  bool with_box3d = false;
  bool with_frbox = false;
  bool with_lights = false;
  bool with_ratios = false;
  int32_t num_areas = 0;
  float border_ratio = 0.01;
};

}  // namespace yolo

namespace smoke {

using yolo::StBBoxStatistics;
using yolo::StDimensionStatistics;
using yolo::StModelParam;

}  // namespace smoke

}  // namespace camera
}  // namespace perception
}  // namespace apollo
