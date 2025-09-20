/******************************************************************************
 * Copyright 2025 The WheelOS Team. All Rights Reserved.
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

#include <NvInferVersion.h>

#include <cstdint>
#include <string>
#include <vector>

#ifdef NV_TENSORRT_MAJOR
#if NV_TENSORRT_MAJOR == 8
#include "modules/perception/inference/tensorrt/rt_legacy.h"
#endif
#endif

#include "NvCaffeParser.h"
#include "NvInfer.h"
#if GPU_PLATFORM == NVIDIA
#include <cudnn.h>
#elif GPU_PLATFORM == AMD
#include <miopen/miopen.h>
#define CUDNN_DATA_FLOAT miopenFloat
#define CUDNN_SOFTMAX_ACCURATE MIOPEN_SOFTMAX_ACCURATE
#define CUDNN_SOFTMAX_MODE_CHANNEL MIOPEN_SOFTMAX_MODE_CHANNEL
#define cudnnCreate miopenCreate
#define cudnnCreateTensorDescriptor miopenCreateTensorDescriptor
#define cudnnDestroy miopenDestroy
#define cudnnDestroyTensorDescriptor miopenDestroyTensorDescriptor
#define cudnnHandle_t miopenHandle_t
#define cudnnSetStream miopenSetStream
#define cudnnSetTensor4dDescriptorEx miopenSet4dTensorDescriptorEx
#define cudnnTensorDescriptor_t miopenTensorDescriptor_t
#endif

#include "cyber/common/log.h"
#include "modules/perception/base/common.h"

namespace apollo {
namespace perception {
namespace inference {

struct StFillerParameter {
  // The filler type.
  std::string type = "constant";
  float value = 0;  // the value in constant filler
  float min = 0;    // the min value in uniform filler
  float max = 1;    // the max value in uniform filler
  float mean = 0;   // the mean value in Gaussian filler
  float std = 1;    // the std value in Gaussian filler
  // The expected number of non-zero output weights for a given input in
  // Gaussian filler -- the default -1 means don't perform sparsification.
  int32_t sparse = -1;
  // Normalize the filler variance by fan_in, fan_out, or their average.
  // Applies to 'xavier' and 'msra' fillers.
  int32_t variance_norm = 0;
};

struct StBBoxRegParameter {
  // Normalization value for box's mean.
  std::vector<float> bbox_mean;
  // Normalization value for box's std.
  std::vector<float> bbox_std;
};

struct StDFMBPSROIAlignParameter {
  // Parameters for DeForMaBle Position Sensitive ROI Align.
  float heat_map_a;
  int32_t output_dim;
  int32_t group_height;
  int32_t group_width;
  int32_t pooled_height;
  int32_t pooled_width;
  float pad_ratio;
  int32_t sample_per_part;

  float trans_std = 0.0;
  int32_t part_height = 0;
  int32_t part_width = 0;
  float heat_map_b = 0.0;
};

struct StGenAnchorParameter {
  // Widths and heights for generating anchors.
  std::vector<float> anchor_width;
  std::vector<float> anchor_height;
};

struct StNMSSSDParameter {
  bool need_nms = true;
  std::vector<float> overlap_ratio;
  std::vector<uint32_t> top_n;
  bool add_score = false;
  std::vector<int32_t> max_candidate_n;
  std::vector<bool> use_soft_nms;
  bool nms_among_classes = false;
  std::vector<bool> voting;
  std::vector<float> vote_iou;
  // Thresholds for learning nms.
  float force_identity_iou_thr = 1.0;
  float force_imparity_iou_thr = 0.0;
  int32_t nms_gpu_max_n_per_time = -1;
};

struct StDetectionOutputSSDParameter {
  // Feature map stride for generating anchors.
  float heat_map_a;
  // Minimum height size for filtering boxes.
  float min_size_h = 2.0;
  // Minimum weight size for filtering boxes.
  float min_size_w = 2.0;
  // Based on which size to filtering boxes.
  int32_t min_size_mode = 0;
  // Threshold for objectness.
  float threshold_objectness = 0.0;
  StGenAnchorParameter gen_anchor_param;
  // Whether to refine boxes that are out of feature map.
  bool refine_out_of_map_bbox = false;
  StNMSSSDParameter nms_param;
  // Number of object classes.
  uint32_t num_class = 1;
  // Whether to output scores of rpn proposals.
  bool rpn_proposal_output_score = false;
  // Whether to regress agnostic proposals.
  bool regress_agnostic;
  // Threshold of each class.
  std::vector<float> threshold;
};

struct StSliceParameter {
  // The axis along which to slice -- may be negative to index from the end
  // (e.g., -1 for the last axis).
  // By default, SliceLayer concatenates blobs along the "channels" axis (1).
  int32_t axis = 1;
  std::vector<uint32_t> slice_point;

  // DEPRECATED: alias for "axis" -- does not support negative indexing.
  uint32_t slice_dim = 1;
};

struct StArgMaxParameter {
  // If true produce pairs (argmax, maxval)
  bool out_max_val = false;
  uint32_t top_k = 1;
  // The axis along which to maximise -- may be negative to index from the
  // end (e.g., -1 for the last axis).
  // By default ArgMaxLayer maximizes over the flattened trailing dimensions
  // for each index of the first / num dimension.
  int32_t axis;
};

struct StSoftmaxParameter {
  // enum Engine {
  //   DEFAULT = 0;
  //   CAFFE = 1;
  //   CUDNN = 2;
  // }
  int engine = 0;

  // The axis along which to perform the softmax -- may be negative to index
  // from the end (e.g., -1 for the last axis).
  // Any other axes will be evaluated as independent softmaxes.
  int32_t axis = 1;
};

struct StReLUParameter {
  // Allow non-zero slope for negative inputs to speed up optimization
  // Described in:
  // Maas, A. L., Hannun, A. Y., & Ng, A. Y. (2013). Rectifier nonlinearities
  // improve neural network acoustic models. In ICML Workshop on Deep Learning
  // for Audio, Speech, and Language Processing.
  float negative_slope = 0.0;
  // enum Engine {
  //   DEFAULT = 0;
  //   CAFFE = 1;
  //   CUDNN = 2;
  // }
  int32_t engine = 0;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
