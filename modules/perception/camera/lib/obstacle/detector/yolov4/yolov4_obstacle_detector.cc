/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera/lib/obstacle/detector/yolov4/yolov4_obstacle_detector.h"

#include <map>
#include <string>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/log.h"

// #include "modules/common/util/perf_util.h"
#include "modules/perception/base/common.h"
#include "modules/perception/camera/common/timer.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/inference/inference_factory.h"
#include "modules/perception/inference/utils/resize.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

void convert_model_params(const yolo::ModelParam &model_param,
                          yolo::StModelParam *st_model_param) {
  st_model_param->model_name = model_param.model_name();
  st_model_param->proto_file = model_param.proto_file();
  st_model_param->weight_file = model_param.weight_file();
  st_model_param->stage1_pt = model_param.stage1_pt();
  st_model_param->stage1_md = model_param.stage1_md();
  st_model_param->stage2_pt = model_param.stage2_pt();
  st_model_param->stage2_md = model_param.stage2_md();
  st_model_param->anchors_file = model_param.anchors_file();
  st_model_param->types_file = model_param.types_file();
  st_model_param->feature_file = model_param.feature_file();

  st_model_param->offset_ratio = model_param.offset_ratio();
  st_model_param->cropped_ratio = model_param.cropped_ratio();
  st_model_param->resized_width = model_param.resized_width();
  st_model_param->aligned_pixel = model_param.aligned_pixel();
  st_model_param->confidence_threshold = model_param.confidence_threshold();
  st_model_param->light_vis_conf_threshold =
      model_param.light_vis_conf_threshold();
  st_model_param->light_swt_conf_threshold =
      model_param.light_swt_conf_threshold();
  st_model_param->roi_conf_threshold = model_param.roi_conf_threshold();
  st_model_param->box_conf_threshold = model_param.box_conf_threshold();
  st_model_param->stage2_nms_threshold = model_param.stage2_nms_threshold();
  st_model_param->min_2d_height = model_param.min_2d_height();
  st_model_param->min_3d_height = model_param.min_3d_height();
  st_model_param->min_3d_width = model_param.min_3d_width();
  st_model_param->min_3d_length = model_param.min_3d_length();
  st_model_param->calibratetable_root = model_param.calibratetable_root();
  st_model_param->model_type = model_param.model_type();
  st_model_param->ori_cycle = model_param.ori_cycle();
  st_model_param->per_cls_reg = model_param.per_cls_reg();
  st_model_param->expand_file = model_param.expand_file();
  st_model_param->with_box3d = model_param.with_box3d();
  st_model_param->with_frbox = model_param.with_frbox();
  st_model_param->with_lights = model_param.with_lights();
  st_model_param->with_ratios = model_param.with_ratios();
  st_model_param->num_areas = model_param.num_areas();
  st_model_param->border_ratio = model_param.border_ratio();

  st_model_param->dimension_statistics.reserve(
      model_param.dimension_statistics_size());
  for (int i = 0; i < model_param.dimension_statistics_size(); ++i) {
    st_model_param->dimension_statistics[i].mean_h =
        model_param.dimension_statistics(i).mean_h();
    st_model_param->dimension_statistics[i].mean_w =
        model_param.dimension_statistics(i).mean_w();
    st_model_param->dimension_statistics[i].mean_l =
        model_param.dimension_statistics(i).mean_l();
    st_model_param->dimension_statistics[i].std_h =
        model_param.dimension_statistics(i).std_h();
    st_model_param->dimension_statistics[i].std_w =
        model_param.dimension_statistics(i).std_w();
    st_model_param->dimension_statistics[i].std_l =
        model_param.dimension_statistics(i).std_l();
  }
  st_model_param->bbox_statistics.reserve(model_param.bbox_statistics_size());
  for (int i = 0; i < model_param.bbox_statistics_size(); ++i) {
    st_model_param->bbox_statistics[i].mean.reserve(
        model_param.bbox_statistics(i).mean_size());
    for (int j = 0; j < model_param.bbox_statistics(i).mean_size(); ++j) {
      st_model_param->bbox_statistics[i].mean[j] =
          model_param.bbox_statistics(i).mean(j);
    }
    st_model_param->bbox_statistics[i].std.reserve(
        model_param.bbox_statistics(i).std_size());
    for (int j = 0; j < model_param.bbox_statistics(i).std_size(); ++j) {
      st_model_param->bbox_statistics[i].std[j] =
          model_param.bbox_statistics(i).std(j);
    }
  }
}

void Yolov4ObstacleDetector::LoadInputShape(
    const yolo::ModelParam &model_param) {
  float offset_ratio = model_param.offset_ratio();
  float cropped_ratio = model_param.cropped_ratio();
  int resized_width = model_param.resized_width();
  int aligned_pixel = model_param.aligned_pixel();
  // inference input shape
  int image_height = static_cast<int>(base_camera_model_->get_height());
  int image_width = static_cast<int>(base_camera_model_->get_width());

  offset_y_ =
      static_cast<int>(offset_ratio * static_cast<float>(image_height) + .5f);
  float roi_ratio = cropped_ratio * static_cast<float>(image_height) /
                    static_cast<float>(image_width);
  width_ = static_cast<int>(resized_width + aligned_pixel / 2) / aligned_pixel *
           aligned_pixel;
  height_ = static_cast<int>(static_cast<float>(width_) * roi_ratio +
                             static_cast<float>(aligned_pixel) / 2.0f) /
            aligned_pixel * aligned_pixel;

  AINFO << "image_height=" << image_height << ", "
        << "image_width=" << image_width << ", "
        << "roi_ratio=" << roi_ratio;
  AINFO << "offset_y=" << offset_y_ << ", height=" << height_
        << ", width=" << width_;
}

void Yolov4ObstacleDetector::LoadParam(const yolo::YoloParam &yolo_param) {
  const auto &model_param = yolo_param.model_param();
  confidence_threshold_ = model_param.confidence_threshold();
  light_vis_conf_threshold_ = model_param.light_vis_conf_threshold();
  light_swt_conf_threshold_ = model_param.light_swt_conf_threshold();
  min_dims_.min_2d_height = model_param.min_2d_height();
  min_dims_.min_3d_height = model_param.min_3d_height();
  min_dims_.min_3d_width = model_param.min_3d_width();
  min_dims_.min_3d_length = model_param.min_3d_length();
  ori_cycle_ = model_param.ori_cycle();

  border_ratio_ = model_param.border_ratio();

  // init NMS
  auto const &nms_param = yolo_param.nms_param();
  nms_.sigma = nms_param.sigma();
  nms_.type = nms_param.type();
  nms_.threshold = nms_param.threshold();
  nms_.inter_cls_nms_thresh = nms_param.inter_cls_nms_thresh();
  nms_.inter_cls_conf_thresh = nms_param.inter_cls_conf_thresh();
}

bool Yolov4ObstacleDetector::InitNet(const yolo::YoloParam &yolo_param,
                                     const std::string &model_root) {
  const auto &model_param = yolo_param.model_param();

  std::string proto_file =
      GetAbsolutePath(model_root, model_param.proto_file());
  std::string weight_file =
      GetAbsolutePath(model_root, model_param.weight_file());
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
  // init Net
  auto const &net_param = yolo_param.net_param();
  input_names.push_back(net_param.input_blob());
  output_names.push_back(net_param.det1_loc_blob());
  output_names.push_back(net_param.det1_obj_blob());
  output_names.push_back(net_param.det1_cls_blob());
  output_names.push_back(net_param.det1_ori_conf_blob());
  output_names.push_back(net_param.det1_ori_blob());
  output_names.push_back(net_param.det1_dim_blob());
  output_names.push_back(net_param.det2_loc_blob());
  output_names.push_back(net_param.det2_obj_blob());
  output_names.push_back(net_param.det2_cls_blob());
  output_names.push_back(net_param.det2_ori_conf_blob());
  output_names.push_back(net_param.det2_ori_blob());
  output_names.push_back(net_param.det2_dim_blob());
  output_names.push_back(net_param.det3_loc_blob());
  output_names.push_back(net_param.det3_obj_blob());
  output_names.push_back(net_param.det3_cls_blob());
  output_names.push_back(net_param.det3_ori_conf_blob());
  output_names.push_back(net_param.det3_ori_blob());
  output_names.push_back(net_param.det3_dim_blob());
  output_names.push_back(net_param.lof_blob());
  output_names.push_back(net_param.lor_blob());
  output_names.push_back(net_param.brvis_blob());
  output_names.push_back(net_param.brswt_blob());
  output_names.push_back(net_param.ltvis_blob());
  output_names.push_back(net_param.ltswt_blob());
  output_names.push_back(net_param.rtvis_blob());
  output_names.push_back(net_param.rtswt_blob());
  output_names.push_back(net_param.feat_blob());
  output_names.push_back(net_param.area_id_blob());
  output_names.push_back(net_param.visible_ratio_blob());
  output_names.push_back(net_param.cut_off_ratio_blob());

  // init Net
  const auto &model_type = model_param.model_type();
  AINFO << "model_type=" << model_type;
  inference_.reset(inference::CreateInferenceByName(model_type, proto_file,
                                                    weight_file, output_names,
                                                    input_names, model_root));
  if (nullptr == inference_.get()) {
    AERROR << "Failed to init CNNAdapter";
    return false;
  }
  inference_->set_gpu_id(gpu_id_);
  std::vector<int> shape = {1, height_, width_, 3};
  std::map<std::string, std::vector<int>> shape_map{
      {net_param.input_blob(), shape}};

  if (!inference_->Init(shape_map)) {
    return false;
  }
  inference_->Infer();
  return true;
}

void Yolov4ObstacleDetector::InitYoloBlob(const yolo::NetworkParam &net_param) {
  auto obj_blob_scale1 = inference_->get_blob(net_param.det1_obj_blob());
  auto obj_blob_scale2 = inference_->get_blob(net_param.det2_obj_blob());
  auto obj_blob_scale3 = inference_->get_blob(net_param.det3_obj_blob());
  int output_height_scale1 = obj_blob_scale1->shape(1);
  int output_width_scale1 = obj_blob_scale1->shape(2);
  int obj_size = output_height_scale1 * output_width_scale1 *
                 static_cast<int>(anchors_.size()) / anchorSizeFactor;
  if (obj_blob_scale2) {
    int output_height_scale2 = obj_blob_scale2->shape(1);
    int output_width_scale2 = obj_blob_scale2->shape(2);
    int output_height_scale3 = obj_blob_scale3->shape(1);
    int output_width_scale3 = obj_blob_scale3->shape(2);
    obj_size = (output_height_scale1 * output_width_scale1 +
                output_height_scale2 * output_width_scale2 +
                output_height_scale3 * output_width_scale3) *
               static_cast<int>(anchors_.size()) / anchorSizeFactor / numScales;
  }

  yolo_blobs_.res_box_blob.reset(
      new base::Blob<float>(1, 1, obj_size, kBoxBlockSize));
  yolo_blobs_.res_cls_blob.reset(new base::Blob<float>(
      1, 1, static_cast<int>(types_.size() + 1), obj_size));
  yolo_blobs_.res_cls_blob->cpu_data();
  overlapped_.reset(
      new base::Blob<bool>(std::vector<int>{obj_k_, obj_k_}, true));
  overlapped_->cpu_data();
  overlapped_->gpu_data();
  idx_sm_.reset(new base::Blob<int>(std::vector<int>{obj_k_}, true));
  yolo_blobs_.anchor_blob.reset(
      new base::Blob<float>(1, 1, static_cast<int>(anchors_.size() / 2), 2));
  yolo_blobs_.expand_blob.reset(
      new base::Blob<float>(1, 1, 1, static_cast<int>(expands_.size())));
  auto expand_cpu_data = yolo_blobs_.expand_blob->mutable_cpu_data();
  memcpy(expand_cpu_data, expands_.data(), expands_.size() * sizeof(float));
  auto anchor_cpu_data = yolo_blobs_.anchor_blob->mutable_cpu_data();
  memcpy(anchor_cpu_data, anchors_.data(), anchors_.size() * sizeof(float));
  yolo_blobs_.anchor_blob->gpu_data();

  image_.reset(new base::Image8U(height_, width_, base::Color::RGB));

  yolo_blobs_.det1_loc_blob =
      inference_->get_blob(yolo_param_.net_param().det1_loc_blob());
  yolo_blobs_.det1_obj_blob =
      inference_->get_blob(yolo_param_.net_param().det1_obj_blob());
  yolo_blobs_.det1_cls_blob =
      inference_->get_blob(yolo_param_.net_param().det1_cls_blob());
  yolo_blobs_.det1_ori_conf_blob =
      inference_->get_blob(yolo_param_.net_param().det1_ori_conf_blob());
  yolo_blobs_.det1_ori_blob =
      inference_->get_blob(yolo_param_.net_param().det1_ori_blob());
  yolo_blobs_.det1_dim_blob =
      inference_->get_blob(yolo_param_.net_param().det1_dim_blob());
  yolo_blobs_.det2_loc_blob =
      inference_->get_blob(yolo_param_.net_param().det2_loc_blob());
  yolo_blobs_.det2_obj_blob =
      inference_->get_blob(yolo_param_.net_param().det2_obj_blob());
  yolo_blobs_.det2_cls_blob =
      inference_->get_blob(yolo_param_.net_param().det2_cls_blob());
  yolo_blobs_.det2_ori_conf_blob =
      inference_->get_blob(yolo_param_.net_param().det2_ori_conf_blob());
  yolo_blobs_.det2_ori_blob =
      inference_->get_blob(yolo_param_.net_param().det2_ori_blob());
  yolo_blobs_.det2_dim_blob =
      inference_->get_blob(yolo_param_.net_param().det2_dim_blob());
  yolo_blobs_.det3_loc_blob =
      inference_->get_blob(yolo_param_.net_param().det3_loc_blob());
  yolo_blobs_.det3_obj_blob =
      inference_->get_blob(yolo_param_.net_param().det3_obj_blob());
  yolo_blobs_.det3_cls_blob =
      inference_->get_blob(yolo_param_.net_param().det3_cls_blob());
  yolo_blobs_.det3_ori_conf_blob =
      inference_->get_blob(yolo_param_.net_param().det3_ori_conf_blob());
  yolo_blobs_.det3_ori_blob =
      inference_->get_blob(yolo_param_.net_param().det3_ori_blob());
  yolo_blobs_.det3_dim_blob =
      inference_->get_blob(yolo_param_.net_param().det3_dim_blob());

  yolo_blobs_.lof_blob =
      inference_->get_blob(yolo_param_.net_param().lof_blob());
  yolo_blobs_.lor_blob =
      inference_->get_blob(yolo_param_.net_param().lor_blob());

  yolo_blobs_.brvis_blob =
      inference_->get_blob(yolo_param_.net_param().brvis_blob());
  yolo_blobs_.brswt_blob =
      inference_->get_blob(yolo_param_.net_param().brswt_blob());
  yolo_blobs_.ltvis_blob =
      inference_->get_blob(yolo_param_.net_param().ltvis_blob());
  yolo_blobs_.ltswt_blob =
      inference_->get_blob(yolo_param_.net_param().ltswt_blob());
  yolo_blobs_.rtvis_blob =
      inference_->get_blob(yolo_param_.net_param().rtvis_blob());
  yolo_blobs_.rtswt_blob =
      inference_->get_blob(yolo_param_.net_param().rtswt_blob());

  yolo_blobs_.area_id_blob =
      inference_->get_blob(yolo_param_.net_param().area_id_blob());
  yolo_blobs_.visible_ratio_blob =
      inference_->get_blob(yolo_param_.net_param().visible_ratio_blob());
  yolo_blobs_.cut_off_ratio_blob =
      inference_->get_blob(yolo_param_.net_param().cut_off_ratio_blob());
}

bool Yolov4ObstacleDetector::Init(const ObstacleDetectorInitOptions &options) {
  gpu_id_ = options.gpu_id;
  BASE_CUDA_CHECK(cudaSetDevice(gpu_id_));
  BASE_CUDA_CHECK(cudaStreamCreate(&stream_));

  base_camera_model_ = options.base_camera_model;
  ACHECK(base_camera_model_ != nullptr) << "base_camera_model is nullptr!";
  std::string config_path =
      GetAbsolutePath(options.root_dir, options.conf_file);
  if (!cyber::common::GetProtoFromFile(config_path, &yolo_param_)) {
    AERROR << "read proto_config fail";
    return false;
  }
  const auto &model_param = yolo_param_.model_param();
  std::string model_root =
      GetAbsolutePath(options.root_dir, model_param.model_name());
  std::string anchors_file =
      GetAbsolutePath(model_root, model_param.anchors_file());
  std::string types_file =
      GetAbsolutePath(model_root, model_param.types_file());
  std::string expand_file =
      GetAbsolutePath(model_root, model_param.expand_file());
  LoadInputShape(model_param);
  LoadParam(yolo_param_);
  min_dims_.min_2d_height /= static_cast<float>(height_);

  if (!LoadAnchors(anchors_file, &anchors_)) {
    return false;
  }
  if (!LoadTypes(types_file, &types_)) {
    return false;
  }
  if (!LoadExpand(expand_file, &expands_)) {
    return false;
  }
  ACHECK(expands_.size() == types_.size());
  if (!InitNet(yolo_param_, model_root)) {
    return false;
  }
  InitYoloBlob(yolo_param_.net_param());
  if (!InitFeatureExtractor(model_root)) {
    return false;
  }
  return true;
}

bool Yolov4ObstacleDetector::Init(const StageConfig &stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }

  yolo_obstacle_detector_config_ = stage_config.yolo_obstacle_detector_config();
  gpu_id_ = yolo_obstacle_detector_config_.gpu_id();
  BASE_CUDA_CHECK(cudaSetDevice(gpu_id_));
  BASE_CUDA_CHECK(cudaStreamCreate(&stream_));

  base_camera_model_ =
      common::SensorManager::Instance()->GetUndistortCameraModel(
          yolo_obstacle_detector_config_.camera_name());
  ACHECK(base_camera_model_ != nullptr) << "base_camera_model is nullptr!";

  yolo_param_ = yolo_obstacle_detector_config_.yolo_param();
  const auto &model_param = yolo_param_.model_param();
  // todo(zero): options.root_dir
  std::string root_dir = yolo_obstacle_detector_config_.root_dir();
  std::string model_root = GetAbsolutePath(root_dir, model_param.model_name());
  std::string anchors_file =
      GetAbsolutePath(model_root, model_param.anchors_file());
  std::string types_file =
      GetAbsolutePath(model_root, model_param.types_file());
  std::string expand_file =
      GetAbsolutePath(model_root, model_param.expand_file());
  LoadInputShape(model_param);
  LoadParam(yolo_param_);
  min_dims_.min_2d_height /= static_cast<float>(height_);

  if (!LoadAnchors(anchors_file, &anchors_)) {
    return false;
  }
  if (!LoadTypes(types_file, &types_)) {
    return false;
  }
  if (!LoadExpand(expand_file, &expands_)) {
    return false;
  }
  ACHECK(expands_.size() == types_.size());
  if (!InitNet(yolo_param_, model_root)) {
    return false;
  }
  InitYoloBlob(yolo_param_.net_param());
  if (!InitFeatureExtractor(model_root)) {
    return false;
  }
  return true;
}

bool Yolov4ObstacleDetector::Process(DataFrame *data_frame) { return true; }

bool Yolov4ObstacleDetector::InitFeatureExtractor(const std::string &root_dir) {
  FeatureExtractorInitOptions feat_options;
  feat_options.conf_file = yolo_param_.model_param().feature_file();
  feat_options.root_dir = root_dir;
  feat_options.gpu_id = gpu_id_;
  auto feat_blob_name = yolo_param_.net_param().feat_blob();
  feat_options.feat_blob = inference_->get_blob(feat_blob_name);
  feat_options.input_height = height_;
  feat_options.input_width = width_;
  feature_extractor_.reset(BaseFeatureExtractorRegisterer::GetInstanceByName(
      "TrackingFeatureExtractor"));
  if (!feature_extractor_->Init(feat_options)) {
    return false;
  }
  return true;
}

bool Yolov4ObstacleDetector::Detect(const ObstacleDetectorOptions &options,
                                    CameraFrame *frame) {
  if (frame == nullptr) {
    return false;
  }

  Timer timer;
  if (cudaSetDevice(gpu_id_) != cudaSuccess) {
    AERROR << "Failed to set device to " << gpu_id_;
    return false;
  }

  auto input_blob = inference_->get_blob(yolo_param_.net_param().input_blob());
  AINFO << "Start: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  image_options.crop_roi = base::RectI(
      0, offset_y_, static_cast<int>(base_camera_model_->get_width()),
      static_cast<int>(base_camera_model_->get_height()) - offset_y_);
  image_options.do_crop = true;
  frame->data_provider->GetImage(image_options, image_.get());
  AINFO << "GetImageBlob: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  inference::ResizeGPU(*image_, input_blob, frame->data_provider->src_width(),
                       0);
  AINFO << "Resize: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  /////////////////////////// detection part ///////////////////////////
  inference_->Infer();
  AINFO << "Network Forward: " << static_cast<double>(timer.Toc()) * 0.001
        << "ms";
  // TODO(All): refactor it
  // convert proto message to pure struct to avoid adding protobuf as
  // cuda_library's dependence, which may case compile issue of nvcc
  yolo::StModelParam st_model_param;
  convert_model_params(yolo_param_.model_param(), &st_model_param);
  get_objects_gpu(yolo_blobs_, stream_, types_, nms_, st_model_param,
                  light_vis_conf_threshold_, light_swt_conf_threshold_,
                  overlapped_.get(), idx_sm_.get(), &(frame->detected_objects));

  AINFO << "GetObj: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  filter_bbox(min_dims_, &(frame->detected_objects));
  FeatureExtractorOptions feat_options;
  feat_options.normalized = true;
  AINFO << "Post1: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  feature_extractor_->Extract(feat_options, frame);
  AINFO << "Extract: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  recover_bbox(frame->data_provider->src_width(),
               frame->data_provider->src_height() - offset_y_, offset_y_,
               &frame->detected_objects);

  // post processing
  int left_boundary =
      static_cast<int>(border_ratio_ * static_cast<float>(image_->cols()));
  int right_boundary = static_cast<int>((1.0f - border_ratio_) *
                                        static_cast<float>(image_->cols()));
  for (auto &obj : frame->detected_objects) {
    // recover alpha
    obj->camera_supplement.alpha /= ori_cycle_;
    // get area_id from visible_ratios
    if (yolo_param_.model_param().num_areas() == 0) {
      obj->camera_supplement.area_id =
          get_area_id(obj->camera_supplement.visible_ratios);
    }
    // clear cut off ratios
    auto &box = obj->camera_supplement.box;
    if (box.xmin >= left_boundary) {
      obj->camera_supplement.cut_off_ratios[2] = 0;
    }
    if (box.xmax <= right_boundary) {
      obj->camera_supplement.cut_off_ratios[3] = 0;
    }
  }
  AINFO << "Post2: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  return true;
}

REGISTER_OBSTACLE_DETECTOR(Yolov4ObstacleDetector);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
