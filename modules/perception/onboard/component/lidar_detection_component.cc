/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/onboard/component/lidar_detection_component.h"

#include "cyber/time/clock.h"
#include "modules/perception/onboard/msg_serializer/msg_serializer.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/onboard/common_flags/common_flags.h"

using ::apollo::cyber::Clock;

namespace apollo {
namespace perception {
namespace onboard {

using apollo::cyber::common::GetAbsolutePath;

std::atomic<uint32_t> LidarDetectionComponent::seq_num_{0};

bool LidarDetectionComponent::Init() {
  LidarDetectionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    AERROR << "Get config failed";
    return false;
  }
  AINFO << "Lidar Component Configs: " << comp_config.DebugString();
  output_channel_name_ = comp_config.output_channel_name();
  sensor_name_ = comp_config.sensor_name();
  detector_name_ = comp_config.detector_name();
  lidar2vehicle_tf2_child_frame_id_ =
      comp_config.lidar2vehicle_tf2_child_frame_id();
  lidar_query_tf_offset_ =
      static_cast<float>(comp_config.lidar_query_tf_offset());
  enable_hdmap_ = comp_config.enable_hdmap();
  writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);
  debug_writer_ = node_->CreateWriter<PerceptionObstacles>("/apollo/perception/obstacles");

  const auto& lidar_detection_root_dir = comp_config.lidar_detection_conf_dir();
  const auto& lidar_detection_conf_file =
      comp_config.lidar_detection_conf_file();

  std::string work_root = "";
  std::string lidardetection_config_file =
      GetAbsolutePath(lidar_detection_root_dir, lidar_detection_conf_file);
  lidardetection_config_file =
      GetAbsolutePath(work_root, lidardetection_config_file);

  ACHECK(cyber::common::GetProtoFromFile(lidardetection_config_file,
                                         &lidar_detection_config_))
      << "failed to load lidar detection file " << lidardetection_config_file;

  if (!InitAlgorithmPlugin()) {
    AERROR << "Failed to init detection component algorithm plugin.";
    return false;
  }
  return true;
}

bool LidarDetectionComponent::Proc(
    const std::shared_ptr<drivers::PointCloud>& message) {
  AINFO << std::setprecision(16)
        << "Enter detection component, message timestamp: "
        << message->measurement_time()
        << " current timestamp: " << Clock::NowInSeconds();

  auto out_message = std::make_shared<LidarFrameMessage>();

  bool status = InternalProc(message, out_message);
  if (status) {
    writer_->Write(out_message);
    AINFO << "Send lidar detect output message.";

    // 额外输出protobuf到 /apollo/perception/obstacles 用于调试
    auto debug_msg = std::make_shared<PerceptionObstacles>();
    MsgSerializer::SerializeMsg(
        out_message->timestamp_,
        out_message->lidar_timestamp_,
        out_message->seq_num_,
        out_message->lidar_frame_->segmented_objects,
        out_message->error_code_,
        debug_msg.get());
    debug_writer_->Write(debug_msg);
    AINFO << "Send detection debug output to /apollo/perception/obstacles";
  }
  return status;
}

bool LidarDetectionComponent::InitAlgorithmPlugin() {
  ACHECK(common::SensorManager::Instance()->GetSensorInfo(sensor_name_,
                                                          &sensor_info_));

  lidar::BaseLidarObstacleDetection* detector =
      lidar::BaseLidarObstacleDetectionRegisterer::GetInstanceByName(
          detector_name_);
  CHECK_NOTNULL(detector);
  // detector_.reset(detector);
  // lidar::LidarObstacleDetectionInitOptions init_options;
  // init_options.sensor_name = sensor_name_;
  // init_options.enable_hdmap_input =
  //     FLAGS_obs_enable_hdmap_input && enable_hdmap_;
  // ACHECK(detector_->Init(init_options)) <<
  //                           "lidar obstacle detection init error";

  lidar_detection_pipeline_.reset(new lidar::LidarObstacleDetection);

  ACHECK(lidar_detection_pipeline_->Init(lidar_detection_config_))
      << "lidar obstacle detection init error";

  std::string lidar_tf_child_frame_id = lidar2vehicle_tf2_child_frame_id_;
  if (lidar_tf_child_frame_id.empty()) {
    lidar_tf_child_frame_id = sensor_info_.frame_id;
  } else if (lidar_tf_child_frame_id == FLAGS_obs_sensor2vehicle_tf2_frame_id &&
             sensor_info_.frame_id != FLAGS_obs_sensor2vehicle_tf2_frame_id &&
             !sensor_info_.frame_id.empty()) {
    AWARN << "Lidar detection config collapses sensor extrinsics to identity "
          << "by using vehicle frame " << lidar_tf_child_frame_id
          << "; fallback to sensor frame " << sensor_info_.frame_id;
    lidar_tf_child_frame_id = sensor_info_.frame_id;
  }

  lidar2world_trans_.Init(FLAGS_obs_sensor2vehicle_tf2_frame_id,
                          lidar_tf_child_frame_id,
                          FLAGS_obs_vehicle2world_tf2_frame_id,
                          FLAGS_obs_vehicle2world_tf2_child_frame_id);
  return true;
}

bool LidarDetectionComponent::ConvertCloud(
    const std::shared_ptr<const drivers::PointCloud>& from,
    std::shared_ptr<base::AttributePointCloud<base::PointF>> to) {
  to->set_timestamp(from->measurement_time());
  to->reserve(from->point_size());
  base::PointF point;
  for (int i = 0; i < from->point_size(); ++i) {
    const apollo::drivers::PointXYZIT& pt = from->point(i);
    point.x = pt.x();
    point.y = pt.y();
    point.z = pt.z();
    point.intensity = static_cast<float>(pt.intensity());
    to->push_back(point, static_cast<double>(pt.timestamp()) * 1e-9,
                  std::numeric_limits<float>::max(), i, 0);
  }
  return true;
}

bool LidarDetectionComponent::InternalProc(
    const std::shared_ptr<const drivers::PointCloud>& in_message,
    const std::shared_ptr<LidarFrameMessage>& out_message) {
  uint32_t seq_num = seq_num_.fetch_add(1);
  const double timestamp = in_message->measurement_time();
  const double cur_time = Clock::NowInSeconds();
  const double start_latency = (cur_time - timestamp) * 1e3;
  AINFO << std::setprecision(16) << "FRAME_STATISTICS:Lidar:Start:msg_time["
        << timestamp << "]:sensor[" << sensor_name_ << "]:cur_time[" << cur_time
        << "]:cur_latency[" << start_latency << "]";

  out_message->timestamp_ = timestamp;
  out_message->lidar_timestamp_ = in_message->header().lidar_timestamp();
  out_message->seq_num_ = seq_num;
  out_message->process_stage_ = ProcessStage::LIDAR_DETECTION;
  out_message->error_code_ = apollo::common::ErrorCode::OK;

  auto& frame = out_message->lidar_frame_;
  frame = lidar::LidarFramePool::Instance().Get();
  frame->cloud = base::PointFCloudPool::Instance().Get();
  frame->timestamp = timestamp;
  frame->sensor_info = sensor_info_;

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d pose_vehicle = Eigen::Affine3d::Identity();
  const double lidar_query_tf_timestamp =
      timestamp - lidar_query_tf_offset_ * 0.001;
  // const double lidar_query_tf_timestamp = cyber::Time::Now().ToSecond() - 0.1;
  if (!lidar2world_trans_.GetSensor2worldTrans(lidar_query_tf_timestamp, &pose,
                                               &pose_vehicle)) {
    out_message->error_code_ = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
    AERROR << std::fixed << std::setprecision(9)
           << "Failed to get pose at time: " << lidar_query_tf_timestamp;
    return false;
  }

  frame->lidar2world_pose = pose;
  frame->vehicle2world_pose = pose_vehicle;

  lidar::LidarObstacleDetectionOptions detect_opts;
  detect_opts.sensor_name = sensor_name_;
  lidar2world_trans_.GetExtrinsics(&detect_opts.sensor2vehicle_extrinsics);
  // lidar::LidarProcessResult ret =
  //     detector_->Process(detect_opts, in_message, frame.get());
  // if (ret.error_code != lidar::LidarErrorCode::Succeed) {
  //   out_message->error_code_ =
  //       apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
  //   AERROR << "Lidar detection process error, " << ret.log;
  //   return false;
  // }

  // Add point cloud to frame
  ConvertCloud(in_message, frame->cloud);
  frame->lidar2vehicle_extrinsics = detect_opts.sensor2vehicle_extrinsics;

  pipeline::DataFrame data_frame;
  data_frame.lidar_frame = frame.get();
  bool res = lidar_detection_pipeline_->Process(&data_frame);

  if (!res) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "Lidar detection process error!";
    return false;
  }

  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
