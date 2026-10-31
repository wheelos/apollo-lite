// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/bevfusion/component/bevfusion_component.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace bevfusion {

namespace {

constexpr std::size_t kCalibrationValueCount = kCameraCount * 16;
constexpr double kSecondsToNanoseconds = 1.0e9;
constexpr double kNanosecondsToSeconds = 1.0e-9;

std::uint64_t SteadyNowNanoseconds() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

bool ToTimestampNanoseconds(double seconds, std::uint64_t* timestamp_ns) {
  if (timestamp_ns == nullptr || !std::isfinite(seconds) || seconds <= 0.0 ||
      seconds > static_cast<double>(std::numeric_limits<std::uint64_t>::max()) /
                    kSecondsToNanoseconds) {
    return false;
  }
  *timestamp_ns = static_cast<std::uint64_t>(seconds * kSecondsToNanoseconds);
  return *timestamp_ns != 0;
}

std::string ImageFrameId(const apollo::drivers::Image& image) {
  if (!image.frame_id().empty()) {
    return image.frame_id();
  }
  if (image.has_header() && image.header().has_frame_id()) {
    return image.header().frame_id();
  }
  return {};
}

std::string PointCloudFrameId(const apollo::drivers::PointCloud& cloud) {
  if (!cloud.frame_id().empty()) {
    return cloud.frame_id();
  }
  if (cloud.has_header() && cloud.header().has_frame_id()) {
    return cloud.header().frame_id();
  }
  return {};
}

bool CopyMatrix(const google::protobuf::RepeatedField<float>& values,
                std::array<float, kCalibrationValueCount>* matrix,
                const char* name, std::string* error) {
  if (values.size() != static_cast<int>(matrix->size())) {
    *error = std::string(name) + " must contain exactly 96 floats";
    return false;
  }
  std::copy(values.begin(), values.end(), matrix->begin());
  return true;
}

bool IsConfigured(const BevFusionComponentConfig& config, std::string* error) {
  if (!config.has_model() ||
      config.camera_channel_size() != static_cast<int>(kCameraCount) ||
      config.lidar_channel().empty() || config.output_channel().empty() ||
      config.max_timestamp_skew_ns() == 0 || config.max_sync_wait_ns() == 0 ||
      config.input_queue_capacity() == 0 ||
      config.camera_frame_id_size() != static_cast<int>(kCameraCount) ||
      config.lidar_frame_id().empty() ||
      config.calibration_revision().empty() ||
      !config.has_lidar_feature_contract() ||
      config.lidar_feature_contract() !=
          XYZ_INTENSITY_POINT_TIME_OFFSET_SECONDS ||
      !config.has_lidar_sweep_policy() ||
      config.lidar_sweep_policy() != SINGLE_SWEEP_EXPERIMENTAL ||
      !config.has_max_point_time_offset_seconds() ||
      !std::isfinite(config.max_point_time_offset_seconds()) ||
      config.max_point_time_offset_seconds() <= 0.0F ||
      config.point_feature_scale_size() !=
          static_cast<int>(kPointFeatureCount)) {
    *error =
        "BEVFusion component requires six camera channels and frame IDs, "
        "LiDAR/output channels and frame ID, a calibration revision, "
        "explicit XYZ/intensity/per-point-time feature and single-sweep "
        "experimental policies, five feature scales, a point-time "
        "offset limit, and nonzero sync limits";
    return false;
  }
  for (std::size_t camera = 0; camera < kCameraCount; ++camera) {
    if (config.camera_channel(static_cast<int>(camera)).empty() ||
        config.camera_frame_id(static_cast<int>(camera)).empty()) {
      *error = std::string("camera channel/frame ID is unset for ") +
               kCameraOrder[camera];
      return false;
    }
  }
  return true;
}

}  // namespace

bool BevFusionComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Failed to load BEVFusion component config";
    return false;
  }
  std::string error;
  if (!IsConfigured(config_, &error)) {
    AERROR << "Invalid BEVFusion component config: " << error;
    return false;
  }
  if (!CopyMatrix(config_.camera_to_lidar(), &camera_to_lidar_,
                  "camera_to_lidar", &error) ||
      !CopyMatrix(config_.camera_intrinsics(), &camera_intrinsics_,
                  "camera_intrinsics", &error) ||
      !CopyMatrix(config_.lidar_to_image(), &lidar_to_image_, "lidar_to_image",
                  &error) ||
      !CopyMatrix(config_.image_augmentation(), &image_augmentation_,
                  "image_augmentation", &error)) {
    AERROR << "Invalid BEVFusion calibration snapshot: " << error;
    return false;
  }
  for (std::size_t feature = 0; feature < kPointFeatureCount; ++feature) {
    const float scale = config_.point_feature_scale(static_cast<int>(feature));
    if (!std::isfinite(scale) || scale == 0.0F) {
      AERROR << "BEVFusion point feature scales must be finite and nonzero";
      return false;
    }
    point_feature_scale_[feature] = scale;
  }
  if (!synchronizer_.Init(config_.input_queue_capacity(),
                          config_.max_timestamp_skew_ns(),
                          config_.max_sync_wait_ns(), &error)) {
    AERROR << "Failed to initialize BEVFusion synchronizer: " << error;
    return false;
  }
  if (!runner_.Init(config_.model(), config_.max_timestamp_skew_ns(), &error)) {
    AERROR << "Failed to initialize BEVFusion runner: " << error;
    return false;
  }

  camera_readers_.reserve(kCameraCount - 1);
  for (std::size_t camera = 1; camera < kCameraCount; ++camera) {
    auto reader = node_->CreateReader<apollo::drivers::Image>(
        config_.camera_channel(static_cast<int>(camera)),
        [this, camera](const std::shared_ptr<apollo::drivers::Image>& image) {
          OnAdditionalCamera(camera, image);
        });
    if (reader == nullptr) {
      AERROR << "Failed to create BEVFusion camera reader for "
             << kCameraOrder[camera];
      return false;
    }
    camera_readers_.push_back(std::move(reader));
  }
  lidar_reader_ = node_->CreateReader<apollo::drivers::PointCloud>(
      config_.lidar_channel(),
      [this](const std::shared_ptr<apollo::drivers::PointCloud>& cloud) {
        OnLidar(cloud);
      });
  if (lidar_reader_ == nullptr) {
    AERROR << "Failed to create BEVFusion LiDAR reader on "
           << config_.lidar_channel();
    return false;
  }
  writer_ = node_->CreateWriter<BevFusionFrameResult>(config_.output_channel());
  if (writer_ == nullptr) {
    AERROR << "Failed to create BEVFusion output writer on channel "
           << config_.output_channel();
    return false;
  }
  AINFO << "Initialized BEVFusion component with six configured camera "
           "channels, LiDAR channel, calibration revision "
        << config_.calibration_revision()
        << ", and bounded timestamp synchronization";
  return true;
}

bool BevFusionComponent::Proc(
    const std::shared_ptr<apollo::drivers::Image>& image) {
  if (image == nullptr) {
    AERROR << "BEVFusion component received null primary camera image";
    return false;
  }
  std::lock_guard<std::mutex> lock(callback_mutex_);
  std::string error;
  if (!EnqueueCamera(0, *image, &error)) {
    AERROR << "BEVFusion rejected " << kCameraOrder[0] << " image: " << error;
    return false;
  }
  if (!Drain(&error)) {
    AERROR << "BEVFusion synchronization or inference failed: " << error;
    return false;
  }
  return true;
}

void BevFusionComponent::OnAdditionalCamera(
    std::size_t camera_index,
    const std::shared_ptr<apollo::drivers::Image>& message) {
  if (message == nullptr) {
    AERROR << "BEVFusion received null image for "
           << kCameraOrder[camera_index];
    return;
  }
  std::lock_guard<std::mutex> lock(callback_mutex_);
  std::string error;
  if (!EnqueueCamera(camera_index, *message, &error)) {
    AERROR << "BEVFusion rejected " << kCameraOrder[camera_index]
           << " image: " << error;
    return;
  }
  if (!Drain(&error)) {
    AERROR << "BEVFusion synchronization or inference failed: " << error;
  }
}

void BevFusionComponent::OnLidar(
    const std::shared_ptr<apollo::drivers::PointCloud>& message) {
  if (message == nullptr) {
    AERROR << "BEVFusion received a null PointCloud";
    return;
  }
  std::lock_guard<std::mutex> lock(callback_mutex_);
  std::string error;
  if (!EnqueueLidar(*message, &error)) {
    AERROR << "BEVFusion rejected LiDAR frame: " << error;
    return;
  }
  if (!Drain(&error)) {
    AERROR << "BEVFusion synchronization or inference failed: " << error;
  }
}

bool BevFusionComponent::EnqueueCamera(std::size_t camera_index,
                                       const apollo::drivers::Image& message,
                                       std::string* error) {
  if (camera_index >= kCameraCount) {
    *error = "camera index is outside the configured six-camera set";
    return false;
  }
  const std::string expected_frame =
      config_.camera_frame_id(static_cast<int>(camera_index));
  if (ImageFrameId(message) != expected_frame) {
    *error = "image frame_id does not match configured camera frame_id";
    return false;
  }
  if (message.encoding() != "rgb8" && message.encoding() != "RGB8_HWC") {
    *error = "only packed RGB8 HWC image encoding is supported";
    return false;
  }

  std::uint64_t timestamp_ns = 0;
  if (message.has_header() && message.header().has_camera_timestamp() &&
      message.header().camera_timestamp() != 0) {
    timestamp_ns = message.header().camera_timestamp();
  } else if (!ToTimestampNanoseconds(message.measurement_time(),
                                     &timestamp_ns)) {
    *error = "image has no valid camera timestamp";
    return false;
  }

  auto storage = std::make_shared<std::vector<std::uint8_t>>(
      message.data().begin(), message.data().end());
  auto packet = std::make_shared<CameraPacket>();
  packet->timestamp_ns = timestamp_ns;
  packet->arrival_monotonic_ns = SteadyNowNanoseconds();
  packet->storage = storage;
  packet->image.camera_name = kCameraOrder[camera_index];
  packet->image.timestamp_ns = timestamp_ns;
  packet->image.width = message.width();
  packet->image.height = message.height();
  packet->image.stride_bytes = message.step();
  packet->image.pixel_format = PixelFormat::kRgb8Hwc;
  packet->image.data = storage->data();
  packet->image.byte_count = storage->size();
  return synchronizer_.PushCamera(camera_index, std::move(packet), error);
}

bool BevFusionComponent::EnqueueLidar(
    const apollo::drivers::PointCloud& message, std::string* error) {
  if (PointCloudFrameId(message) != config_.lidar_frame_id()) {
    *error = "PointCloud frame_id does not match configured LiDAR frame_id";
    return false;
  }
  const std::size_t point_count =
      static_cast<std::size_t>(message.point_size());
  if (point_count == 0 ||
      point_count > config_.model().max_input_point_count() ||
      point_count > kMaxPointCount) {
    *error =
        "PointCloud is empty or exceeds configured maximum; clipping is "
        "not permitted";
    return false;
  }
  std::uint64_t timestamp_ns = 0;
  if (!ToTimestampNanoseconds(message.measurement_time(), &timestamp_ns)) {
    *error = "PointCloud has no valid measurement_time timestamp";
    return false;
  }
  auto packed_points =
      std::make_shared<std::vector<float>>(point_count * kPointFeatureCount);
  for (std::size_t index = 0; index < point_count; ++index) {
    const auto& point = message.point(static_cast<int>(index));
    const float xyz_intensity[kPointFeatureCount - 1] = {
        point.x(), point.y(), point.z(), static_cast<float>(point.intensity())};
    for (std::size_t feature = 0; feature < kPointFeatureCount - 1; ++feature) {
      const float value =
          xyz_intensity[feature] * point_feature_scale_[feature];
      if (!std::isfinite(value)) {
        *error = "PointCloud contains a non-finite scaled feature";
        return false;
      }
      (*packed_points)[index * kPointFeatureCount + feature] = value;
    }
    double time_offset = 0.0;
    if (point.timestamp() == 0) {
      *error =
          "configured per-point time-offset feature requires a nonzero "
          "nanosecond timestamp on every point";
      return false;
    }
    const auto point_timestamp = point.timestamp();
    const double signed_delta =
        point_timestamp >= timestamp_ns
            ? static_cast<double>(point_timestamp - timestamp_ns)
            : -static_cast<double>(timestamp_ns - point_timestamp);
    time_offset = signed_delta * kNanosecondsToSeconds;
    if (!std::isfinite(time_offset) ||
        std::abs(time_offset) > config_.max_point_time_offset_seconds()) {
      *error = "PointCloud point-time offset exceeds configured seconds limit";
      return false;
    }
    const float scaled_time_offset =
        static_cast<float>(time_offset) * point_feature_scale_[4];
    if (!std::isfinite(scaled_time_offset)) {
      *error = "PointCloud contains a non-finite scaled point time offset";
      return false;
    }
    (*packed_points)[index * kPointFeatureCount + 4] = scaled_time_offset;
  }

  auto packet = std::make_shared<LidarPacket>();
  packet->timestamp_ns = timestamp_ns;
  packet->arrival_monotonic_ns = SteadyNowNanoseconds();
  packet->point_count = point_count;
  packet->packed_points = std::move(packed_points);
  return synchronizer_.PushLidar(std::move(packet), error);
}

bool BevFusionComponent::Drain(std::string* error) {
  while (true) {
    SynchronizedPackets packets;
    const SyncResult status =
        synchronizer_.TryPop(SteadyNowNanoseconds(), &packets, error);
    if (status == SyncResult::kWaiting) {
      return true;
    }
    if (status == SyncResult::kRejected) {
      return false;
    }
    if (!RunBundle(packets, error)) {
      return false;
    }
  }
}

bool BevFusionComponent::RunBundle(const SynchronizedPackets& packets,
                                   std::string* error) {
  if (packets.lidar == nullptr || packets.lidar->packed_points == nullptr) {
    *error = "synchronizer returned an incomplete LiDAR bundle";
    return false;
  }
  BevFusionInputView input;
  input.timestamp_ns = packets.lidar->timestamp_ns;
  input.point_count = packets.lidar->point_count;
  input.point_feature_count = kPointFeatureCount;
  input.lidar_points = packets.lidar->packed_points->data();
  input.camera_to_lidar = camera_to_lidar_;
  input.camera_intrinsics = camera_intrinsics_;
  input.lidar_to_image = lidar_to_image_;
  input.image_augmentation = image_augmentation_;
  for (std::size_t camera = 0; camera < kCameraCount; ++camera) {
    if (packets.cameras[camera] == nullptr ||
        packets.cameras[camera]->storage == nullptr) {
      *error = std::string("synchronizer returned missing camera ") +
               kCameraOrder[camera];
      return false;
    }
    input.cameras[camera] = packets.cameras[camera]->image;
  }

  BevFusionInferenceResult inference;
  if (!runner_.Run(input, &inference, error)) {
    return false;
  }
  BevFusionFrameResult result;
  result.set_timestamp_ns(inference.timestamp_ns);
  result.set_input_point_count(
      static_cast<std::uint32_t>(inference.input_point_count));
  result.set_inference_ms(inference.inference_ms);
  result.set_model_revision(config_.model().model_revision());
  result.set_calibration_revision(config_.calibration_revision());
  for (const auto& detection : inference.detections) {
    auto* output = result.add_detection();
    output->set_center_x(detection.center_x);
    output->set_center_y(detection.center_y);
    output->set_center_z(detection.center_z);
    output->set_width(detection.width);
    output->set_length(detection.length);
    output->set_height(detection.height);
    output->set_velocity_x(detection.velocity_x);
    output->set_velocity_y(detection.velocity_y);
    output->set_yaw(detection.yaw);
    output->set_score(detection.score);
    output->set_class_id(detection.class_id);
  }
  if (!writer_->Write(
          std::make_shared<BevFusionFrameResult>(std::move(result)))) {
    *error = "failed to publish BEVFusion frame result";
    return false;
  }
  return true;
}

}  // namespace bevfusion
}  // namespace apollo

CYBER_REGISTER_COMPONENT(apollo::bevfusion::BevFusionComponent)
