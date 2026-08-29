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

//  Created Date: 2026-08-28
//  Author: daohu527

#include "modules/lane/component/lane_detection_component.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace lane {

namespace {

apollo::perception::camera::LaneLinePositionType ToWirePosition(
    LanePosition position) {
  switch (position) {
    case LanePosition::kEgoLeft:
      return apollo::perception::camera::EGO_LEFT;
    case LanePosition::kEgoRight:
      return apollo::perception::camera::EGO_RIGHT;
    case LanePosition::kAdjacentLeft:
      return apollo::perception::camera::ADJACENT_LEFT;
    case LanePosition::kAdjacentRight:
      return apollo::perception::camera::ADJACENT_RIGHT;
    case LanePosition::kThirdLeft:
      return apollo::perception::camera::THIRD_LEFT;
    case LanePosition::kThirdRight:
      return apollo::perception::camera::THIRD_RIGHT;
    case LanePosition::kUnknown:
    default:
      return apollo::perception::camera::UNKNOWN;
  }
}

bool ToCameraTimestamp(double timestamp_sec, uint64_t* timestamp_ns) {
  if (timestamp_ns == nullptr || !std::isfinite(timestamp_sec) ||
      timestamp_sec < 0.0 ||
      timestamp_sec >
          static_cast<double>(std::numeric_limits<uint64_t>::max()) * 1.0e-9) {
    return false;
  }
  *timestamp_ns = static_cast<uint64_t>(timestamp_sec * 1.0e9);
  return true;
}

}  // namespace

bool LaneDetectionComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Unable to load lane component configuration";
    return false;
  }
  if (config_.engine_path().empty() || config_.camera_name().empty() ||
      config_.source_topic().empty() || !config_.use_static_pitch()) {
    AERROR << "The lane component requires an engine, camera/source names, and "
              "explicit static-pitch calibration";
    return false;
  }
  StaticCameraCalibration calibration;
  calibration.focal_x = config_.focal_x();
  calibration.focal_y = config_.focal_y();
  calibration.principal_x = config_.principal_x();
  calibration.principal_y = config_.principal_y();
  calibration.camera_height_meters = config_.camera_height_meters();
  calibration.pitch_radians = config_.static_pitch_radians();
  if (!calibrator_.Init(calibration)) {
    AERROR << "Invalid static camera calibration";
    return false;
  }
  TensorRtEngineOptions engine_options;
  engine_options.engine_path = config_.engine_path();
  engine_options.device_id = config_.gpu_device_id();
  engine_options.tensor_names.input = config_.input_tensor_name();
  engine_options.tensor_names.loc_row = config_.loc_row_tensor_name();
  engine_options.tensor_names.loc_col = config_.loc_col_tensor_name();
  engine_options.tensor_names.exist_row = config_.exist_row_tensor_name();
  engine_options.tensor_names.exist_col = config_.exist_col_tensor_name();
  executor_.reset(new TensorRtUfldv2Executor);
  if (!executor_->Init(engine_options)) {
    AERROR << "Unable to initialize the UFLDv2 TensorRT engine";
    return false;
  }
  detector_.reset(new Ufldv2Detector(executor_.get()));
  writer_ = node_->CreateWriter<apollo::perception::PerceptionLanes>(
      config_.output_channel());
  return writer_ != nullptr;
}

bool LaneDetectionComponent::MakeImageView(
    const apollo::drivers::Image& message, ImageView* image_view) const {
  if (image_view == nullptr) {
    return false;
  }
  ImageEncoding encoding;
  size_t expected_byte_count = 0;
  if (!ParseImageEncoding(message.encoding(), &encoding) ||
      !ImageView::ExpectedByteCount(message.width(), message.height(),
                                    &expected_byte_count) ||
      message.step() != expected_byte_count / message.height() ||
      message.data().size() != expected_byte_count) {
    AERROR << "Rejected image payload with unsupported encoding, dimensions, "
              "stride, or byte count";
    return false;
  }
  image_view->bytes = reinterpret_cast<const uint8_t*>(message.data().data());
  image_view->byte_count = message.data().size();
  image_view->width = message.width();
  image_view->height = message.height();
  image_view->encoding = encoding;
  image_view->timestamp_sec = message.measurement_time();
  if (message.has_header()) {
    const auto& header = message.header();
    if (header.has_camera_timestamp()) {
      image_view->camera_timestamp_ns = header.camera_timestamp();
    }
    if (header.has_sequence_num()) {
      image_view->sequence_num = header.sequence_num();
    }
  }
  if (image_view->camera_timestamp_ns == 0) {
    ToCameraTimestamp(image_view->timestamp_sec,
                      &image_view->camera_timestamp_ns);
  }
  image_view->frame_id = message.frame_id();
  if (image_view->frame_id.empty() && message.has_header() &&
      message.header().has_frame_id()) {
    image_view->frame_id = message.header().frame_id();
  }
  image_view->camera_name = config_.camera_name();
  std::string error;
  if (!image_view->Validate(&error)) {
    AERROR << "Rejected image payload: " << error;
    return false;
  }
  return true;
}

bool LaneDetectionComponent::Serialize(
    const LaneDetectionResult& result,
    apollo::perception::PerceptionLanes* message) const {
  if (message == nullptr) {
    return false;
  }
  message->set_source_topic(config_.source_topic());
  auto* header = message->mutable_header();
  header->set_timestamp_sec(result.timestamp_sec);
  header->set_module_name("lane_detection");
  if (result.sequence_num != 0U) {
    header->set_sequence_num(result.sequence_num);
  }
  if (!result.frame_id.empty()) {
    header->set_frame_id(result.frame_id);
  }
  uint64_t timestamp_ns = 0;
  if (result.camera_timestamp_ns != 0U) {
    header->set_camera_timestamp(result.camera_timestamp_ns);
  } else if (ToCameraTimestamp(result.timestamp_sec, &timestamp_ns)) {
    header->set_camera_timestamp(timestamp_ns);
  }
  message->mutable_camera_calibrator()->set_pitch_angle(
      result.calibration_pitch_radians);
  message->mutable_camera_calibrator()->set_camera_height(
      result.camera_height_meters);
  for (const LaneLineResult& lane : result.lanes) {
    auto* wire_lane = message->add_camera_laneline();
    wire_lane->set_pos_type(ToWirePosition(lane.position));
    wire_lane->set_track_id(static_cast<int32_t>(lane.candidate_id));
    wire_lane->set_confidence(lane.confidence);
    wire_lane->set_use_type(apollo::perception::camera::REAL);
    for (const ImagePoint& point : lane.image_points) {
      auto* wire_point = wire_lane->add_curve_image_point_set();
      wire_point->set_x(point.x);
      wire_point->set_y(point.y);
    }
    for (const CameraPoint& point : lane.camera_points) {
      auto* wire_point = wire_lane->add_curve_camera_point_set();
      wire_point->set_x(point.x);
      wire_point->set_y(point.y);
      wire_point->set_z(point.z);
    }
    if (lane.camera_curve.valid) {
      auto* curve = wire_lane->mutable_curve_camera_coord();
      curve->set_longitude_min(lane.camera_curve.longitude_min);
      curve->set_longitude_max(lane.camera_curve.longitude_max);
      curve->set_a(lane.camera_curve.a);
      curve->set_b(lane.camera_curve.b);
      curve->set_c(lane.camera_curve.c);
      curve->set_d(lane.camera_curve.d);
    }
    if (lane.image_points.size() >= 2U) {
      auto* endpoints = wire_lane->add_image_end_point_set();
      endpoints->mutable_start()->set_x(lane.image_points.front().x);
      endpoints->mutable_start()->set_y(lane.image_points.front().y);
      endpoints->mutable_end()->set_x(lane.image_points.back().x);
      endpoints->mutable_end()->set_y(lane.image_points.back().y);
    }
  }
  return true;
}

bool LaneDetectionComponent::Proc(
    const std::shared_ptr<apollo::drivers::Image>& image) {
  if (image == nullptr || detector_ == nullptr || writer_ == nullptr) {
    return false;
  }
  ImageView image_view;
  if (!MakeImageView(*image, &image_view)) {
    return false;
  }
  if (config_.require_camera_to_vehicle_transform()) {
    const std::string camera_frame = config_.camera_frame_id().empty()
                                         ? image->frame_id()
                                         : config_.camera_frame_id();
    std::string transform_error;
    if (!transform_adapter_.HasCameraToVehicleTransform(
            camera_frame, config_.vehicle_frame_id(), image_view.timestamp_sec,
            config_.transform_timeout_sec(), &transform_error)) {
      AERROR << "Camera transform is unavailable: " << transform_error;
      return false;
    }
  }
  LaneDetectionResult result;
  if (!detector_->Detect(image_view, &result) ||
      !calibrator_.PopulateCameraCoordinates(&result)) {
    AERROR << "UFLDv2 lane inference or static calibration failed";
    return false;
  }
  std::shared_ptr<apollo::perception::PerceptionLanes> output(
      new apollo::perception::PerceptionLanes);
  if (!Serialize(result, output.get())) {
    AERROR << "Failed to serialize UFLDv2 lane output";
    return false;
  }
  writer_->Write(output);
  return true;
}

}  // namespace lane
}  // namespace apollo
