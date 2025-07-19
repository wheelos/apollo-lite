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

#include "modules/drivers/gnss/parser/data_parser.h"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include <proj.h>

#include "Eigen/Geometry"
#include "boost/array.hpp"

#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/time_conversion.h"
#include "modules/drivers/gnss/parser/parser_factory.h"
#include "modules/drivers/gnss/util/util.h"

namespace apollo {
namespace drivers {
namespace gnss {

using ::apollo::localization::CorrectedImu;
using ::apollo::localization::Gps;

using apollo::common::util::GpsToUnixSeconds;
using apollo::transform::TransformStamped;

namespace {

// Using EPSG:4326 for WGS84 geographic is standard.
// Alternatively, could use "+proj=latlong +ellps=WGS84 +datum=WGS84" with
// proj_create
const char* WGS84_TEXT = "EPSG:4326";

// covariance data for pose if can not get from novatel inscov topic
// Marked constexpr as it's compile-time constant
static constexpr boost::array<double, 36> POSE_COVAR = {
    2, 0, 0, 0,    0, 0, 0, 2, 0, 0, 0,    0, 0, 0, 2, 0, 0, 0,
    0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01};

}  // namespace

DataParser::DataParser(const config::Config& config,
                       const std::shared_ptr<apollo::cyber::Node>& node)
    : config_(config), tf_broadcaster_(node), node_(node) {
  proj_context_ = proj_context_create();
  if (!proj_context_) {
    AFATAL << "Failed to create PROJ context";
  }
  proj_transform_ = proj_create_crs_to_crs(
      proj_context_, WGS84_TEXT, config_.proj4_text().c_str(), nullptr);
  PJ* normalized_transform =
      proj_normalize_for_visualization(proj_context_, proj_transform_);
  proj_destroy(proj_transform_);
  proj_transform_ = normalized_transform;
  if (!proj_transform_) {
    proj_context_destroy(proj_context_);
    proj_context_ = nullptr;
    AFATAL << "Failed to create PROJ transformation from " << WGS84_TEXT
           << " to " << config_.proj4_text();
  }

  gnss_status_.set_solution_status(0);
  gnss_status_.set_num_sats(0);
  gnss_status_.set_position_type(0);
  gnss_status_.set_solution_completed(false);
  ins_status_.set_type(InsStatus::INVALID);
}

// Destructor to clean up PROJ resources
DataParser::~DataParser() {
  if (proj_transform_) {
    proj_destroy(proj_transform_);
    proj_transform_ = nullptr;
  }
  if (proj_context_) {
    proj_context_destroy(proj_context_);
    proj_context_ = nullptr;
  }
}

bool DataParser::Init() {
  // Check if PROJ initialization failed in constructor
  if (!proj_context_ || !proj_transform_) {
    AFATAL << "PROJ objects not initialized. Cannot proceed.";
    return false;
  }

  gnssstatus_writer_ = node_->CreateWriter<GnssStatus>(FLAGS_gnss_status_topic);
  insstatus_writer_ = node_->CreateWriter<InsStatus>(FLAGS_ins_status_topic);
  gnssbestpose_writer_ =
      node_->CreateWriter<GnssBestPose>(FLAGS_gnss_best_pose_topic);
  corrimu_writer_ = node_->CreateWriter<CorrectedImu>(FLAGS_imu_topic);
  insstat_writer_ = node_->CreateWriter<InsStat>(FLAGS_ins_stat_topic);
  gnssephemeris_writer_ =
      node_->CreateWriter<GnssEphemeris>(FLAGS_gnss_rtk_eph_topic);
  epochobservation_writer_ =
      node_->CreateWriter<EpochObservation>(FLAGS_gnss_rtk_obs_topic);
  heading_writer_ = node_->CreateWriter<Heading>(FLAGS_heading_topic);
  rawimu_writer_ = node_->CreateWriter<Imu>(FLAGS_raw_imu_topic);
  gps_writer_ = node_->CreateWriter<Gps>(FLAGS_gps_topic);

  // Fill and publish initial status
  common::util::FillHeader("gnss", &ins_status_);
  insstatus_writer_->Write(ins_status_);
  common::util::FillHeader("gnss", &gnss_status_);
  gnssstatus_writer_->Write(gnss_status_);

  AINFO << "Creating data parser of format: " << config_.data().format();
  gnss_parser_ = ParserFactory::Create(config_);
  CHECK_NOTNULL(gnss_parser_);

  init_flag_ = true;
  return true;
}

void DataParser::ParseRawData(const std::string& msg) {
  if (!init_flag_) {
    AERROR << "Data parser not init or PROJ initialization failed.";
    return;
  }

  gnss_parser_->AppendData(msg);
  auto messages = gnss_parser_->ParseAllMessages();

  for (const auto& [msg_type, msg_variant] : messages) {
    if (std::holds_alternative<Parser::RawDataPtr>(msg_variant)) {
      // Store raw byte messages. Currently only GPGGA is stored.
      // This is needed if raw NMEA strings are required elsewhere.
      if (msg_type == Parser::MessageType::GPGGA) {
        auto raw_ptr = std::get<Parser::RawDataPtr>(msg_variant);
        message_map_[Parser::MessageType::GPGGA].first =
            cyber::Time::Now().ToNanosecond();
        message_map_[Parser::MessageType::GPGGA].second = *raw_ptr;
      } else {
        // Handle other raw message types if needed, or ignore
        ADEBUG << "Received unhandled raw byte message type: "
               << static_cast<int>(msg_type);
      }
    } else if (std::holds_alternative<Parser::ProtoMessagePtr>(msg_variant)) {
      DispatchMessage(msg_type, std::get<Parser::ProtoMessagePtr>(msg_variant));
    } else {
      AERROR << "Unknown message type variant.";
    }
  }
}

void DataParser::CheckInsStatus(const std::shared_ptr<Ins>& ins) {
  if (ins == nullptr) {
    AERROR << "Received null Ins message";
    return;
  }

  static double last_notify = cyber::Time().Now().ToSecond();
  double now = cyber::Time::Now().ToSecond();

  // Only update and publish if status changes or if 1 second has passed
  if (ins_status_record_ != static_cast<uint32_t>(ins->type()) ||
      (now - last_notify) > 1.0) {
    last_notify = now;
    ins_status_record_ = static_cast<uint32_t>(ins->type());
    switch (ins->type()) {
      case Ins::GOOD:
        ins_status_.set_type(InsStatus::GOOD);
        break;
      case Ins::CONVERGING:
        ins_status_.set_type(InsStatus::CONVERGING);
        break;
      case Ins::INVALID:
      default:
        ins_status_.set_type(InsStatus::INVALID);
        break;
    }
    common::util::FillHeader("gnss", &ins_status_);
    insstatus_writer_->Write(ins_status_);
  }
}

void DataParser::CheckGnssStatus(const std::shared_ptr<Gnss>& gnss) {
  // Update status always, publish is handled implicitly by Cyber writer if
  // needed. Could add logic here to only publish on status change if desired,
  // similar to CheckInsStatus
  if (gnss == nullptr) {
    AERROR << "Received null Gnss message";
    return;
  }

  gnss_status_.set_solution_status(
      static_cast<uint32_t>(gnss->solution_status()));
  gnss_status_.set_num_sats(static_cast<uint32_t>(gnss->num_sats()));
  gnss_status_.set_position_type(static_cast<uint32_t>(gnss->position_type()));

  if (static_cast<uint32_t>(gnss->solution_status()) == 0) {
    gnss_status_.set_solution_completed(true);
  } else {
    gnss_status_.set_solution_completed(false);
  }
  common::util::FillHeader("gnss", &gnss_status_);
  gnssstatus_writer_->Write(gnss_status_);
}

void DataParser::DispatchMessage(Parser::MessageType type,
                                 const Parser::ProtoMessagePtr& msg_ptr) {
  CHECK_NOTNULL(msg_ptr);
  switch (type) {
    case Parser::MessageType::GNSS:
      CheckGnssStatus(std::dynamic_pointer_cast<Gnss>(msg_ptr));
      break;
    case Parser::MessageType::BEST_GNSS_POS:
      PublishBestpos(msg_ptr);
      break;
    case Parser::MessageType::IMU:
      PublishImu(msg_ptr);
      break;
    case Parser::MessageType::INS:
      CheckInsStatus(std::dynamic_pointer_cast<Ins>(msg_ptr));
      PublishCorrimu(msg_ptr);
      PublishOdometry(msg_ptr);
      break;
    case Parser::MessageType::INS_STAT:
      PublishInsStat(msg_ptr);
      break;
    case Parser::MessageType::BDSEPHEMERIDES:
    case Parser::MessageType::GPSEPHEMERIDES:
    case Parser::MessageType::GLOEPHEMERIDES:
      PublishEphemeris(msg_ptr);
      break;
    case Parser::MessageType::OBSERVATION:
      PublishObservation(msg_ptr);
      break;
    case Parser::MessageType::HEADING:
      PublishHeading(msg_ptr);
      break;
    default:
      ADEBUG << "Received unhandled Protobuf message type: "
             << static_cast<int>(type);
      break;
  }
}

void DataParser::PublishInsStat(const Parser::ProtoMessagePtr& msg_ptr) {
  // Get the underlying protobuf message
  auto ins_stat_ptr = std::dynamic_pointer_cast<InsStat>(msg_ptr);
  if (!ins_stat_ptr) {
    AERROR << "Failed to cast message to InsStat";
    return;
  }
  common::util::FillHeader("gnss", ins_stat_ptr.get());
  insstat_writer_->Write(ins_stat_ptr);
}

void DataParser::PublishBestpos(const Parser::ProtoMessagePtr& msg_ptr) {
  // Get the underlying protobuf message
  auto bestpos_ptr = std::dynamic_pointer_cast<GnssBestPose>(msg_ptr);
  if (!bestpos_ptr) {
    AERROR << "Failed to cast message to GnssBestPose";
    return;
  }
  // Create message directly and publish
  common::util::FillHeader("gnss", bestpos_ptr.get());
  gnssbestpose_writer_->Write(bestpos_ptr);
}

void DataParser::PublishImu(const Parser::ProtoMessagePtr& msg_ptr) {
  auto imu_in = std::dynamic_pointer_cast<Imu>(msg_ptr);
  if (!imu_in) {
    AERROR << "Failed to cast message to Imu";
    return;
  }
  // Create a *new* Imu message and populate it after transformation
  auto raw_imu_out = std::make_shared<Imu>();

  // --- Coordinate System Transformation (Example: Novatel IMU to Apollo
  // Vehicle) --- Assuming sensor frame: +X forward, +Y right, +Z down Assuming
  // target frame (Apollo): +X forward, +Y left, +Z up Rotation: Swap X and Y,
  // negate new X (+Y in sensor -> -X in target), keep Z Angular Velocity:
  // Similarly transform angular velocities
  raw_imu_out->mutable_linear_acceleration()->set_x(
      -imu_in->linear_acceleration()
           .y());  // Sensor +Y (right) becomes Target -X (forward)
  raw_imu_out->mutable_linear_acceleration()->set_y(
      imu_in->linear_acceleration()
          .x());  // Sensor +X (forward) becomes Target +Y (left)
  raw_imu_out->mutable_linear_acceleration()->set_z(
      imu_in->linear_acceleration()
          .z());  // Sensor +Z (down) becomes Target +Z (up) - Check sign based
                  // on convention

  raw_imu_out->mutable_angular_velocity()->set_x(
      -imu_in->angular_velocity()
           .y());  // Sensor +Y (right) becomes Target -X (roll)
  raw_imu_out->mutable_angular_velocity()->set_y(
      imu_in->angular_velocity()
          .x());  // Sensor +X (forward) becomes Target +Y (pitch)
  raw_imu_out->mutable_angular_velocity()->set_z(
      imu_in->angular_velocity()
          .z());  // Sensor +Z (down) becomes Target +Z (yaw) - Check sign based
                  // on convention

  common::util::FillHeader("gnss", raw_imu_out.get());
  rawimu_writer_->Write(raw_imu_out);  // Using shared_ptr for created message
}

void DataParser::PublishOdometry(const Parser::ProtoMessagePtr& msg_ptr) {
  auto ins = std::dynamic_pointer_cast<Ins>(msg_ptr);
  if (!ins) {
    AERROR << "Failed to cast message to Ins for Odometry";
    return;
  }
  // Create a *new* Gps message (used for Odometry topic in Apollo)
  auto gps = std::make_shared<Gps>();

  double unix_sec = GpsToUnixSeconds(ins->measurement_time());
  gps->mutable_header()->set_timestamp_sec(unix_sec);
  auto* gps_msg = gps->mutable_localization();

  // 1. pose xyz (WGS84 to UTM transformation)
  double lon = ins->position().lon();
  double lat = ins->position().lat();

  PJ_COORD src = proj_coord(lon, lat, 0.0, 0.0);
  PJ_COORD dst = proj_trans(proj_transform_, PJ_FWD, src);
  // Easting (transformed longitude)
  gps_msg->mutable_position()->set_x(dst.xy.x);
  // Northing (transformed latitude)
  gps_msg->mutable_position()->set_y(dst.xy.y);
  gps_msg->mutable_position()->set_z(ins->position().height());

  Eigen::Quaterniond q =
      Eigen::AngleAxisd(ins->euler_angles().z() - 90 * kDegToRad,
                        Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-ins->euler_angles().y(), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(ins->euler_angles().x(), Eigen::Vector3d::UnitY());
  // Comment: This quaternion composition applies rotations around Z, X, and Y
  // axes with specific angle modifications based on the sensor's reported Euler
  // angles (ins->euler_angles().x(), ins->euler_angles().y(),
  // ins->euler_angles().z()). The exact meaning of sensor's Euler angles (e.g.,
  // order, positive direction, reference frame) and the reason for these
  // specific transformations (e.g., -90 deg yaw offset, negating pitch,
  // swapping axes in Eigen AngleAxisd) should be verified against the sensor
  // documentation and Apollo's coordinate conventions.

  gps_msg->mutable_orientation()->set_qx(q.x());
  gps_msg->mutable_orientation()->set_qy(q.y());
  gps_msg->mutable_orientation()->set_qz(q.z());
  gps_msg->mutable_orientation()->set_qw(q.w());

  // 3. Linear velocity
  // Assuming linear velocity in vehicle body frame (X-forward, Y-left, Z-up)
  // Novatel often reports velocity in the NED or ENU frame.
  // Original code copies directly: Assuming sensor velocity is already in
  // vehicle body frame. If sensor reports velocity in NED/ENU, it needs
  // rotation by the vehicle's orientation quaternion. Let's assume for now
  // sensor velocity is compatible or this transformation is done elsewhere.
  // Comment: Assuming sensor reports linear velocity in the target vehicle body
  // frame.
  gps_msg->mutable_linear_velocity()->set_x(ins->linear_velocity().x());
  gps_msg->mutable_linear_velocity()->set_y(ins->linear_velocity().y());
  gps_msg->mutable_linear_velocity()->set_z(ins->linear_velocity().z());

  gps_writer_->Write(gps);  // Using shared_ptr for created message
  if (config_.tf().enable()) {
    TransformStamped transform;
    // Pass by reference as GpsToTransformStamped modifies the object
    GpsToTransformStamped(gps, &transform);
    tf_broadcaster_.SendTransform(transform);
  }
}

void DataParser::PublishCorrimu(const Parser::ProtoMessagePtr& msg_ptr) {
  auto ins = std::dynamic_pointer_cast<Ins>(msg_ptr);
  if (!ins) {
    AERROR << "Failed to cast message to Ins for Corrimu";
    return;
  }
  // Create a *new* CorrectedImu message
  auto imu = std::make_shared<CorrectedImu>();
  double unix_sec = GpsToUnixSeconds(ins->measurement_time());
  imu->mutable_header()->set_timestamp_sec(unix_sec);

  auto* imu_msg = imu->mutable_imu();

  // --- Coordinate System Transformation (Example: Novatel INS to Apollo
  // Vehicle) --- Similar to PublishImu, transforming linear acceleration and
  // angular velocity based on assumed sensor vs. target frame conventions.
  // Verify these transformations against sensor documentation and Apollo
  // conventions.
  imu_msg->mutable_linear_acceleration()->set_x(
      -ins->linear_acceleration()
           .y());  // Sensor +Y (right) becomes Target -X (forward)
  imu_msg->mutable_linear_acceleration()->set_y(
      ins->linear_acceleration()
          .x());  // Sensor +X (forward) becomes Target +Y (left)
  imu_msg->mutable_linear_acceleration()->set_z(
      ins->linear_acceleration()
          .z());  // Sensor +Z (down) becomes Target +Z (up) - Check sign

  imu_msg->mutable_angular_velocity()->set_x(
      -ins->angular_velocity()
           .y());  // Sensor +Y (right) becomes Target -X (roll)
  imu_msg->mutable_angular_velocity()->set_y(
      ins->angular_velocity()
          .x());  // Sensor +X (forward) becomes Target +Y (pitch)
  imu_msg->mutable_angular_velocity()->set_z(
      ins->angular_velocity()
          .z());  // Sensor +Z (down) becomes Target +Z (yaw) - Check sign

  // --- Euler Angle Transformation ---
  // Applying specific transformations to Euler angles.
  // The meaning of sensor's Euler angles (ins->euler_angles().x/.y/.z) and
  // the reason for negation and yaw offset should be verified.
  // Assuming sensor angles are Roll(X), Pitch(Y), Yaw(Z).
  imu_msg->mutable_euler_angles()->set_x(
      ins->euler_angles().x());  // Roll (kept as is)
  imu_msg->mutable_euler_angles()->set_y(
      -ins->euler_angles().y());  // Pitch (negated)
  imu_msg->mutable_euler_angles()->set_z(
      ins->euler_angles().z() - 90 * kDegToRad);  // Yaw (-90 deg offset)
  // Comment: The Euler angles are transformed based on sensor specifications
  // and target frame requirements. Specifically, pitch is negated and a -90
  // degree offset is applied to the yaw angle. Verify this transformation
  // against sensor documentation and Apollo's coordinate conventions.

  corrimu_writer_->Write(imu);
}

void DataParser::PublishEphemeris(const Parser::ProtoMessagePtr& msg_ptr) {
  auto eph_ptr = std::dynamic_pointer_cast<GnssEphemeris>(msg_ptr);
  if (!eph_ptr) {
    AERROR << "Failed to cast message to GnssEphemeris";
    return;
  }
  gnssephemeris_writer_->Write(eph_ptr);  // Use const& overload
}

void DataParser::PublishObservation(const Parser::ProtoMessagePtr& msg_ptr) {
  // Get the underlying protobuf message
  auto observation_ptr = std::dynamic_pointer_cast<EpochObservation>(msg_ptr);
  if (!observation_ptr) {
    AERROR << "Failed to cast message to EpochObservation";
    return;
  }
  // Create message directly and publish
  // Observation message often doesn't have standard header, but can fill if
  // needed common::util::FillHeader("gnss", &observation);
  epochobservation_writer_->Write(observation_ptr);
}

void DataParser::PublishHeading(const Parser::ProtoMessagePtr& msg_ptr) {
  // Get the underlying protobuf message
  auto heading_ptr = std::dynamic_pointer_cast<Heading>(msg_ptr);
  if (!heading_ptr) {
    AERROR << "Failed to cast message to Heading";
    return;
  }
  // Create message directly and publish
  common::util::FillHeader("gnss", heading_ptr.get());
  heading_writer_->Write(heading_ptr);  // Use const& overload
}

void DataParser::GpsToTransformStamped(const std::shared_ptr<Gps>& gps,
                                       TransformStamped* transform) {
  CHECK_NOTNULL(gps);
  CHECK_NOTNULL(transform);

  transform->mutable_header()->set_timestamp_sec(gps->header().timestamp_sec());
  transform->mutable_header()->set_frame_id(config_.tf().frame_id());
  transform->set_child_frame_id(config_.tf().child_frame_id());
  auto translation = transform->mutable_transform()->mutable_translation();
  translation->set_x(gps->localization().position().x());
  translation->set_y(gps->localization().position().y());
  translation->set_z(gps->localization().position().z());
  auto rotation = transform->mutable_transform()->mutable_rotation();
  rotation->set_qx(gps->localization().orientation().qx());
  rotation->set_qy(gps->localization().orientation().qy());
  rotation->set_qz(gps->localization().orientation().qz());
  rotation->set_qw(gps->localization().orientation().qw());
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
