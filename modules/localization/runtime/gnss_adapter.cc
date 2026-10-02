// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/runtime/gnss_adapter.h"

#include <cmath>

namespace apollo {
namespace localization {
namespace unified {
namespace {

constexpr double kRadians = 3.14159265358979323846 / 180.0;

Eigen::Vector3d Ecef(double latitude, double longitude, double height) {
  constexpr double kSemiMajor = 6378137.0;
  constexpr double kEccentricitySquared = 6.6943799901413165e-3;
  const double radius =
      kSemiMajor / std::sqrt(1.0 - kEccentricitySquared *
                            std::sin(latitude) * std::sin(latitude));
  return {(radius + height) * std::cos(latitude) * std::cos(longitude),
          (radius + height) * std::cos(latitude) * std::sin(longitude),
          (radius * (1.0 - kEccentricitySquared) + height) * std::sin(latitude)};
}

bool UsableSolution(uint32_t type) {
  return type == drivers::gnss::L1_INT || type == drivers::gnss::WIDE_INT ||
         type == drivers::gnss::NARROW_INT ||
         type == drivers::gnss::L1_FLOAT ||
         type == drivers::gnss::IONOFREE_FLOAT ||
         type == drivers::gnss::NARROW_FLOAT ||
         type == drivers::gnss::INS_RTKFIXED ||
         type == drivers::gnss::INS_RTKFLOAT;
}

Result Reject(Reason reason, const std::string& message, double now,
              SourceCounters* source) {
  ++source->rejected;
  source->reason = reason;
  source->last_receive = now;
  source->duplicates += reason == Reason::DUPLICATE;
  source->regressions += reason == Reason::TIMESTAMP_REGRESSION;
  return {reason, message};
}

}  // namespace

Result GeodeticToMap(const MapManifest& manifest, double latitude,
                     double longitude, double ellipsoid_height,
                     Eigen::Vector3d* position) {
  if (position == nullptr || !manifest.georeferenced() ||
      !manifest.has_origin_latitude() || !manifest.has_origin_longitude() ||
      !manifest.has_origin_height() ||
      !std::isfinite(latitude) || std::abs(latitude) > 90.0 ||
      !std::isfinite(longitude) || std::abs(longitude) > 180.0 ||
      !std::isfinite(ellipsoid_height) ||
      !std::isfinite(manifest.origin_latitude()) ||
      std::abs(manifest.origin_latitude()) > 90.0 ||
      !std::isfinite(manifest.origin_longitude()) ||
      std::abs(manifest.origin_longitude()) > 180.0 ||
      !std::isfinite(manifest.origin_height()) ||
      !std::isfinite(manifest.enu_to_map_yaw())) {
    return {Reason::CONFIG_INVALID, "invalid WGS84 map reference"};
  }
  const double lat = manifest.origin_latitude() * kRadians;
  const double lon = manifest.origin_longitude() * kRadians;
  Eigen::Matrix3d ecef_to_enu;
  ecef_to_enu << -std::sin(lon), std::cos(lon), 0.0,
      -std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat),
      std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat);
  *position =
      Eigen::AngleAxisd(manifest.enu_to_map_yaw(), Eigen::Vector3d::UnitZ()) *
      ecef_to_enu *
      (Ecef(latitude * kRadians, longitude * kRadians, ellipsoid_height) -
       Ecef(lat, lon, manifest.origin_height()));
  return {};
}

GnssAdapter::GnssAdapter(const GlobalEstimatorConfig& config,
                         const Eigen::Vector3d& antenna_in_base)
    : config_(config), antenna_in_base_(antenna_in_base) {}

Result GnssAdapter::ValidateConfig() const {
  Eigen::Vector3d origin;
  const auto reference = GeodeticToMap(
      config_.map(), config_.map().origin_latitude(),
      config_.map().origin_longitude(), config_.map().origin_height(), &origin);
  if (!reference.ok()) {
    return reference;
  }
  if (!config_.has_gnss_to_unix_offset() ||
      !std::isfinite(config_.gnss_to_unix_offset()) ||
      (config_.use_gnss_heading() &&
       (!config_.has_heading_to_unix_offset() ||
        !config_.has_heading_mount_yaw() ||
        !std::isfinite(config_.heading_to_unix_offset()) ||
        !std::isfinite(config_.heading_mount_yaw()))) ||
      !antenna_in_base_.allFinite() ||
      !std::isfinite(config_.heading_pair_window()) ||
      config_.heading_pair_window() <= 0.0 ||
      !std::isfinite(config_.max_gnss_std()) || config_.max_gnss_std() <= 0.0 ||
      !std::isfinite(config_.max_heading_std()) ||
      config_.max_heading_std() <= 0.0) {
    return {Reason::CONFIG_INVALID, "GNSS needs explicit clock/mount/quality budgets"};
  }
  return {};
}

Result GnssAdapter::AddHeading(const drivers::gnss::Heading& message,
                               double now) {
  if (!config_.use_gnss_heading()) {
    return Reject(Reason::CONFIG_INVALID, "heading input is disabled", now,
                  &heading_health_);
  }
  const double time = message.measurement_time() + config_.heading_to_unix_offset();
  if (!message.has_measurement_time() || !message.has_solution_status() ||
      message.solution_status() != drivers::gnss::SOL_COMPUTED ||
      !message.has_position_type() || !UsableSolution(message.position_type()) ||
      !message.has_heading() || !message.has_heading_std_dev() ||
      !std::isfinite(now) || now <= 0.0 ||
      !std::isfinite(message.heading()) || message.heading() < 0.0 ||
      message.heading() >= 360.0 || !std::isfinite(message.heading_std_dev()) ||
      message.heading_std_dev() <= 0.0 ||
      message.heading_std_dev() * kRadians > config_.max_heading_std() ||
      !std::isfinite(time) || time <= 0.0 ||
      time - now > config_.future_tolerance() ||
      now - time > config_.max_observation_age()) {
    return Reject(Reason::INVALID_INPUT, "invalid receiver heading/time/uncertainty",
                  now, &heading_health_);
  }
  if (time <= heading_health_.last_measurement) {
    return Reject(time == heading_health_.last_measurement ? Reason::DUPLICATE
                                                          : Reason::TIMESTAMP_REGRESSION,
                  "heading frontier did not advance", now, &heading_health_);
  }
  HeadingSample sample;
  sample.time = time;
  sample.yaw = 3.14159265358979323846 / 2.0 - message.heading() * kRadians +
               config_.heading_mount_yaw() + config_.map().enu_to_map_yaw();
  sample.variance = std::pow(message.heading_std_dev() * kRadians, 2);
  headings_.push_back(sample);
  while (headings_.size() > 1 &&
         time - headings_.front().time > config_.history_duration()) {
    headings_.pop_front();
    ++heading_health_.evictions;
  }
  heading_health_.last_measurement = time;
  heading_health_.last_receive = now;
  heading_health_.reason = Reason::NONE;
  ++heading_health_.accepted;
  return {};
}

Result GnssAdapter::Observe(const drivers::gnss::GnssBestPose& message,
                            double now, const LocalState& local,
                            GlobalObservation* observation,
                            const GlobalAlignment* history) {
  const double time = message.measurement_time() + config_.gnss_to_unix_offset();
  if (observation == nullptr || !message.has_measurement_time() ||
      !std::isfinite(now) || now <= 0.0 ||
      !message.has_sol_status() || message.sol_status() != drivers::gnss::SOL_COMPUTED ||
      !message.has_sol_type() || !UsableSolution(message.sol_type()) ||
      !message.has_datum_id() || message.datum_id() != drivers::gnss::WGS84 ||
      !message.has_latitude() || !message.has_longitude() ||
      !message.has_height_msl() || !message.has_undulation() ||
      !message.has_latitude_std_dev() || !message.has_longitude_std_dev() ||
      !message.has_height_std_dev() ||
      !std::isfinite(message.undulation()) ||
      !std::isfinite(time) || time <= 0.0 ||
      std::abs(time - local.stamp.time) > 1e-6 ||
      time - now > config_.future_tolerance() ||
      now - time > config_.max_observation_age()) {
    return Reject(Reason::INVALID_INPUT, "GNSS solution incomplete/unusable",
                  now, &position_health_);
  }
  for (double sigma : {message.longitude_std_dev(), message.latitude_std_dev(),
                        message.height_std_dev()}) {
    if (!std::isfinite(sigma) || sigma <= 0.0 || sigma > config_.max_gnss_std()) {
      return Reject(Reason::COVARIANCE_EXCEEDED, "invalid GNSS covariance",
                    now, &position_health_);
    }
  }
  if (time <= position_health_.last_measurement) {
    return Reject(time == position_health_.last_measurement ? Reason::DUPLICATE
                                                           : Reason::TIMESTAMP_REGRESSION,
                  "GNSS frontier did not advance", now, &position_health_);
  }
  const HeadingSample* heading = nullptr;
  for (auto it = headings_.rbegin(); it != headings_.rend(); ++it) {
    if (it->time <= time && time - it->time <= config_.heading_pair_window()) {
      heading = &*it;
      break;
    }
  }
  Eigen::Vector3d antenna;
  const auto projected = GeodeticToMap(
      config_.map(), message.latitude(), message.longitude(),
      message.height_msl() + message.undulation(), &antenna);
  if (!projected.ok()) {
    return Reject(projected.reason, projected.message, now, &position_health_);
  }
  *observation = GlobalObservation();
  const Eigen::Matrix3d enu_to_map =
      Eigen::AngleAxisd(config_.map().enu_to_map_yaw(), Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  const Eigen::Vector3d sigmas(message.longitude_std_dev(), message.latitude_std_dev(),
                               message.height_std_dev());
  observation->stamp = {time, now, ++sequence_, config_.clock_id()};
  observation->epoch = local.epoch;
  observation->source = "gnss";
  observation->map_id = config_.map().map_id();
  observation->map_version = config_.map().version();
  observation->calibration_id = config_.map().calibration_id();
  observation->quality_valid = true;
  observation->georeferenced = true;
  observation->covariance.topLeftCorner<3, 3>() =
      enu_to_map * sigmas.array().square().matrix().asDiagonal() *
      enu_to_map.transpose();
  const Eigen::Matrix3d local_rotation = local.orientation.toRotationMatrix();
  const double local_yaw = std::atan2(local_rotation(1, 0), local_rotation(0, 0));
  double aligned_heading = heading == nullptr ? 0.0 : heading->yaw;
  double heading_variance = heading == nullptr ? 0.0 : heading->variance;
  if (heading != nullptr && std::abs(time - heading->time) > 1e-6) {
    LocalState heading_local;
    if (history == nullptr ||
        !history->Lookup(heading->time, &heading_local).ok() ||
        heading_local.epoch != local.epoch) {
      heading = nullptr;
      ++heading_health_.rejected;
      heading_health_.reason = Reason::HISTORY_UNAVAILABLE;
    } else {
      const Eigen::Matrix3d previous_rotation =
          heading_local.orientation.toRotationMatrix();
      const double past_yaw =
          std::atan2(previous_rotation(1, 0), previous_rotation(0, 0));
      const double current_horizontal = local_rotation.col(0).head<2>().squaredNorm();
      const double previous_horizontal = previous_rotation.col(0).head<2>().squaredNorm();
      if (current_horizontal < 1e-6 || previous_horizontal < 1e-6) {
        heading = nullptr;
        ++heading_health_.rejected;
        heading_health_.reason = Reason::DEGENERATE;
      } else {
        aligned_heading += local_yaw - past_yaw;
        heading_variance += 2.0 * (
            local.covariance.block<3, 3>(6, 6).trace() / current_horizontal +
            heading_local.covariance.block<3, 3>(6, 6).trace() / previous_horizontal);
      }
    }
  }
  if (heading == nullptr) {
    observation->kind = GlobalObservation::Kind::POSITION;
    observation->pose.translation() = antenna;
    observation->point_in_base = antenna_in_base_;
    position_health_.last_measurement = time;
    position_health_.last_receive = now;
    position_health_.reason = Reason::NONE;
    ++position_health_.accepted;
    return {};
  }
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(aligned_heading - local_yaw, Eigen::Vector3d::UnitZ())
          .toRotationMatrix() * local_rotation;
  observation->pose = Eigen::Isometry3d::Identity();
  observation->pose.linear() = rotation;
  observation->pose.translation() = antenna - rotation * antenna_in_base_;
  observation->covariance.bottomRightCorner<3, 3>() =
      2.0 * rotation * local.covariance.block<3, 3>(6, 6) * rotation.transpose();
  observation->covariance(5, 5) += heading_variance;
  Matrix6d lever = Matrix6d::Identity();
  lever.topRightCorner<3, 3>() = Skew(rotation * antenna_in_base_);
  observation->covariance =
      2.0 * lever * observation->covariance * lever.transpose();
  position_health_.last_measurement = time;
  position_health_.last_receive = now;
  position_health_.reason = Reason::NONE;
  ++position_health_.accepted;
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
