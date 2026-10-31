// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_structured_gnss_adapter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "proj.h"

namespace apollo::localization::openloc_diagnostic {
namespace loc = ::openloc::localization;
namespace {
constexpr double kPi = 3.14159265358979323846;
loc::Status Invalid(const char* detail) {
  return {loc::StatusCode::kInvalidArgument, detail};
}
loc::Status Rejected(const char* detail) {
  return {loc::StatusCode::kRejectedMeasurement, detail};
}
bool Positive(double value) { return std::isfinite(value) && value > 0; }
}  // namespace

class NativeStructuredGnssAdapter::Projection {
 public:
  explicit Projection(const NativeStructuredGnssConfig& config) {
    context_ = proj_context_create();
    if (!context_) throw std::runtime_error("cannot create structured GNSS projection context");
    const std::string pipeline = "+proj=utm +ellps=WGS84 +zone=" +
        std::to_string(config.utm_zone()) +
        (config.northern_hemisphere() ? "" : " +south");
    transform_ = proj_create(context_, pipeline.c_str());
    if (!transform_) {
      proj_context_destroy(context_);
      throw std::runtime_error("cannot create structured GNSS projection");
    }
  }
  ~Projection() {
    proj_destroy(transform_);
    proj_context_destroy(context_);
  }
  PJ_COORD Forward(double longitude_rad, double latitude_rad) {
    proj_errno_reset(transform_);
    const auto xy = proj_trans(transform_, PJ_FWD,
        proj_coord(longitude_rad, latitude_rad, 0, 0));
    if (proj_errno(transform_) || !std::isfinite(xy.xy.x) ||
        !std::isfinite(xy.xy.y))
      throw std::runtime_error("structured GNSS coordinate projection failed");
    return xy;
  }
 private:
  PJ_CONTEXT* context_ = nullptr;
  PJ* transform_ = nullptr;
};

NativeStructuredGnssAdapter::NativeStructuredGnssAdapter(
    const NativeGlobalHostConfig& config) : config_(config) {
  const auto& policy = config.structured_gnss();
  if (!config.has_structured_gnss() || config.has_raw_gga() ||
      !policy.IsInitialized() || policy.utm_zone() < 1 || policy.utm_zone() > 60 ||
      !std::isfinite(policy.measurement_to_unix_offset_s()) ||
      !std::isfinite(policy.origin_easting_m()) ||
      !std::isfinite(policy.origin_northing_m()) ||
      !std::isfinite(policy.lever_x()) || !std::isfinite(policy.lever_y()) ||
      !std::isfinite(policy.lever_z()) || policy.preparation_id().empty() ||
      policy.maximum_measurement_age_ns() <= 0 ||
      policy.maximum_measurement_age_ns() > 30000000000LL)
    throw std::invalid_argument("invalid structured GNSS profile");
  loc::GnssPositionGatePolicy gate;
  gate.minimum_satellites = policy.minimum_satellites();
  gate.maximum_differential_age_seconds = policy.maximum_differential_age_s();
  gate.maximum_horizontal_accuracy_m = policy.maximum_horizontal_std_m();
  gate.fixed_solution_stability_ns = policy.fixed_stability_ns();
  gate.maximum_fixed_solution_gap_ns = policy.maximum_gap_ns();
  gate.allow_missing_hdop = true;
  const auto status = position_gate_.Configure(gate);
  if (!status.ok()) throw std::invalid_argument(status.message());
  if (policy.has_heading()) {
    const auto& heading = policy.heading();
    const double length = std::hypot(std::hypot(heading.baseline_x(),
        heading.baseline_y()), heading.baseline_z());
    if (!heading.IsInitialized() || heading.topic().empty() ||
        !Positive(length) || !Positive(std::hypot(heading.baseline_x(), heading.baseline_y())) ||
        !std::isfinite(heading.true_north_yaw_rad()) ||
        !Positive(heading.maximum_std_rad()) ||
        !std::isfinite(heading.maximum_baseline_error_m()) ||
        heading.maximum_baseline_error_m() < 0 ||
        heading.minimum_satellites() == 0 || heading.fixed_stability_ns() < 0 ||
        heading.maximum_gap_ns() <= 0)
      throw std::invalid_argument("invalid dual-antenna GNSS profile");
  }
  projection_ = std::make_unique<Projection>(policy);
}

NativeStructuredGnssAdapter::~NativeStructuredGnssAdapter() = default;

void NativeStructuredGnssAdapter::Reset(const std::string& session,
    uint64_t generation, int64_t minimum_measurement_ns) {
  session_ = session;
  generation_ = generation;
  minimum_measurement_ns_ = minimum_measurement_ns;
  position_gate_.Reset();
  heading_fixed_start_ns_ = 0;
  last_heading_ns_ = 0;
}

loc::Status NativeStructuredGnssAdapter::Time(double source, int64_t receive,
                                             int64_t* measurement) const {
  const long double ns = (static_cast<long double>(source) +
      config_.structured_gnss().measurement_to_unix_offset_s()) * 1e9L;
  if (!std::isfinite(source) || source <= 0 || !std::isfinite(ns) ||
      ns <= 0 || ns > std::numeric_limits<int64_t>::max() - 1.0L || receive <= 0)
    return Invalid("invalid_structured_gnss_source_time");
  *measurement = static_cast<int64_t>(std::round(ns));
  if (*measurement <= minimum_measurement_ns_)
    return Rejected("structured_gnss_precedes_reset_frontier");
  if (*measurement - receive > 20000000 ||
      receive - *measurement > config_.structured_gnss().maximum_measurement_age_ns())
    return Rejected("structured_gnss_stale_or_future_measurement");
  return loc::Status::Ok();
}

NativeStructuredPosition NativeStructuredGnssAdapter::Position(
    const drivers::gnss::GnssBestPose& input, int64_t receive) {
  NativeStructuredPosition result;
  result.sequence = ++position_sequence_;
  result.receive_ns = receive;
  const auto reject = [&](loc::Status status) {
    position_gate_.Reset();
    result.status = std::move(status);
    return result;
  };
  if (!input.has_measurement_time() || !input.has_sol_status() ||
      !input.has_sol_type() || !input.has_latitude() || !input.has_longitude() ||
      !input.has_datum_id() || !input.has_latitude_std_dev() ||
      !input.has_longitude_std_dev() || !input.has_differential_age() ||
      !input.has_num_sats_in_solution())
    return reject(Invalid("incomplete_gnss_best_pose"));
  result.status = Time(input.measurement_time(), receive, &result.measurement_ns);
  if (!result.status.ok()) return reject(result.status);
  const auto& policy = config_.structured_gnss();
  if (!std::isfinite(input.latitude()) || !std::isfinite(input.longitude()) ||
      input.latitude() < -80 || input.latitude() > 84 ||
      (input.latitude() >= 0) != policy.northern_hemisphere() ||
      input.longitude() < policy.utm_zone() * 6.0 - 186.0 ||
      input.longitude() > policy.utm_zone() * 6.0 - 180.0 ||
      input.datum_id() != drivers::gnss::WGS84 ||
      !Positive(input.latitude_std_dev()) || !Positive(input.longitude_std_dev()))
    return reject(Invalid("invalid_gnss_best_pose_geometry_or_uncertainty"));
  if (input.sol_status() != drivers::gnss::SOL_COMPUTED ||
      input.sol_type() != drivers::gnss::NARROW_INT)
    return reject(Rejected("gnss_best_pose_not_pure_fixed_solution"));
  loc::GnssReceiverQuality receiver;
  receiver.carrier_solution = loc::GnssCarrierSolution::kFixed;
  receiver.satellite_count = input.num_sats_in_solution();
  receiver.hdop = std::numeric_limits<double>::quiet_NaN();
  receiver.differential_age_seconds = input.differential_age();
  receiver.horizontal_accuracy_m =
      std::max(input.latitude_std_dev(), input.longitude_std_dev());
  result.status = position_gate_.Check(
      {result.measurement_ns, loc::ClockDomain::kUnixEpoch}, receiver);
  if (!result.status.ok()) return result;
  if (session_.empty() || generation_ == 0) {
    result.status = {loc::StatusCode::kStateUnavailable, "native_odometry_epoch_unavailable"};
    return result;
  }
  const double latitude = input.latitude() * kPi / 180;
  const double longitude = input.longitude() * kPi / 180;
  const auto xy = projection_->Forward(longitude, latitude);
  // Local tangent ENU -> UTM Jacobian, including convergence and scale.
  constexpr double a = 6378137.0;
  constexpr double e2 = 0.0066943799901413165;
  const double denominator = 1 - e2 * std::pow(std::sin(latitude), 2);
  const double east_radius = a / std::sqrt(denominator) * std::cos(latitude);
  const double north_radius = a * (1 - e2) / std::pow(denominator, 1.5);
  constexpr double step = 0.1;
  const auto east = projection_->Forward(longitude + step / east_radius, latitude);
  const auto north = projection_->Forward(longitude, latitude + step / north_radius);
  const double ex = (east.xy.x - xy.xy.x) / step;
  const double ey = (east.xy.y - xy.xy.y) / step;
  const double nx = (north.xy.x - xy.xy.x) / step;
  const double ny = (north.xy.y - xy.xy.y) / step;
  const double ve = std::pow(input.longitude_std_dev(), 2);
  const double vn = std::pow(input.latitude_std_dev(), 2);
  auto& observation = result.observation.emplace();
  observation.set_session(session_);
  observation.set_generation(generation_);
  observation.set_sequence(result.sequence);
  observation.set_measurement_ns(result.measurement_ns);
  observation.set_receive_ns(receive);
  observation.set_reference_id(config_.reference_id());
  observation.set_reference_version(config_.reference_version());
  observation.set_calibration_id(config_.calibration_id());
  observation.set_content_id(config_.content_id());
  observation.set_x(xy.xy.x - policy.origin_easting_m());
  observation.set_y(xy.xy.y - policy.origin_northing_m());
  observation.set_covariance_xx(ex * ex * ve + nx * nx * vn);
  observation.set_covariance_xy(ex * ey * ve + nx * ny * vn);
  observation.set_covariance_yy(ey * ey * ve + ny * ny * vn);
  observation.set_lever_x(policy.lever_x());
  observation.set_lever_y(policy.lever_y());
  observation.set_lever_z(policy.lever_z());
  observation.set_preparation_id(policy.preparation_id());
  observation.set_receiver_admitted(true);
  observation.set_recorded_input(config_.recorded_input());
  observation.set_diagnostic_only(true);
  observation.set_covariance_source("receiver-enu-diagonal-projected-uncalibrated");
  observation.set_gnss_map_construction(policy.gnss_map_construction());
  if (policy.has_position_graph_qualification())
    *observation.mutable_graph_qualification() = policy.position_graph_qualification();
  return result;
}

NativeStructuredHeading NativeStructuredGnssAdapter::Heading(
    const drivers::gnss::Heading& input, int64_t receive) {
  NativeStructuredHeading result;
  result.sequence = ++heading_sequence_;
  result.receive_ns = receive;
  const auto reject = [&](loc::Status status) {
    heading_fixed_start_ns_ = 0;
    result.status = std::move(status);
    return result;
  };
  const auto& policy = config_.structured_gnss().heading();
  if (!config_.structured_gnss().has_heading() ||
      !input.has_measurement_time() || !input.has_solution_status() ||
      !input.has_position_type() || !input.has_baseline_length() ||
      !input.has_heading() || !input.has_heading_std_dev() ||
      !input.has_satellite_soulution_number())
    return reject(Invalid("incomplete_dual_antenna_heading"));
  result.status = Time(input.measurement_time(), receive, &result.measurement_ns);
  if (!result.status.ok()) return reject(result.status);
  if (last_heading_ns_ && result.measurement_ns <= last_heading_ns_)
    return reject(Rejected("out_of_order_dual_antenna_heading"));
  if (last_heading_ns_ && result.measurement_ns - last_heading_ns_ > policy.maximum_gap_ns())
    heading_fixed_start_ns_ = 0;
  last_heading_ns_ = result.measurement_ns;
  const double std_rad = input.heading_std_dev() * kPi / 180;
  const double length = std::hypot(std::hypot(policy.baseline_x(),
      policy.baseline_y()), policy.baseline_z());
  if (input.solution_status() != drivers::gnss::SOL_COMPUTED ||
      input.position_type() != drivers::gnss::NARROW_INT ||
      input.satellite_soulution_number() < policy.minimum_satellites() ||
      !std::isfinite(input.heading()) || input.heading() < 0 || input.heading() >= 360 ||
      !Positive(std_rad) || std_rad > policy.maximum_std_rad() ||
      !Positive(input.baseline_length()) ||
      std::abs(input.baseline_length() - length) > policy.maximum_baseline_error_m())
    return reject(Rejected("dual_antenna_heading_quality_or_baseline_rejected"));
  if (!heading_fixed_start_ns_) heading_fixed_start_ns_ = result.measurement_ns;
  if (result.measurement_ns - heading_fixed_start_ns_ < policy.fixed_stability_ns()) {
    result.status = {loc::StatusCode::kPending, "dual_antenna_waiting_for_stability"};
    return result;
  }
  result.baseline_yaw_rad = std::remainder(policy.true_north_yaw_rad() -
      input.heading() * kPi / 180, 2 * kPi);
  result.variance_rad2 = std_rad * std_rad;
  result.status = loc::Status::Ok();
  return result;
}

}  // namespace apollo::localization::openloc_diagnostic
