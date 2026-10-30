// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_gga_adapter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "proj.h"

namespace apollo::localization::openloc_diagnostic {
namespace loc = ::openloc::localization;
namespace {

class InputError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

double Number(const std::string& text) {
  if (text.empty() || !std::all_of(text.begin(), text.end(), [](char c) {
        return (c >= '0' && c <= '9') || c == '.';
      }))
    throw InputError("invalid_gga_numeric_field");
  std::size_t used = 0;
  double value;
  try {
    value = std::stod(text, &used);
  } catch (const std::invalid_argument&) {
    throw InputError("invalid_gga_numeric_field");
  } catch (const std::out_of_range&) {
    throw InputError("gga_numeric_field_out_of_range");
  }
  if (used != text.size() || !std::isfinite(value))
    throw InputError("invalid_gga_numeric_field");
  return value;
}

uint32_t Count(const std::string& text) {
  if (text.empty() || text.size() > 9 ||
      !std::all_of(text.begin(), text.end(),
                   [](char c) { return c >= '0' && c <= '9'; }))
    throw InputError("invalid_gga_integer_field");
  return static_cast<uint32_t>(std::stoul(text));
}

int Hex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

int64_t Measurement(const std::string& text, int64_t receive) {
  const auto dot = text.find('.');
  const auto digits = dot == std::string::npos ? text.size() : dot;
  if (digits != 6 || (dot != std::string::npos &&
      (text.size() <= 7 || text.size() > 16)))
    throw InputError("invalid_gga_utc_time");
  const auto hours = Count(text.substr(0, 2));
  const auto minutes = Count(text.substr(2, 2));
  const auto seconds = Count(text.substr(4, 2));
  if (hours >= 24 || minutes >= 60 || seconds >= 60)
    throw InputError("invalid_gga_utc_time");
  int64_t fraction = 0;
  if (dot != std::string::npos) {
    auto value = text.substr(7);
    Count(value);
    value.append(9 - value.size(), '0');
    fraction = std::stoll(value);
  }
  constexpr int64_t day = 86400000000000LL;
  const int64_t tod =
      (hours * 3600LL + minutes * 60LL + seconds) * 1000000000LL + fraction;
  const int64_t base = (receive / day) * day;
  // Limit receive time to safe arithmetic range; leap seconds are not guessed.
  int64_t candidate = base + tod;
  if (candidate - receive > day / 2) candidate -= day;
  if (receive - candidate > day / 2) candidate += day;
  return candidate;
}

double Coordinate(const std::string& text, const std::string& hemisphere,
                   bool latitude) {
  const std::size_t width = latitude ? 4 : 5;
  const auto dot = text.find('.');
  if ((dot == std::string::npos ? text.size() : dot) != width ||
      (hemisphere != (latitude ? "N" : "E") &&
       hemisphere != (latitude ? "S" : "W")))
    throw InputError("invalid_gga_coordinate");
  const double value = Number(text);
  const double degree = std::floor(value / 100);
  const double minutes = value - degree * 100;
  if (minutes >= 60 || degree + minutes / 60 > (latitude ? 90 : 180))
    throw InputError("invalid_gga_coordinate");
  return (degree + minutes / 60) *
      ((hemisphere == "S" || hemisphere == "W") ? -1 : 1);
}

}  // namespace

class NativeGgaAdapter::Projection {
 public:
  explicit Projection(const NativeRawGgaConfig& config) {
    context = proj_context_create();
    if (!context) throw std::runtime_error("cannot create GNSS projection context");
    // Explicit WGS84 geographic degrees -> UTM metres, no EPSG database or
    // external grid lookup. This adapter does not perform datum corrections.
    const std::string pipeline =
        "+proj=pipeline +step +proj=unitconvert +xy_in=deg +xy_out=rad "
        "+step +proj=utm +ellps=WGS84 +zone=" +
        std::to_string(config.utm_zone()) +
        (config.northern_hemisphere() ? "" : " +south");
    transform = proj_create(context, pipeline.c_str());
    if (!transform) {
      proj_context_destroy(context);
      throw std::runtime_error("cannot create configured WGS84/UTM projection");
    }
  }
  ~Projection() {
    proj_destroy(transform);
    proj_context_destroy(context);
  }
  PJ_CONTEXT* context = nullptr;
  PJ* transform = nullptr;
};

NativeGgaAdapter::NativeGgaAdapter(const NativeGlobalHostConfig& config)
    : config_(config) {
  const auto& raw = config.raw_gga();
  if (!config.has_raw_gga() || !raw.IsInitialized() ||
      raw.utm_zone() < 1 || raw.utm_zone() > 60 ||
      !std::isfinite(raw.origin_easting_m()) ||
      !std::isfinite(raw.origin_northing_m()) ||
      !std::isfinite(raw.lever_x()) || !std::isfinite(raw.lever_y()) ||
      !std::isfinite(raw.lever_z()) ||
      !std::isfinite(raw.diagnostic_sigma_m()) || raw.diagnostic_sigma_m() <= 0 ||
      !std::isfinite(raw.diagnostic_sigma_m() * raw.diagnostic_sigma_m()) ||
      raw.preparation_id().empty())
    throw std::invalid_argument("invalid raw GGA configuration");
  loc::GnssPositionGatePolicy policy;
  policy.minimum_satellites = raw.minimum_satellites();
  policy.maximum_hdop = raw.maximum_hdop();
  policy.maximum_differential_age_seconds = raw.maximum_differential_age_s();
  policy.fixed_solution_stability_ns = raw.fixed_stability_ns();
  policy.maximum_fixed_solution_gap_ns = raw.maximum_gap_ns();
  policy.allow_missing_horizontal_accuracy_diagnostic = true;
  const auto status = gate_.Configure(policy);
  if (!status.ok()) throw std::invalid_argument(status.message());
  projection_ = std::make_unique<Projection>(raw);
}

NativeGgaAdapter::~NativeGgaAdapter() = default;

void NativeGgaAdapter::Finish() const {
  if (!buffer_.empty())
    throw std::runtime_error("raw NMEA input ended with an incomplete sentence");
}

void NativeGgaAdapter::Reset(const std::string& session, uint64_t generation,
                             int64_t minimum_measurement_ns) {
  session_ = session;
  generation_ = generation;
  minimum_measurement_ns_ = minimum_measurement_ns;
  buffer_.clear();
  gate_.Reset();
}

void NativeGgaAdapter::Feed(const std::string& bytes, int64_t receive,
                           Callback callback) {
  if (bytes.empty() || bytes.size() > 4096 || receive <= 0 ||
      receive > std::numeric_limits<int64_t>::max() - 2 * 86400000000000LL ||
      receive < last_receive_ns_ || !callback)
    throw std::invalid_argument("invalid raw GGA packet/receive frontier");
  last_receive_ns_ = receive;
  for (char c : bytes) {
    if (c == '$') {
      if (!buffer_.empty()) {
        gate_.Reset();
        callback({++sequence_, 0, receive,
                  {loc::StatusCode::kInvalidArgument, "truncated_nmea_sentence"},
                  std::nullopt});
      }
      buffer_ = "$";
    } else if (!buffer_.empty() && c == '\n') {
      if (buffer_.back() == '\r') buffer_.pop_back();
      // Other NMEA types are not GNSS position observations.
      if (buffer_.rfind("$GPGGA,", 0) == 0 || buffer_.rfind("$GNGGA,", 0) == 0)
        callback(Parse(buffer_, receive));
      buffer_.clear();
    } else if (!buffer_.empty()) {
      if (buffer_.size() >= 256)
        throw std::runtime_error("NMEA sentence exceeded bounded 256 bytes");
      buffer_ += c;
    }
  }
}

NativeGgaDecision NativeGgaAdapter::Parse(const std::string& sentence,
                                        int64_t receive) {
  NativeGgaDecision decision;
  decision.sequence = ++sequence_;
  decision.receive_ns = receive;
  try {
    const auto star = sentence.find('*');
    if (star == std::string::npos || star + 3 != sentence.size() ||
        Hex(sentence[star + 1]) < 0 || Hex(sentence[star + 2]) < 0)
      throw InputError("missing_or_invalid_nmea_checksum");
    unsigned checksum = 0;
    for (std::size_t i = 1; i < star; ++i)
      checksum ^= static_cast<unsigned char>(sentence[i]);
    if (checksum != static_cast<unsigned>(Hex(sentence[star + 1]) * 16 +
                                          Hex(sentence[star + 2])))
      throw InputError("nmea_checksum_mismatch");
    std::vector<std::string> fields;
    std::size_t start = 1;
    for (std::size_t i = 1; i <= star; ++i) {
      if (i == star || sentence[i] == ',') {
        fields.push_back(sentence.substr(start, i - start));
        start = i + 1;
      }
    }
    if (fields.size() != 15) throw InputError("invalid_gga_field_count");
    decision.measurement_ns = Measurement(fields[1], receive);
    if (decision.measurement_ns <= minimum_measurement_ns_)
      throw InputError("gga_measurement_precedes_reset_frontier");
    const auto& raw = config_.raw_gga();
    const double latitude = Coordinate(fields[2], fields[3], true);
    const double longitude = Coordinate(fields[4], fields[5], false);
    const auto quality = Count(fields[6]);
    loc::GnssReceiverQuality receiver;
    receiver.carrier_solution =
        quality == 4 ? loc::GnssCarrierSolution::kFixed :
        quality == 5 ? loc::GnssCarrierSolution::kFloat :
                       loc::GnssCarrierSolution::kInvalid;
    receiver.satellite_count = Count(fields[7]);
    receiver.hdop = Number(fields[8]);
    receiver.differential_age_seconds = Number(fields[13]);
    receiver.horizontal_accuracy_m = std::numeric_limits<double>::quiet_NaN();
    // Invalid geometry/time must not contribute to fixed-solution dwell.
    if (latitude < -80 || latitude > 84 ||
        (latitude >= 0) != raw.northern_hemisphere() ||
        longitude < raw.utm_zone() * 6.0 - 186.0 ||
        longitude > raw.utm_zone() * 6.0 - 180.0)
      throw InputError("gga_outside_declared_utm_zone");
    if (decision.measurement_ns - receive > 20000000 ||
        receive - decision.measurement_ns > 2000000000LL)
      throw InputError("gga_stale_or_future_measurement");
    proj_errno_reset(projection_->transform);
    const auto xy = proj_trans(projection_->transform, PJ_FWD,
                               proj_coord(longitude, latitude, 0, 0));
    if (proj_errno(projection_->transform) || !std::isfinite(xy.xy.x) ||
        !std::isfinite(xy.xy.y))
      throw std::runtime_error("configured GNSS projection failed");
    decision.status = gate_.Check(
        {decision.measurement_ns, loc::ClockDomain::kUnixEpoch}, receiver);
    if (decision.status.code() == loc::StatusCode::kInvalidArgument)
      gate_.Reset();
    if (!decision.status.ok()) return decision;
    if (session_.empty() || generation_ == 0) {
      decision.status = {loc::StatusCode::kStateUnavailable,
                         "native_odometry_epoch_unavailable"};
      return decision;
    }
    NativeGlobalGnssInput observation;
    observation.set_session(session_);
    observation.set_generation(generation_);
    observation.set_sequence(decision.sequence);
    observation.set_measurement_ns(decision.measurement_ns);
    observation.set_receive_ns(receive);
    observation.set_reference_id(config_.reference_id());
    observation.set_reference_version(config_.reference_version());
    observation.set_calibration_id(config_.calibration_id());
    observation.set_content_id(config_.content_id());
    observation.set_preparation_id(raw.preparation_id());
    observation.set_covariance_source("explicit-uncalibrated-gga-sigma");
    observation.set_x(xy.xy.x - raw.origin_easting_m());
    observation.set_y(xy.xy.y - raw.origin_northing_m());
    observation.set_covariance_xx(raw.diagnostic_sigma_m() * raw.diagnostic_sigma_m());
    observation.set_covariance_xy(0);
    observation.set_covariance_yy(observation.covariance_xx());
    observation.set_lever_x(raw.lever_x());
    observation.set_lever_y(raw.lever_y());
    observation.set_lever_z(raw.lever_z());
    observation.set_receiver_admitted(true);
    observation.set_recorded_input(config_.recorded_input());
    observation.set_diagnostic_only(true);
    observation.set_gnss_map_construction(raw.gnss_map_construction());
    if (raw.has_graph_qualification())
      *observation.mutable_graph_qualification() = raw.graph_qualification();
    decision.observation = std::move(observation);
  } catch (const InputError& error) {
    gate_.Reset();
    decision.status = {loc::StatusCode::kInvalidArgument, error.what()};
  }
  return decision;
}

}  // namespace apollo::localization::openloc_diagnostic
