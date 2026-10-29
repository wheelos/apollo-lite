// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <cmath>
#include <cstdio>

#include "native_global_worker.h"

namespace apollo::localization::openloc_diagnostic::test {

inline NativeGlobalHostConfig Config(const std::string& journal) {
  NativeGlobalHostConfig config;
  config.set_odometry_topic("/openloc/diagnostic/global-test/odom");
  config.set_gnss_topic("/openloc/diagnostic/global-test/gnss");
  config.set_output_topic("/openloc/diagnostic/global-test/result");
  config.set_reference_id("synthetic-site");
  config.set_reference_version("v1");
  config.set_calibration_id("synthetic-calibration");
  config.set_content_id("synthetic-enu-contract");
  config.set_journal(journal);
  config.set_assume_gravity_aligned_odom(true);
  config.set_recorded_input(true);
  config.set_diagnostic_position_sigma_m(0.1);
  config.set_diagnostic_rotation_sigma_rad(0.01);
  config.set_maximum_odometry_age_ms(200);
  return config;
}

inline void EnableTargetGraph(NativeGlobalHostConfig* config) {
  auto* graph = config->mutable_target_graph();
  graph->set_motion_error_model_id("native-frozen-linearization");
  graph->set_maximum_error_dimension(1024);
  graph->set_history_duration_ms(60);
  graph->set_maximum_observation_age_ms(60);
  graph->set_maximum_states(100);
  graph->set_solve_budget_ms(100000);
  graph->set_verification_frames(2);
  graph->set_allow_uncalibrated_inputs(true);
  graph->set_assume_deterministic_map(true);
  graph->set_scan_voxel_m(0.05);
  graph->set_minimum_scan_points(20);
  graph->set_normal_neighbors(6);
  graph->set_motion_error_model_qualified(true);
}

inline NativeOdometryDiagnostic Odometry(int index, uint64_t generation = 1) {
  NativePublication publisher(0.2);
  NativePublication::State state;
  state.stamp = {100 + index * 0.1, 100 + index * 0.1,
                 static_cast<uint64_t>(index + 1), "unix"};
  state.epoch = {"synthetic-global-test", generation};
  state.valid = true;
  state.position.x() = index;
  const auto now = NativePublication::Clock::now();
  publisher.Update(state, {}, now);
  auto output = publisher.Sample(200, now);
  output.set_publication_sequence(index + 1);
  output.set_recorded_input(true);
  return output;
}

inline NativeGlobalGnssInput Gnss(const NativeOdometryDiagnostic& source) {
  const auto config = Config("");
  NativeGlobalGnssInput input;
  input.set_session(source.session());
  input.set_generation(source.generation());
  input.set_sequence(source.source_sequence());
  input.set_measurement_ns(std::llround(source.measurement_time() * 1e9));
  input.set_receive_ns(input.measurement_ns());
  input.set_reference_id(config.reference_id());
  input.set_reference_version(config.reference_version());
  input.set_calibration_id(config.calibration_id());
  input.set_content_id(config.content_id());
  input.set_x(100 + source.odom_from_imu().transform().translation().x());
  input.set_y(20 + source.odom_from_imu().transform().translation().y());
  input.set_covariance_xx(0.01);
  input.set_covariance_xy(0);
  input.set_covariance_yy(0.01);
  input.set_lever_x(0);
  input.set_lever_y(0);
  input.set_lever_z(0);
  input.set_preparation_id("synthetic-fixed-xy");
  input.set_receiver_admitted(true);
  input.set_recorded_input(true);
  input.set_diagnostic_only(true);
  input.set_covariance_source("synthetic-diagnostic-uncalibrated");
  return input;
}

inline void EnableRawGga(NativeGlobalHostConfig* config) {
  auto* raw = config->mutable_raw_gga();
  raw->set_utm_zone(50);
  raw->set_northern_hemisphere(true);
  raw->set_origin_easting_m(500000);
  raw->set_origin_northing_m(4427757.218738374);
  raw->set_lever_x(0);
  raw->set_lever_y(0);
  raw->set_lever_z(0);
  raw->set_diagnostic_sigma_m(1);
  raw->set_preparation_id("synthetic-wgs84-utm50n");
  raw->set_minimum_satellites(6);
  raw->set_maximum_hdop(3);
  raw->set_maximum_differential_age_s(10);
  raw->set_fixed_stability_ns(2000000000LL);
  raw->set_maximum_gap_ns(2000000000LL);
}

inline void EnableReset(NativeGlobalHostConfig* config) {
  config->set_reset_topic("/openloc/diagnostic/global-test/reset");
  config->set_alignment_session("synthetic-global-host");
  config->add_reset_consumers("planner");
  config->add_reset_consumers("observer");
}

inline void EnableStructuredGnss(NativeGlobalHostConfig* config, bool heading = true) {
  auto* policy = config->mutable_structured_gnss();
  policy->set_measurement_to_unix_offset_s(0);
  policy->set_utm_zone(50);
  policy->set_northern_hemisphere(true);
  policy->set_origin_easting_m(500000);
  policy->set_origin_northing_m(4427757.218738374);
  policy->set_lever_x(0);
  policy->set_lever_y(0);
  policy->set_lever_z(0);
  policy->set_preparation_id("synthetic-novatel-utm50n");
  policy->set_minimum_satellites(6);
  policy->set_maximum_horizontal_std_m(0.3);
  policy->set_maximum_differential_age_s(3);
  policy->set_fixed_stability_ns(2000000000LL);
  policy->set_maximum_gap_ns(1500000000LL);
  policy->set_maximum_measurement_age_ns(2000000000LL);
  if (heading) {
    auto* dual = policy->mutable_heading();
    dual->set_topic("/openloc/diagnostic/global-test/heading");
    dual->set_baseline_x(1);
    dual->set_baseline_y(0);
    dual->set_baseline_z(0);
    dual->set_true_north_yaw_rad(1.5707963267948966);
    dual->set_maximum_std_rad(0.035);
    dual->set_maximum_baseline_error_m(0.02);
    dual->set_minimum_satellites(6);
    dual->set_fixed_stability_ns(2000000000LL);
    dual->set_maximum_gap_ns(1500000000LL);
  }
}

inline drivers::gnss::GnssBestPose BestPose(int index) {
  drivers::gnss::GnssBestPose input;
  input.mutable_header()->set_sequence_num(index + 1);
  input.mutable_header()->set_timestamp_sec(100 + index + 0.001);
  input.set_measurement_time(100 + index);
  input.set_sol_status(drivers::gnss::SOL_COMPUTED);
  input.set_sol_type(drivers::gnss::NARROW_INT);
  input.set_latitude(40);
  input.set_longitude(117 + index / (111320.0 * std::cos(40 * 3.141592653589793 / 180)));
  input.set_datum_id(drivers::gnss::WGS84);
  input.set_latitude_std_dev(0.02);
  input.set_longitude_std_dev(0.03);
  input.set_differential_age(1);
  input.set_num_sats_in_solution(20);
  return input;
}

inline drivers::gnss::Heading DualHeading(int index) {
  drivers::gnss::Heading input;
  input.mutable_header()->set_sequence_num(index + 1);
  input.mutable_header()->set_timestamp_sec(100 + index + 0.001);
  input.set_measurement_time(100 + index);
  input.set_solution_status(drivers::gnss::SOL_COMPUTED);
  input.set_position_type(drivers::gnss::NARROW_INT);
  input.set_baseline_length(1);
  input.set_heading(90);
  input.set_heading_std_dev(0.5);
  input.set_satellite_soulution_number(20);
  return input;
}

template <typename T>
T Alignment(T input, uint64_t generation) {
  input.set_alignment_session("synthetic-global-host");
  input.set_alignment_generation(generation);
  return input;
}

inline NativeGlobalResetRequest Reset(uint64_t sequence = 1,
                                      uint64_t expected_generation = 1) {
  NativeGlobalResetRequest request;
  request.set_alignment_session("synthetic-global-host");
  request.set_expected_alignment_generation(expected_generation);
  request.set_sequence(sequence);
  request.set_reason("explicit-operator-reacquisition");
  return request;
}

inline NativeGlobalResetAck ResetAck(const std::string& consumer,
                                     uint64_t sequence = 1,
                                     uint64_t generation = 2) {
  NativeGlobalResetAck ack;
  ack.set_alignment_session("synthetic-global-host");
  ack.set_alignment_generation(generation);
  ack.set_sequence(sequence);
  ack.set_consumer_id(consumer);
  return ack;
}

inline std::string Nmea(const std::string& body) {
  unsigned checksum = 0;
  for (unsigned char c : body) checksum ^= c;
  char suffix[8];
  std::snprintf(suffix, sizeof(suffix), "*%02X\r\n", checksum);
  return "$" + body + suffix;
}

inline std::string Gga(int index, const std::string& quality = "4") {
  char fields[100];
  const int seconds = 100 + index;
  std::snprintf(fields, sizeof(fields),
      "GNGGA,%02d%02d%02d.000000000,4000.0000000,N,11700.%07d,E,",
      seconds / 3600, (seconds / 60) % 60, seconds % 60, index * 7000);
  return Nmea(std::string(fields) + quality + ",12,0.8,30,M,0,M,0.5,0001");
}

inline NativeOdometryDiagnostic GgaOdometry(int index, uint64_t generation = 1) {
  auto source = Odometry(index, generation);
  source.set_measurement_time(100 + index);
  source.set_receive_time(100 + index);
  source.mutable_odom_from_imu()->mutable_header()->set_timestamp_sec(100 + index);
  return source;
}

inline NativeGlobalMapInput Map(const NativeOdometryDiagnostic& source,
                                bool projected = false) {
  const auto gnss = Gnss(source);
  NativeGlobalMapInput input;
  input.set_session(gnss.session());
  input.set_generation(gnss.generation());
  input.set_sequence(gnss.sequence());
  input.set_measurement_ns(gnss.measurement_ns());
  input.set_receive_ns(gnss.receive_ns());
  input.set_reference_id(gnss.reference_id());
  input.set_reference_version(gnss.reference_version());
  input.set_calibration_id(gnss.calibration_id());
  input.set_content_id(gnss.content_id());
  input.set_preparation_id("synthetic-map");
  input.set_covariance_source("synthetic-map-uncalibrated");
  input.set_diagnostic_only(true);
  input.set_recorded_input(true);
  input.set_ambiguous(false);
  input.set_recovery(false);
  input.set_odometry_deskew(NativeGlobalGnssInput::PRESENT);
  input.set_shared_local_scans(NativeGlobalGnssInput::PRESENT);
  input.set_gnss_map_construction(NativeGlobalGnssInput::PRESENT);
  auto* pose = input.mutable_map_from_imu();
  pose->set_x(gnss.x());
  pose->set_y(gnss.y());
  pose->set_z(3);
  pose->set_qx(0);
  pose->set_qy(0);
  pose->set_qz(0);
  pose->set_qw(1);
  if (!projected) {
    input.set_kind(NativeGlobalMapInput::FULL_POSE);
    for (int row = 0; row < 6; ++row)
      for (int col = 0; col < 6; ++col)
        input.add_covariance(row == col ? 0.01 : 0.0);
  } else {
    input.set_kind(NativeGlobalMapInput::PROJECTED_POSE);
    input.set_rank(1);
    for (int col = 0; col < 6; ++col)
      input.add_projection(col == 0 ? 1.0 : 0.0);
    input.add_projected_covariance(0.01);
  }
  return input;
}

}  // namespace apollo::localization::openloc_diagnostic::test
