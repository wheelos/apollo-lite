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

#include "modules/drivers/gnss/parser/rtcm3/rtcm3_parser.h"

#include <utility>

#include "cyber/common/log.h"
#include "modules/drivers/gnss/util/util.h"

namespace apollo {
namespace drivers {
namespace gnss {

Rtcm3Parser::Rtcm3Parser(const config::Config &config)
    : Rtcm3Parser(config.is_base_station()) {}

Rtcm3Parser::Rtcm3Parser(bool is_base_station)
    : Parser(), is_base_station_(is_base_station), init_flag_(false) {
  if (1 != init_rtcm(&rtcm_)) {
    init_flag_ = true;
  }

  ephemeris_.Clear();
  observation_.Clear();
}

bool Rtcm3Parser::SetStationPosition() {
  auto iter = station_location_.find(rtcm_.staid);
  if (iter == station_location_.end()) {
    AWARN << "Station " << rtcm_.staid << " has no location info.";
    return false;
  }

  observation_.set_position_x(iter->second.x);
  observation_.set_position_y(iter->second.y);
  observation_.set_position_z(iter->second.z);
  return true;
}

void Rtcm3Parser::FillKepplerOrbit(
    const eph_t &eph, apollo::drivers::gnss::KepplerOrbit *keppler_orbit) {
  keppler_orbit->set_week_num(eph.week);

  keppler_orbit->set_af0(eph.f0);
  keppler_orbit->set_af1(eph.f1);
  keppler_orbit->set_af2(eph.f2);

  keppler_orbit->set_iode(eph.iode);
  keppler_orbit->set_deltan(eph.deln);
  keppler_orbit->set_m0(eph.M0);
  keppler_orbit->set_e(eph.e);

  keppler_orbit->set_roota(std::sqrt(eph.A));

  keppler_orbit->set_toe(eph.toes);
  keppler_orbit->set_toc(eph.tocs);

  keppler_orbit->set_cic(eph.cic);
  keppler_orbit->set_crc(eph.crc);
  keppler_orbit->set_cis(eph.cis);
  keppler_orbit->set_crs(eph.crs);
  keppler_orbit->set_cuc(eph.cuc);
  keppler_orbit->set_cus(eph.cus);

  keppler_orbit->set_omega0(eph.OMG0);
  keppler_orbit->set_omega(eph.omg);
  keppler_orbit->set_i0(eph.i0);
  keppler_orbit->set_omegadot(eph.OMGd);
  keppler_orbit->set_idot(eph.idot);

  // keppler_orbit->set_codesonL2channel(eph.);
  keppler_orbit->set_l2pdataflag(eph.flag);
  keppler_orbit->set_accuracy(eph.sva);
  keppler_orbit->set_health(eph.svh);
  keppler_orbit->set_tgd(eph.tgd[0]);
  keppler_orbit->set_iodc(eph.iodc);

  int prn = 0;
  satsys(eph.sat, &prn);
  keppler_orbit->set_sat_prn(prn);
}

void Rtcm3Parser::FillGlonassOrbit(const geph_t &eph,
                                   apollo::drivers::gnss::GlonassOrbit *orbit) {
  orbit->set_position_x(eph.pos[0]);
  orbit->set_position_y(eph.pos[1]);
  orbit->set_position_z(eph.pos[2]);

  orbit->set_velocity_x(eph.vel[0]);
  orbit->set_velocity_y(eph.vel[1]);
  orbit->set_velocity_z(eph.vel[2]);

  orbit->set_accelerate_x(eph.acc[0]);
  orbit->set_accelerate_y(eph.acc[1]);
  orbit->set_accelerate_z(eph.acc[2]);

  orbit->set_health(eph.svh);
  orbit->set_clock_offset(-eph.taun);
  orbit->set_clock_drift(eph.gamn);
  orbit->set_infor_age(eph.age);

  orbit->set_frequency_no(eph.frq);
  // orbit->set_toe(eph.toe.time + eph.toe.sec);
  // orbit->set_tk(eph.tof.time + eph.tof.sec);

  int week = 0;

  double second = time2gpst(eph.toe, &week);
  orbit->set_week_num(week);
  orbit->set_week_second_s(second);
  orbit->set_toe(second);

  second = time2gpst(eph.tof, &week);
  orbit->set_tk(second);

  orbit->set_gnss_time_type(apollo::drivers::gnss::GnssTimeType::GPS_TIME);

  int prn = 0;
  satsys(eph.sat, &prn);
  orbit->set_slot_prn(prn);
}

void Rtcm3Parser::SetObservationTime() {
  int week = 0;
  double second = time2gpst(rtcm_.time, &week);
  observation_.set_gnss_time_type(apollo::drivers::gnss::GPS_TIME);
  observation_.set_gnss_week(week);
  observation_.set_gnss_second_s(second);
}

std::vector<Parser::ParsedMessage> Rtcm3Parser::ParseAllMessages() {
  std::vector<Parser::ParsedMessage> parsed_messages;

  while (auto byte_opt = buffer_.Poll()) {
    uint8_t byte = *byte_opt;
    const int status = input_rtcm3(&rtcm_, byte);

    switch (status) {
      case 0:  // No message ready, need more data. Continue loop to feed next
               // byte.
        // AINFO_EVERY(500) << "input_rtcm3 status 0: Need more data.";
        break;  // Continue while loop

      case 1:  // Observation data ready (e.g., RTCM 1074-1078, 1084-1088, etc.)
        AINFO << "input_rtcm3 status 1: Observation data ready. Msg type: "
              << rtcm_.msgtype;
        // Process the ready observation data from rtcm_ structure
        if (ProcessObservation(&observation_)) {  // Use adapted helper
          // Create ParsedMessage by copying from the internal member
          auto msg_ptr =
              std::make_unique<apollo::drivers::gnss::EpochObservation>();
          msg_ptr->CopyFrom(observation_);
          parsed_messages.emplace_back(MessageType::OBSERVATION,
                                       std::move(msg_ptr));
        }
        break;

      case 2:  // Ephemeris data ready (e.g., RTCM 1019, 1020, 1042, 1045, 1046,
               // etc.)
        AINFO << "input_rtcm3 status 2: Ephemeris data ready. Msg type: "
              << rtcm_.msgtype;
        if (ProcessEphemerides(&ephemeris_)) {
          auto msg_ptr =
              std::make_unique<apollo::drivers::gnss::GnssEphemeris>();
          msg_ptr->CopyFrom(ephemeris_);
          parsed_messages.emplace_back(MessageType::EPHEMERIDES,
                                       std::move(msg_ptr));
          // ephemeris_.Clear(); // Optional
        }
        break;

      case 3:  // Station Auxiliary Data (e.g. antenna type, etc.)
        AINFO
            << "input_rtcm3 status 3: Station Auxiliary data ready. Msg type: "
            << rtcm_.msgtype;
        // If needed, add a ProcessStationAuxiliary() helper and create a
        // message here.
        break;

      case 4:  // Untyped product specific messages
        AINFO << "input_rtcm3 status 4: Untyped product specific message. Msg "
                 "type: "
              << rtcm_.msgtype;
        // If needed, add a handler for these messages.
        break;

      case 5:  // Station Position or Grid Info (e.g., RTCM 1005, 1006, 1007,
               // 1008)
        AINFO << "input_rtcm3 status 5: Station info ready. Msg type: "
              << rtcm_.msgtype;
        ProcessStationParameters();
        break;

      case 10:  // SSR messages (State Space Representation)
        AINFO_EVERY(100)
            << "input_rtcm3 status 10: SSR message ready. Msg type: "
            << rtcm_.msgtype;
        // SSR messages are complex. If needed, add a HandleSSR() helper here.
        break;

      case -1:  // Input data error
        AERROR_EVERY(100)
            << "input_rtcm3 status -1: Input data error processing byte "
            << std::hex << static_cast<int>(byte) << std::dec
            << ". Buffer ReadableBytes: " << buffer_.ReadableBytes();
        break;

      case -2:  // RTCM message length error
        AERROR_EVERY(100) << "input_rtcm3 status -2: RTCM message length error "
                             "processing byte "
                          << std::hex << static_cast<int>(byte) << std::dec
                          << ". Buffer ReadableBytes: " << buffer_.ReadableBytes();
        break;

      case -3:  // RTCM message CRC error
        AERROR_EVERY(100)
            << "input_rtcm3 status -3: RTCM message CRC error processing byte "
            << std::hex << static_cast<int>(byte) << std::dec
            << ". Msg type: " << rtcm_.msgtype
            << ". Buffer ReadableBytes: " << buffer_.ReadableBytes();
        // input_rtcm3 likely discards the message. Continue loop.
        break;

      default:  // Other possible status codes? (Consult RTKLIB docs if needed)
        AWARN_EVERY(100) << "input_rtcm3 returned unknown status: " << status
                         << " processing byte " << std::hex
                         << static_cast<int>(byte) << std::dec
                         << ". Msg type: " << rtcm_.msgtype
                         << ". Buffer ReadableBytes: " << buffer_.ReadableBytes();
        break;
    }
  }

  return parsed_messages;
}

bool Rtcm3Parser::ProcessObservation(
    apollo::drivers::gnss::EpochObservation *observation_msg) {
  if (rtcm_.obs.n == 0) {
    AWARN << "Obs is zero.";
  }

  observation_.Clear();
  SetStationPosition();
  if (!is_base_station_) {
    observation_.set_receiver_id(0);
  } else {
    observation_.set_receiver_id(rtcm_.staid + 0x80000000);
  }

  // set time
  SetObservationTime();

  // set satellite obs
  observation_.set_sat_obs_num(rtcm_.obs.n);
  observation_.set_health_flag(rtcm_.stah);

  for (int i = 0; i < rtcm_.obs.n; ++i) {
    int prn = 0;
    int sys = 0;

    sys = satsys(rtcm_.obs.data[i].sat, &prn);

    apollo::drivers::gnss::GnssType gnss_type;

    // transform sys to local sys type
    if (!gnss_sys_type(sys, &gnss_type)) {
      return false;
    }

    auto sat_obs = observation_.add_sat_obs();  // create obj
    sat_obs->set_sat_prn(prn);
    sat_obs->set_sat_sys(gnss_type);

    int j = 0;

    for (j = 0; j < NFREQ + NEXOBS; ++j) {
      if (is_zero(rtcm_.obs.data[i].L[j])) {
        break;
      }

      apollo::drivers::gnss::GnssBandID baud_id;
      if (!gnss_baud_id(gnss_type, j, &baud_id)) {
        break;
      }

      auto band_obs = sat_obs->add_band_obs();
      if (rtcm_.obs.data[i].code[i] == CODE_L1C) {
        band_obs->set_pseudo_type(
            apollo::drivers::gnss::PseudoType::CORSE_CODE);
      } else if (rtcm_.obs.data[i].code[i] == CODE_L1P) {
        band_obs->set_pseudo_type(
            apollo::drivers::gnss::PseudoType::PRECISION_CODE);
      } else {
        // AINFO << "Message type " << rtcm_.message_type;
      }

      band_obs->set_band_id(baud_id);
      band_obs->set_pseudo_range(rtcm_.obs.data[i].P[j]);
      band_obs->set_carrier_phase(rtcm_.obs.data[i].L[j]);
      band_obs->set_loss_lock_index(rtcm_.obs.data[i].SNR[j]);
      band_obs->set_doppler(rtcm_.obs.data[i].D[j]);
      band_obs->set_snr(rtcm_.obs.data[i].SNR[j]);
    }
    sat_obs->set_band_obs_num(j);
  }

  return true;
}

bool Rtcm3Parser::ProcessEphemerides(
    apollo::drivers::gnss::GnssEphemeris *ephemeris_msg) {
  apollo::drivers::gnss::GnssType gnss_type;

  if (!gnss_sys(rtcm_.message_type, &gnss_type)) {
    AINFO << "Failed get gnss type from message type " << rtcm_.message_type;
    return false;
  }

  apollo::drivers::gnss::GnssTimeType time_type;
  gnss_time_type(gnss_type, &time_type);

  AINFO << "Gnss sys " << static_cast<int>(gnss_type) << "ephemeris info.";

  ephemeris_.Clear();
  ephemeris_.set_gnss_type(gnss_type);

  if (gnss_type == apollo::drivers::gnss::GnssType::GLO_SYS) {
    auto obit = ephemeris_.mutable_glonass_orbit();
    obit->set_gnss_type(gnss_type);
    obit->set_gnss_time_type(time_type);
    FillGlonassOrbit(rtcm_.nav.geph[rtcm_.ephsat - 1], obit);
  } else {
    auto obit = ephemeris_.mutable_keppler_orbit();
    obit->set_gnss_type(gnss_type);
    obit->set_gnss_time_type(time_type);
    FillKepplerOrbit(rtcm_.nav.eph[rtcm_.ephsat - 1], obit);
  }

  return true;
}

bool Rtcm3Parser::ProcessStationParameters() {
  // station pose/ant parameters, set pose.

  // update station location
  auto iter = station_location_.find(rtcm_.staid);
  if (iter == station_location_.end()) {
    Point3D point;
    AINFO << "Add pose for station id: " << rtcm_.staid;
    point.x = rtcm_.sta.pos[0];
    point.y = rtcm_.sta.pos[1];
    point.z = rtcm_.sta.pos[2];
    station_location_.insert(std::make_pair(rtcm_.staid, point));
  } else {
    iter->second.x = rtcm_.sta.pos[0];
    iter->second.y = rtcm_.sta.pos[1];
    iter->second.z = rtcm_.sta.pos[2];
  }
  return true;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
