// Copyright 2026 WheelOS All Rights Reserved.
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

#include "modules/drivers/gnss/parser/novatel/novatel_parser.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace gnss {
namespace {

// Wire offsets and units are from the OEM6/OEM7 log definitions, independent
// of the packed production structures.
template <typename T>
void Put(std::string* data, size_t offset, T value) {
  std::memcpy(data->data() + offset, &value, sizeof(value));
}

uint32_t WireCrc(const std::string& data) {
  uint32_t crc = 0;
  for (unsigned char byte : data) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0u);
    }
  }
  return crc;
}

std::string Frame(uint16_t id, const std::string& payload,
                  bool short_header = true, uint32_t millis = 100000) {
  std::string frame(short_header ? 12 : 28, '\0');
  frame[0] = '\xaa';
  frame[1] = '\x44';
  frame[2] = short_header ? '\x13' : '\x12';
  frame[3] = short_header ? payload.size() : 28;
  Put(&frame, 4, id);
  if (short_header) {
    Put<uint16_t>(&frame, 6, 2200);
    Put(&frame, 8, millis);
  } else {
    Put<uint16_t>(&frame, 8, payload.size());
    frame[13] = static_cast<char>(160);  // FINESTEERING
    Put<uint16_t>(&frame, 14, 2200);
    Put(&frame, 16, millis);
  }
  frame += payload;
  const uint32_t crc = WireCrc(frame);
  frame.append(reinterpret_cast<const char*>(&crc), sizeof(crc));
  return frame;
}

std::string RawPayload(double seconds = 100.0) {
  std::string payload(40, '\0');
  payload[1] = 26;  // ISA100C
  Put<uint16_t>(&payload, 2, 2200);
  Put(&payload, 4, seconds);
  Put<int32_t>(&payload, 16, 2500000);
  Put<int32_t>(&payload, 20, -500000);
  Put<int32_t>(&payload, 24, 1000000);
  Put<int32_t>(&payload, 28, 1500000);
  Put<int32_t>(&payload, 32, -1000000);
  Put<int32_t>(&payload, 36, 500000);
  return payload;
}

std::string CorrPayload(double seconds = 100.0) {
  std::string payload(60, '\0');
  Put<uint32_t>(&payload, 0, 2200);
  Put(&payload, 4, seconds);
  for (int i = 0; i < 6; ++i) {
    Put(&payload, 12 + i * 8, (i + 1) * 0.01);
  }
  return payload;
}

std::string PvaPayload(double seconds = 100.0) {
  std::string payload(88, '\0');
  Put<uint32_t>(&payload, 0, 2200);
  Put(&payload, 4, seconds);
  Put(&payload, 12, 37.0);
  Put(&payload, 20, -122.0);
  Put(&payload, 28, 25.0);
  Put(&payload, 36, 2.0);  // north
  Put(&payload, 44, 3.0);  // east
  Put(&payload, 52, 4.0);
  Put(&payload, 60, 10.0);
  Put(&payload, 68, 20.0);
  Put(&payload, 76, 30.0);
  Put<uint32_t>(&payload, 84, 3);
  return payload;
}

std::vector<Parser::ParsedMessage> Parse(NovatelParser* parser,
                                         const std::string& bytes) {
  parser->AppendData(bytes);
  return parser->ParseAllMessages();
}

template <typename T>
std::shared_ptr<T> Message(const Parser::ParsedMessage& parsed) {
  return std::dynamic_pointer_cast<T>(
      std::get<Parser::ProtoMessagePtr>(parsed.second));
}

TEST(NovatelParserTest, RawImuWireTypeUnitsFramesAndFragmentation) {
  const std::string frame = Frame(novatel::RAWIMUSX, RawPayload());
  for (int mapping : {5, 6}) {
    config::Config config;
    config.mutable_novatel_config()->set_imu_orientation(mapping);
    NovatelParser parser(config);
    std::vector<Parser::ParsedMessage> messages;
    for (char byte : frame) {
      auto batch = Parse(&parser, std::string(1, byte));
      messages.insert(messages.end(), batch.begin(), batch.end());
    }
    ASSERT_EQ(1u, messages.size());
    ASSERT_EQ(Parser::MessageType::IMU, messages[0].first);
    const auto imu = Message<Imu>(messages[0]);
    ASSERT_NE(nullptr, imu);
    EXPECT_DOUBLE_EQ(2200.0 * 604800 + 100, imu->measurement_time());
    EXPECT_NEAR(0.005, imu->measurement_span(), 1e-9);
    EXPECT_DOUBLE_EQ(mapping == 5 ? 2.0 : 4.0, imu->linear_acceleration().x());
    EXPECT_DOUBLE_EQ(mapping == 5 ? -4.0 : -2.0,
                     imu->linear_acceleration().y());
    EXPECT_DOUBLE_EQ(mapping == 5 ? 10.0 : -10.0,
                     imu->linear_acceleration().z());
    EXPECT_NEAR(mapping == 5 ? 0.2 : 0.1, imu->angular_velocity().x(), 1e-12);
    EXPECT_NEAR(mapping == 5 ? -0.1 : -0.2, imu->angular_velocity().y(), 1e-12);
    EXPECT_NEAR(mapping == 5 ? 0.3 : -0.3, imu->angular_velocity().z(), 1e-12);
  }
}

TEST(NovatelParserTest, NativeModelRatesAndScalesMatchOfficialDefinitions) {
  struct Model {
    uint8_t id;
    double rate;
    double gyro_increment;
    double accel_increment;
  };
  constexpr double deg_to_rad = 0.017453292519943295;
  // Official oem7_imu.cpp definitions, not GetImuParameter's return values.
  const Model models[] = {
      {13, 200, (0.1 / 256 / 3600) * deg_to_rad, 0.05 / 32768},
      {26, 200, 1e-9, 2e-8},
      {31, 200, (720.0 / 2147483648) * deg_to_rad, 200.0 / 2147483648},
      {32, 125, deg_to_rad / 2097152, 1.0 / 4194304},
      {34, 200, 1e-9, 2e-8},
      {38, 400, 1e-9, 2e-8},
      {39, 400, 1e-9, 2e-8},
      {41, 125, (0.008 / 65536) * deg_to_rad / 125,
       (0.2 / 65536) * (9.80665 / 1000) / 125},
  };
  for (const auto& model : models) {
    SCOPED_TRACE(static_cast<int>(model.id));
    NovatelParser parser;
    auto payload = RawPayload();
    payload[1] = static_cast<char>(model.id);
    const auto messages = Parse(&parser, Frame(novatel::RAWIMUSX, payload));
    ASSERT_EQ(1u, messages.size());
    const auto imu = Message<Imu>(messages[0]);
    EXPECT_NEAR(1 / model.rate, imu->measurement_span(), 1e-9);
    EXPECT_NEAR(2500000 * model.accel_increment * model.rate,
                imu->linear_acceleration().z(), 1e-10);
    EXPECT_NEAR(1500000 * model.gyro_increment * model.rate,
                imu->angular_velocity().z(), 1e-10);
  }
}

TEST(NovatelParserTest, RejectsTypeConflictsFaultsAndUnsupportedMapping) {
  config::Config config;
  config.set_imu_type(config::ADIS16488);
  NovatelParser conflict(config);
  EXPECT_TRUE(Parse(&conflict, Frame(novatel::RAWIMUSX, RawPayload())).empty());
  config.clear_imu_type();
  config.mutable_novatel_config()->set_imu_orientation(1);
  NovatelParser mapping(config);
  EXPECT_TRUE(Parse(&mapping, Frame(novatel::RAWIMUSX, RawPayload())).empty());
  NovatelParser parser;
  auto fault = RawPayload();
  fault[0] = 1;
  EXPECT_TRUE(Parse(&parser, Frame(novatel::RAWIMUSX, fault)).empty());
  auto unknown = RawPayload();
  unknown[1] = 61;  // OEM7 G370, not the configuration enum's EPSON_G354.
  EXPECT_TRUE(Parse(&parser, Frame(novatel::RAWIMUSX, unknown)).empty());
  ASSERT_EQ(1u, Parse(&parser, Frame(novatel::RAWIMUSX, RawPayload())).size());
  EXPECT_TRUE(Parse(&parser, Frame(novatel::RAWIMUSX, RawPayload())).empty());
  EXPECT_TRUE(
      Parse(&parser, Frame(novatel::RAWIMUSX, RawPayload(99.0))).empty());
  EXPECT_EQ(
      1u, Parse(&parser, Frame(novatel::RAWIMUSX, RawPayload(100.005))).size());
}

TEST(NovatelParserTest, LegacyRawImuRequiresExplicitTypeAndDoesNotMutateIns) {
  auto payload = RawPayload();
  Put<uint32_t>(&payload, 0, 2200);
  NovatelParser unconfigured;
  EXPECT_TRUE(Parse(&unconfigured, Frame(novatel::RAWIMUS, payload)).empty());
  config::Config config;
  config.set_imu_type(config::ISA100C);
  NovatelParser parser(config);
  const auto raw = Parse(&parser, Frame(novatel::RAWIMU, payload, false));
  ASSERT_EQ(1u, raw.size());
  EXPECT_DOUBLE_EQ(10.0, Message<Imu>(raw[0])->linear_acceleration().z());
  const auto pva = Parse(&parser, Frame(novatel::INSPVAS, PvaPayload()));
  ASSERT_EQ(1u, pva.size());
  EXPECT_FALSE(Message<Ins>(pva[0])->has_linear_acceleration());
}

TEST(NovatelParserTest, MissingSamplesKeepNativeSpanAndIntMinIsSafe) {
  NovatelParser parser;
  auto payload = RawPayload();
  Put<int32_t>(&payload, 20, std::numeric_limits<int32_t>::min());
  Put<int32_t>(&payload, 32, std::numeric_limits<int32_t>::min());
  const auto first = Parse(&parser, Frame(novatel::RAWIMUX, payload, false));
  ASSERT_EQ(1u, first.size());
  const auto imu = Message<Imu>(first[0]);
  EXPECT_NEAR(8589.934592, imu->linear_acceleration().x(), 1e-8);
  EXPECT_NEAR(429.4967296, imu->angular_velocity().x(), 1e-9);
  const auto later =
      Parse(&parser, Frame(novatel::RAWIMUSX, RawPayload(100.015)));
  ASSERT_EQ(1u, later.size());
  EXPECT_NEAR(0.005, Message<Imu>(later[0])->measurement_span(), 1e-9);
  EXPECT_DOUBLE_EQ(10.0, Message<Imu>(later[0])->linear_acceleration().z());
}

TEST(NovatelParserTest, GpsWeekRolloverAndLargeWeeksDoNotOverflow) {
  NovatelParser parser;
  EXPECT_EQ(
      1u,
      Parse(&parser, Frame(novatel::RAWIMUSX, RawPayload(604799.995))).size());
  auto rollover = RawPayload(0.0);
  Put<uint16_t>(&rollover, 2, 2201);
  const auto next = Parse(&parser, Frame(novatel::RAWIMUSX, rollover));
  ASSERT_EQ(1u, next.size());
  EXPECT_DOUBLE_EQ(2201.0 * 604800, Message<Imu>(next[0])->measurement_time());
  Put<uint16_t>(&rollover, 2, 5000);
  const auto future = Parse(&parser, Frame(novatel::RAWIMUSX, rollover));
  ASSERT_EQ(1u, future.size());
  EXPECT_DOUBLE_EQ(5000.0 * 604800,
                   Message<Imu>(future[0])->measurement_time());
}

TEST(NovatelParserTest, CorrectedRateIsIndependentOfRawAndNeverPublishesPva) {
  config::Config config;
  config.set_imu_type(config::ISA100C);
  config.mutable_novatel_config()->set_corrimu_rate_hz(10.0);
  NovatelParser parser(config);
  const auto corrected =
      Parse(&parser, Frame(novatel::CORRIMUDATAS, CorrPayload()));
  ASSERT_EQ(1u, corrected.size());
  EXPECT_EQ(Parser::MessageType::CORRECTED_IMU, corrected[0].first);
  const auto ins = Message<Ins>(corrected[0]);
  EXPECT_FALSE(ins->has_position());
  EXPECT_NEAR(0.5, ins->linear_acceleration().x(), 1e-12);
  EXPECT_NEAR(-0.4, ins->linear_acceleration().y(), 1e-12);
  const auto pva = Parse(&parser, Frame(novatel::INSPVAS, PvaPayload()));
  ASSERT_EQ(1u, pva.size());
  EXPECT_EQ(Parser::MessageType::INS, pva[0].first);
  EXPECT_FALSE(Message<Ins>(pva[0])->has_angular_velocity());
  const auto native =
      Parse(&parser, Frame(novatel::IMURATECORRIMUS, CorrPayload(100.005)));
  ASSERT_EQ(1u, native.size());
  EXPECT_NEAR(10.0, Message<Ins>(native[0])->linear_acceleration().x(), 1e-12);
  NovatelParser no_rate;
  EXPECT_TRUE(
      Parse(&no_rate, Frame(novatel::CORRIMUDATAS, CorrPayload())).empty());
  EXPECT_TRUE(
      Parse(&no_rate, Frame(novatel::IMURATECORRIMUS, CorrPayload())).empty());
}

TEST(NovatelParserTest, ResynchronizesAfterBadLengthCrcAndHeader) {
  const auto good = Frame(novatel::RAWIMUSX, RawPayload());
  auto bad_crc = good;
  bad_crc.back() ^= 1;
  auto bad_length = Frame(novatel::RAWIMUX, RawPayload(), false);
  Put<uint16_t>(&bad_length, 8, 65535);
  auto bad_header = Frame(novatel::RAWIMUX, RawPayload(), false);
  bad_header[3] = 0;
  for (const auto& corrupt : {bad_crc, bad_length, bad_header}) {
    NovatelParser parser;
    const auto parsed = Parse(&parser, std::string("garbage") + corrupt + good);
    ASSERT_EQ(1u, parsed.size());
    EXPECT_EQ(Parser::MessageType::IMU, parsed[0].first);
  }
  // The valid header begins inside the corrupt candidate header.
  NovatelParser overlapping;
  auto prefix = good.substr(0, 12);
  EXPECT_EQ(1u, Parse(&overlapping, prefix + good).size());
  NovatelParser partial;
  EXPECT_TRUE(Parse(&partial, "garbage!!" + good.substr(0, 3)).empty());
  EXPECT_EQ(1u, Parse(&partial, good.substr(3)).size());
}

TEST(NovatelParserTest, EphemerisSnapshotsDoNotRetainOtherConstellations) {
  NovatelParser parser;
  std::string gps(224, '\0');
  Put<uint32_t>(&gps, 0, 3);
  Put<uint32_t>(&gps, 24, 2200);
  Put(&gps, 40, 26560000.0);  // semi-major axis [m]
  const auto gps_messages =
      Parse(&parser, Frame(novatel::GPSEPHEMERIS, gps, false));
  ASSERT_EQ(1u, gps_messages.size());
  EXPECT_TRUE(Message<GnssEphemeris>(gps_messages[0])->has_keppler_orbit());
  std::string glo(144, '\0');
  Put<uint16_t>(&glo, 0, 40);
  Put<uint16_t>(&glo, 2, 5);
  Put<uint16_t>(&glo, 6, 2200);
  Put<uint32_t>(&glo, 8, 100123);
  const auto glo_messages =
      Parse(&parser, Frame(novatel::GLOEPHEMERIS, glo, false));
  ASSERT_EQ(1u, glo_messages.size());
  const auto eph = Message<GnssEphemeris>(glo_messages[0]);
  EXPECT_FALSE(eph->has_keppler_orbit());
  EXPECT_EQ(-2, eph->glonass_orbit().frequency_no());
  EXPECT_NEAR(100.123, eph->glonass_orbit().week_second_s(), 1e-12);
  EXPECT_NEAR(100.123, eph->glonass_orbit().toe(), 1e-12);
  const auto again = Parse(&parser, Frame(novatel::GPSEPHEMERIS, gps, false));
  ASSERT_EQ(1u, again.size());
  EXPECT_FALSE(Message<GnssEphemeris>(again[0])->has_glonass_orbit());
}

TEST(NovatelParserTest, RejectsInvalidTimeNonfinitePvaAndResponse) {
  auto bad_time = RawPayload();
  Put(&bad_time, 4, std::numeric_limits<double>::quiet_NaN());
  NovatelParser parser;
  EXPECT_TRUE(Parse(&parser, Frame(novatel::RAWIMUSX, bad_time)).empty());
  auto bad_pva = PvaPayload();
  Put(&bad_pva, 76, std::numeric_limits<double>::infinity());
  EXPECT_TRUE(Parse(&parser, Frame(novatel::INSPVAS, bad_pva)).empty());
  auto response = Frame(novatel::RAWIMUX, RawPayload(), false);
  response[6] = '\x80';
  response.resize(response.size() - 4);
  const auto crc = WireCrc(response);
  response.append(reinterpret_cast<const char*>(&crc), 4);
  EXPECT_TRUE(Parse(&parser, response).empty());
  EXPECT_EQ(1u, Parse(&parser, Frame(novatel::RAWIMUSX, RawPayload())).size());
}

TEST(NovatelParserTest, FixedStationIdsAndGnssStatusHaveNoStaleFields) {
  std::string pos(72, '\0');
  Put<uint32_t>(&pos, 4, 50);
  Put(&pos, 8, 37.0);
  Put(&pos, 16, -122.0);
  Put(&pos, 24, 20.0);
  Put<float>(&pos, 32, 5.0f);
  Put<uint32_t>(&pos, 36, 61);
  pos.replace(52, 4, "ABCD");
  Put<float>(&pos, 56, 1.0f);
  pos[64] = 12;
  pos[65] = 10;
  NovatelParser parser;
  const auto best = Parse(&parser, Frame(novatel::BESTGNSSPOS, pos, false));
  ASSERT_EQ(2u, best.size());
  EXPECT_EQ("ABCD", Message<GnssBestPose>(best[0])->base_station_id());
  EXPECT_DOUBLE_EQ(25.0, Message<Gnss>(best[1])->position().height());
  EXPECT_FALSE(Message<Gnss>(best[1])->has_linear_velocity());
  auto failed = pos;
  Put<uint32_t>(&failed, 0, 1);  // INSUFFICIENT_OBS
  Put(&failed, 8, std::numeric_limits<double>::quiet_NaN());
  const auto status =
      Parse(&parser, Frame(novatel::BESTGNSSPOS, failed, false));
  ASSERT_EQ(1u, status.size());
  EXPECT_EQ(Parser::MessageType::GNSS, status[0].first);
  EXPECT_EQ(1u, Message<Gnss>(status[0])->solution_status());
  EXPECT_FALSE(Message<Gnss>(status[0])->has_position());
  EXPECT_EQ(Gnss::INVALID, Message<Gnss>(status[0])->type());
  auto bad_uncertainty = pos;
  Put<float>(&bad_uncertainty, 40, std::numeric_limits<float>::quiet_NaN());
  const auto invalid =
      Parse(&parser, Frame(novatel::BESTGNSSPOS, bad_uncertainty, false));
  ASSERT_EQ(1u, invalid.size());
  EXPECT_FALSE(Message<Gnss>(invalid[0])->has_position());
  EXPECT_EQ(Gnss::INVALID, Message<Gnss>(invalid[0])->type());
  std::string vel(44, '\0');
  Put<uint32_t>(&vel, 4, 8);
  Put<float>(&vel, 8, 0.25f);
  Put(&vel, 16, 10.0);
  Put(&vel, 24, 90.0);
  const auto velocity = Parse(&parser, Frame(novatel::BESTGNSSVEL, vel, false));
  ASSERT_EQ(1u, velocity.size());
  const auto gnss = Message<Gnss>(velocity[0]);
  EXPECT_FALSE(gnss->has_position());
  EXPECT_FALSE(gnss->has_solution_status());
  EXPECT_NEAR(10.0, gnss->linear_velocity().x(), 1e-12);
  EXPECT_FLOAT_EQ(0.25f, gnss->velocity_latency());
  std::string heading(44, '\0');
  heading.replace(32, 4, "WXYZ");
  heading[36] = 12;
  const auto headings = Parse(&parser, Frame(novatel::HEADING, heading, false));
  ASSERT_EQ(1u, headings.size());
  EXPECT_EQ("WXYZ", Message<Heading>(headings[0])->station_id());
}

TEST(NovatelParserTest, RejectsInvalidHeadingAndUnsafeEphemerisConversions) {
  NovatelParser parser;
  std::string heading(44, '\0');
  Put<float>(&heading, 12, std::numeric_limits<float>::quiet_NaN());
  EXPECT_TRUE(Parse(&parser, Frame(novatel::HEADING, heading, false)).empty());
  Put<float>(&heading, 12, 90.0f);
  Put<float>(&heading, 24, -1.0f);
  EXPECT_TRUE(Parse(&parser, Frame(novatel::HEADING, heading, false)).empty());
  Put<float>(&heading, 24, 0.5f);
  ASSERT_EQ(1u, Parse(&parser, Frame(novatel::HEADING, heading, false)).size());
  std::string gps(224, '\0');
  Put<uint32_t>(&gps, 24, 2200);
  Put(&gps, 40, -1.0);  // sqrt(A) would be invalid.
  EXPECT_TRUE(Parse(&parser, Frame(novatel::GPSEPHEMERIS, gps, false)).empty());
  Put(&gps, 40, 26560000.0);
  Put(&gps, 216, std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(Parse(&parser, Frame(novatel::GPSEPHEMERIS, gps, false)).empty());
  Put(&gps, 216, 4.0);
  const auto valid = Parse(&parser, Frame(novatel::GPSEPHEMERIS, gps, false));
  ASSERT_EQ(1u, valid.size());
  EXPECT_EQ(2u, Message<GnssEphemeris>(valid[0])->keppler_orbit().accuracy());
  Put(&gps, 64, std::numeric_limits<double>::quiet_NaN());  // eccentricity
  EXPECT_TRUE(Parse(&parser, Frame(novatel::GPSEPHEMERIS, gps, false)).empty());
  std::string bds(196, '\0');
  Put(&bds, 76, 5153.795);  // sqrt(A), after AODE/TOE (RTKLIB decoder).
  Put(&bds, 8, 4.0);
  ASSERT_EQ(1u,
            Parse(&parser, Frame(novatel::BDSEPHEMERIS, bds, false)).size());
  Put(&bds, 8, std::numeric_limits<double>::infinity());
  EXPECT_TRUE(Parse(&parser, Frame(novatel::BDSEPHEMERIS, bds, false)).empty());
  Put(&bds, 8, 4294967296.0);
  EXPECT_TRUE(Parse(&parser, Frame(novatel::BDSEPHEMERIS, bds, false)).empty());
}

TEST(NovatelParserTest,
     GlonassTimesMatchGpsReferenceAcrossDayAndWeekBoundaries) {
  NovatelParser parser;
  std::string glo(144, '\0');
  Put<uint16_t>(&glo, 0, 40);
  Put<uint16_t>(&glo, 2, 5);
  Put<uint16_t>(&glo, 6, 2200);
  Put<uint32_t>(&glo, 12, 10782);  // GLONASS = GPS + 3 h - 18 leap seconds.
  struct Epoch {
    uint32_t milliseconds;
    uint32_t frame_glonass;
    double frame_gps;
  };
  for (const Epoch epoch :
       {Epoch{100123, 10882, 100}, Epoch{1500, 10780, 604798},
        Epoch{86399500, 10783, 86401}, Epoch{86401000, 10781, 86399}}) {
    Put(&glo, 8, epoch.milliseconds);
    Put(&glo, 124, epoch.frame_glonass);
    const auto messages =
        Parse(&parser, Frame(novatel::GLOEPHEMERIS, glo, false));
    ASSERT_EQ(1u, messages.size());
    const auto& orbit = Message<GnssEphemeris>(messages[0])->glonass_orbit();
    EXPECT_EQ(GPS_TIME, orbit.gnss_time_type());
    EXPECT_NEAR(epoch.milliseconds * 0.001, orbit.toe(), 1e-9);
    EXPECT_DOUBLE_EQ(epoch.frame_gps, orbit.tk());
  }
  Put<uint16_t>(&glo, 0, 37);  // Invalid slot must not underflow the PRN.
  EXPECT_TRUE(Parse(&parser, Frame(novatel::GLOEPHEMERIS, glo, false)).empty());
  Put<uint16_t>(&glo, 0, 40);
  Put(&glo, 28, std::numeric_limits<double>::infinity());
  EXPECT_TRUE(Parse(&parser, Frame(novatel::GLOEPHEMERIS, glo, false)).empty());
}

TEST(NovatelParserTest,
     RangeFeedsCompleteFrameAndPreservesCodeOnlyObservations) {
  std::string payload(4 + 44, '\0');
  Put<uint32_t>(&payload, 0, 1);
  Put<uint16_t>(&payload, 4, 3);  // GPS PRN
  Put(&payload, 8, 21000000.0);
  Put<float>(&payload, 36, 45.0f);  // C/N0
  Put<float>(&payload, 40, 100.0f);
  // GPS L1 C/A: parity known, code locked, phase unlocked.
  Put<uint32_t>(&payload, 44, (1u << 11) | (1u << 12));
  NovatelParser parser;
  const auto messages = Parse(&parser, Frame(novatel::RANGE, payload, false));
  ASSERT_EQ(1u, messages.size());
  const auto obs = Message<EpochObservation>(messages[0]);
  ASSERT_NE(nullptr, obs);
  EXPECT_EQ(2200u, obs->gnss_week());
  EXPECT_DOUBLE_EQ(100.0, obs->gnss_second_s());
  ASSERT_EQ(1, obs->sat_obs_size());
  ASSERT_EQ(1, obs->sat_obs(0).band_obs_size());
  const auto& band = obs->sat_obs(0).band_obs(0);
  EXPECT_EQ(3u, obs->sat_obs(0).sat_prn());
  EXPECT_EQ(CORSE_CODE, band.pseudo_type());
  EXPECT_DOUBLE_EQ(21000000.0, band.pseudo_range());
  EXPECT_DOUBLE_EQ(0.0, band.carrier_phase());
  EXPECT_FLOAT_EQ(45.0f, band.snr());
  EXPECT_EQ(0u, band.loss_lock_index());
  // Another frame at the same epoch is a snapshot, not an RTKLIB merge.
  Put<uint16_t>(&payload, 4, 4);
  const auto next = Parse(&parser, Frame(novatel::RANGE, payload, false));
  ASSERT_EQ(1u, next.size());
  const auto next_obs = Message<EpochObservation>(next[0]);
  ASSERT_EQ(1, next_obs->sat_obs_size());
  EXPECT_EQ(1u, next_obs->sat_obs_num());
  EXPECT_EQ(4u, next_obs->sat_obs(0).sat_prn());
  // An empty frame must not repeat the preceding epoch's satellites.
  const auto empty =
      Parse(&parser, Frame(novatel::RANGE, std::string(4, '\0'), false));
  ASSERT_EQ(1u, empty.size());
  EXPECT_EQ(0, Message<EpochObservation>(empty[0])->sat_obs_size());
  EXPECT_EQ(0u, Message<EpochObservation>(empty[0])->sat_obs_num());
  EXPECT_EQ(3u, obs->sat_obs(0).sat_prn());  // Old snapshots stay immutable.
  Put<uint32_t>(&payload, 0, UINT32_MAX);
  EXPECT_TRUE(Parse(&parser, Frame(novatel::RANGE, payload, false)).empty());
}

TEST(NovatelParserTest, CovarianceIsEpochBoundAndAttitudeCrossTermsChangeSign) {
  config::Config config;
  NovatelParser parser(config);
  std::string cov(228, '\0');
  Put<uint32_t>(&cov, 0, 2200);
  Put(&cov, 4, 100.0);
  const double matrix[] = {1, 0.1, 0.2, 0.1, 4, 0.3, 0.2, 0.3, 9};
  for (int i = 0; i < 9; ++i) {
    Put(&cov, 12 + i * 8, matrix[i]);
    Put(&cov, 84 + i * 8, matrix[i]);
    Put(&cov, 156 + i * 8, matrix[i]);
  }
  EXPECT_TRUE(Parse(&parser, Frame(novatel::INSCOVS, cov)).empty());
  const auto matched = Parse(&parser, Frame(novatel::INSPVAS, PvaPayload()));
  ASSERT_EQ(1u, matched.size());
  const auto ins = Message<Ins>(matched[0]);
  ASSERT_EQ(9, ins->euler_angles_covariance_size());
  constexpr double square_deg_to_rad = 0.00030461741978670857;
  EXPECT_NEAR(4 * square_deg_to_rad, ins->euler_angles_covariance(0), 1e-9);
  EXPECT_NEAR(-0.1 * square_deg_to_rad, ins->euler_angles_covariance(1), 1e-9);
  EXPECT_NEAR(-0.3 * square_deg_to_rad, ins->euler_angles_covariance(2), 1e-9);
  EXPECT_NEAR(0.2 * square_deg_to_rad, ins->euler_angles_covariance(5), 1e-9);
  const auto stale = Parse(&parser, Frame(novatel::INSPVAS, PvaPayload(101.0)));
  ASSERT_EQ(1u, stale.size());
  EXPECT_EQ(0, Message<Ins>(stale[0])->euler_angles_covariance_size());
  Put(&cov, 4, 102.0);
  Put(&cov, 12, -1.0);
  EXPECT_TRUE(Parse(&parser, Frame(novatel::INSCOVS, cov)).empty());
  const auto negative =
      Parse(&parser, Frame(novatel::INSPVAS, PvaPayload(102.0)));
  ASSERT_EQ(1u, negative.size());
  EXPECT_EQ(0, Message<Ins>(negative[0])->position_covariance_size());
  Put(&cov, 12, 1e100);  // finite double, but not representable in float proto
  EXPECT_TRUE(Parse(&parser, Frame(novatel::INSCOVS, cov)).empty());
  const auto overflow =
      Parse(&parser, Frame(novatel::INSPVAS, PvaPayload(102.0)));
  ASSERT_EQ(1u, overflow.size());
  EXPECT_EQ(0, Message<Ins>(overflow[0])->position_covariance_size());
}

TEST(NovatelParserTest, RealCaptureMatchesRecordedRatesAndPhysicalScale) {
  const char* runfiles = std::getenv("TEST_SRCDIR");
  const std::string path =
      std::string(runfiles ? std::string(runfiles) + "/_main/" : "") +
      "modules/drivers/gnss/test_data/novatel.bin";
  std::ifstream input(path, std::ios::binary);
  ASSERT_TRUE(input.is_open()) << path;
  const std::string bytes((std::istreambuf_iterator<char>(input)), {});
  ASSERT_EQ(1070568u, bytes.size());
  config::Config config;
  config.set_imu_type(config::ISA100C);
  config.mutable_novatel_config()->set_corrimu_rate_hz(10);
  NovatelParser parser(config);
  std::map<Parser::MessageType, size_t> counts;
  std::map<Parser::MessageType, double> previous;
  const auto check_period = [&previous](Parser::MessageType type, double time,
                                        double period) {
    const auto found = previous.find(type);
    if (found != previous.end()) {
      EXPECT_NEAR(period, time - found->second, 2e-6);
    }
    previous[type] = time;
  };
  double previous_raw = -1;
  double acceleration_sum = 0;
  for (size_t offset = 0; offset < bytes.size(); offset += 127) {
    for (const auto& message : Parse(&parser, bytes.substr(offset, 127))) {
      ++counts[message.first];
      if (message.first == Parser::MessageType::IMU) {
        const auto imu = Message<Imu>(message);
        const auto& acc = imu->linear_acceleration();
        acceleration_sum += std::sqrt(acc.x() * acc.x() + acc.y() * acc.y() +
                                      acc.z() * acc.z());
        if (previous_raw > 0) {
          EXPECT_NEAR(0.005, imu->measurement_time() - previous_raw, 2e-6);
        }
        previous_raw = imu->measurement_time();
      } else if (message.first == Parser::MessageType::INS) {
        EXPECT_FALSE(Message<Ins>(message)->has_linear_acceleration());
        check_period(message.first, Message<Ins>(message)->measurement_time(),
                     0.1);
      } else if (message.first == Parser::MessageType::CORRECTED_IMU) {
        EXPECT_FALSE(Message<Ins>(message)->has_position());
        check_period(message.first, Message<Ins>(message)->measurement_time(),
                     0.1);
      } else if (message.first == Parser::MessageType::BEST_GNSS_POS) {
        check_period(message.first,
                     Message<GnssBestPose>(message)->measurement_time(), 1.0);
      }
    }
  }
  EXPECT_EQ(15953u, counts[Parser::MessageType::IMU]);
  EXPECT_EQ(796u, counts[Parser::MessageType::INS]);
  EXPECT_EQ(796u, counts[Parser::MessageType::CORRECTED_IMU]);
  EXPECT_EQ(80u, counts[Parser::MessageType::BEST_GNSS_POS]);
  EXPECT_EQ(160u, counts[Parser::MessageType::GNSS]);
  EXPECT_NEAR(9.80665, acceleration_sum / counts[Parser::MessageType::IMU],
              0.01);
}

}  // namespace
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
