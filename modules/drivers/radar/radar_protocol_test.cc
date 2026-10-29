// Copyright 2026 The WheelOS Authors. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <cstdint>

#include "gtest/gtest.h"

#include "modules/drivers/radar/conti_radar/protocol/motion_input_speed_300.h"
#include "modules/drivers/radar/conti_radar/protocol/object_general_info_60b.h"
#include "modules/drivers/radar/nano_radar/protocol/object_general_info_60b.h"
#include "modules/drivers/radar/racobit_radar/protocol/object_general_info_60b.h"
#include "modules/drivers/radar/udas_ultrasonic_radar/protocol/sensor_dist_f_5c9.h"
#include "modules/drivers/radar/udas_ultrasonic_radar/protocol/sensor_dist_r_5c8.h"
#include "modules/drivers/radar/yg_radar/protocol/radar_object_info_540.h"

namespace apollo {
namespace drivers {
namespace {

template <typename Protocol, typename Message>
void CheckObjectDecoding(const char* vendor) {
  SCOPED_TRACE(vendor);
  const std::array<uint8_t, 8> bytes{7,    0x4F, 0xB3, 0xE8,
                                     0x7E, 0x21, 0x82, 0x80};
  Message message;
  message.mutable_header()->set_frame_id("radar");
  Protocol().Parse(bytes.data(), bytes.size(), &message);
  ASSERT_EQ(message.contiobs_size(), 1);
  const auto& object = message.contiobs(0);
  EXPECT_EQ(object.obstacle_id(), 7);
  EXPECT_FALSE(object.clusterortrack());
  EXPECT_NEAR(object.longitude_dist(), 10.0, 1e-9);
  EXPECT_NEAR(object.lateral_dist(), -4.6, 1e-9);
  EXPECT_DOUBLE_EQ(object.longitude_vel(), -2.0);
  EXPECT_DOUBLE_EQ(object.lateral_vel(), 3.0);
  EXPECT_DOUBLE_EQ(object.rcs(), 0.0);
  EXPECT_EQ(object.dynprop(), 2);
  EXPECT_EQ(object.header().frame_id(), "radar");
}

TEST(RadarProtocolTest, DecodesObjectScalingAcrossArsCompatibleRadars) {
  CheckObjectDecoding<conti_radar::ObjectGeneralInfo60B, ContiRadar>("conti");
  CheckObjectDecoding<nano_radar::ObjectGeneralInfo60B, NanoRadar>("nano");
  CheckObjectDecoding<racobit_radar::ObjectGeneralInfo60B, RacobitRadar>(
      "racobit");
}

TEST(RadarProtocolTest, DecodesYgSignedCrossByteFieldsAndLimits) {
  struct Case {
    std::array<uint8_t, 8> bytes;
    double longitudinal;
    double speed;
    double angle;
    double lateral;
    double acceleration;
    int id;
  };
  for (const auto& test : {
           Case{{0x64, 0x60, 0xBF, 0x25, 0xC4, 0xBF, 0x7F, 0x00},
                10.0,
                -2.0,
                15.0,
                -3.0,
                -0.5,
                7},
           Case{{0x00, 0xFC, 0x1F, 0x00, 0xFF, 0x07, 0xF8, 0x7F},
                -102.4,
                102.3,
                -102.4,
                102.3,
                -12.8,
                2047},
       }) {
    SCOPED_TRACE(test.id);
    yg_radar::Yg_radar message;
    yg_radar::Radarobjectinfo540().Parse(test.bytes.data(), test.bytes.size(),
                                         &message);
    const auto& object = message.radar_object_info_540();
    EXPECT_NEAR(object.object_distance_lon(), test.longitudinal, 1e-9);
    EXPECT_NEAR(object.object_speed(), test.speed, 1e-9);
    EXPECT_NEAR(object.object_angle(), test.angle, 1e-9);
    EXPECT_NEAR(object.object_distance_lat(), test.lateral, 1e-9);
    EXPECT_NEAR(object.object_acceleration(), test.acceleration, 1e-9);
    EXPECT_EQ(object.object_tracking_id(), test.id);
  }
}

TEST(RadarProtocolTest, DecodesAllUdasChannelsInCentimeters) {
  const std::array<uint8_t, 8> bytes{0, 1, 2, 127, 254, 255, 0, 0};
  udas_ultrasonic_radar::Udas_ultrasonic_radar message;
  udas_ultrasonic_radar::Sensordistf5c9().Parse(bytes.data(), bytes.size(),
                                                &message);
  udas_ultrasonic_radar::Sensordistr5c8().Parse(bytes.data(), bytes.size(),
                                                &message);
  const auto& front = message.sensor_dist_f_5c9();
  const auto& rear = message.sensor_dist_r_5c8();
  EXPECT_DOUBLE_EQ(front.sensor_1_fa(), 0);
  EXPECT_DOUBLE_EQ(front.sensor_2_fb(), 2);
  EXPECT_DOUBLE_EQ(front.sensor_3_fc(), 4);
  EXPECT_DOUBLE_EQ(front.sensor_4_fd(), 254);
  EXPECT_DOUBLE_EQ(front.sensor_5_fe(), 508);
  EXPECT_DOUBLE_EQ(front.sensor_6_ff(), 510);
  EXPECT_DOUBLE_EQ(rear.sensor_1_ra(), 0);
  EXPECT_DOUBLE_EQ(rear.sensor_2_rb(), 2);
  EXPECT_DOUBLE_EQ(rear.sensor_3_rc(), 4);
  EXPECT_DOUBLE_EQ(rear.sensor_4_rd(), 254);
  EXPECT_DOUBLE_EQ(rear.sensor_5_re(), 508);
  EXPECT_DOUBLE_EQ(rear.sensor_6_rf(), 510);
}

TEST(RadarProtocolTest, EncodesEgoSpeedDirectionAndLeavesUnusedBytesUntouched) {
  struct Case {
    float speed;
    uint8_t high;
    uint8_t low;
  };
  conti_radar::MotionInputSpeed300 protocol;
  for (const auto& test : {Case{0.0F, 0, 0}, Case{0.01F, 0, 0},
                           Case{10.0F, 0x41, 0xF4}, Case{-10.0F, 0x81, 0xF4}}) {
    SCOPED_TRACE(test.speed);
    std::array<uint8_t, 8> bytes;
    bytes.fill(0xA5);
    protocol.SetSpeed(test.speed);
    protocol.UpdateData(bytes.data());
    EXPECT_EQ(bytes[0], test.high);
    EXPECT_EQ(bytes[1], test.low);
    for (size_t i = 2; i < bytes.size(); ++i) {
      EXPECT_EQ(bytes[i], 0xA5);
    }
    protocol.Reset();
    const auto before = bytes;
    protocol.UpdateData(bytes.data());
    EXPECT_EQ(bytes, before);
  }
}

}  // namespace
}  // namespace drivers
}  // namespace apollo
