// Copyright 2026 The WheelOS Authors.

#include <cstdint>
#include <iostream>

#include "wheelos_msgs/chassis_msgs/chassis.pb.h"
#include "wheelos_msgs/sensor_msgs/sensor_image.pb.h"

#include "cyber/record/record_reader.h"

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: record_probe RECORD CHASSIS_TOPIC IMAGE_TOPIC\n";
    return 2;
  }
  apollo::cyber::record::RecordReader reader(argv[1]);
  if (!reader.IsValid() || reader.GetProtoDesc(argv[2]).empty() ||
      reader.GetProtoDesc(argv[3]).empty()) {
    std::cerr << "invalid record or missing message descriptors\n";
    return 1;
  }
  apollo::cyber::record::RecordMessage message;
  int chassis_count = 0;
  int image_count = 0;
  uint64_t last_chassis_time = 0;
  uint64_t last_image_time = 0;
  while (reader.ReadMessage(&message)) {
    if (message.channel_name == argv[2]) {
      apollo::canbus::Chassis chassis;
      if (!chassis.ParseFromString(message.content) || !chassis.has_header() ||
          chassis.driving_mode() != apollo::canbus::Chassis::COMPLETE_MANUAL) {
        std::cerr << "invalid FakeCAN chassis payload\n";
        return 1;
      }
      ++chassis_count;
      last_chassis_time = message.time;
    } else if (message.channel_name == argv[3]) {
      apollo::drivers::Image image;
      if (!image.ParseFromString(message.content) || !image.has_header() ||
          image.frame_id() != "synthetic" || image.height() != 1 ||
          image.width() != 2 || image.step() != 2 ||
          image.encoding() != "mono8" || image.data() != "AB") {
        std::cerr << "invalid synthetic image payload\n";
        return 1;
      }
      ++image_count;
      last_image_time = message.time;
    } else {
      std::cerr << "unexpected recorded topic\n";
      return 1;
    }
  }
  std::cout << "{\"chassis\":" << chassis_count << ",\"images\":" << image_count
            << ",\"last_chassis_time\":" << last_chassis_time
            << ",\"last_image_time\":" << last_image_time << "}\n";
  return chassis_count >= 5 && image_count >= 5 ? 0 : 1;
}
