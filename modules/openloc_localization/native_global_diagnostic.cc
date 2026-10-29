// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <fstream>
#include <iostream>
#include <thread>

#include "google/protobuf/text_format.h"
#include "native_global_cyber.h"

namespace host = apollo::localization::openloc_diagnostic;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: native_global_diagnostic CONFIG.pb.txt\n";
    return 1;
  }
  bool initialized = false;
  try {
    std::ifstream file(argv[1]);
    const std::string text((std::istreambuf_iterator<char>(file)), {});
    host::NativeGlobalHostConfig config;
    if (!file || !google::protobuf::TextFormat::ParseFromString(text, &config))
      throw std::runtime_error("unreadable global diagnostic profile");
    if (!apollo::cyber::Init(argv[0]))
      throw std::runtime_error("global host Cyber initialization failed");
    initialized = true;
    {
      std::shared_ptr<apollo::cyber::Node> node =
          apollo::cyber::CreateNode("openloc_native_global_diagnostic:" +
                                    config.output_topic());
      host::NativeGlobalCyber global(node, config);
      while (apollo::cyber::OK()) {
        global.Check();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      global.Finish();
    }
    apollo::cyber::Clear();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "OpenLoc global diagnostic failed: " << error.what() << '\n';
    if (initialized) apollo::cyber::Clear();
    return 1;
  }
}
