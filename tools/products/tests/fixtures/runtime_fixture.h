// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <fstream>
#include <iostream>
#include <string>

#include "cyber/common/resource_manager.h"

inline bool ReadFixture(const std::string& config_path, std::string* config) {
  std::ifstream input(config_path);
  if (!std::getline(input, *config) || config->empty()) {
    std::cerr << "Fixture configuration is missing or empty" << std::endl;
    return false;
  }
  std::string model;
  if (!apollo::cyber::common::ResourceManager::ResolveAssetPath(
          "models/alpha/fixture/model.bin", &model)) {
    return false;
  }
  std::ifstream resource(model);
  std::string contents;
  if (!std::getline(resource, contents) ||
      contents != "fixture-model-not-for-inference") {
    std::cerr << "Fixture asset contents are incorrect" << std::endl;
    return false;
  }
  return true;
}
