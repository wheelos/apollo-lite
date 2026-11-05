// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.

#include <iostream>
#include <memory>
#include <string>

#include "fixtures/runtime_fixture.h"

#include "wheelos_msgs/chassis_msgs/chassis.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

class ProductBeta : public apollo::cyber::Component<apollo::canbus::Chassis> {
 public:
  bool Init() override {
    return ReadFixture(ConfigFilePath(), &configuration_);
  }

  bool Proc(const std::shared_ptr<apollo::canbus::Chassis>& message) override {
    if (message->has_speed_mps() && message->speed_mps() == 0.0F) {
      std::cout << "PRODUCT_BETA " << configuration_ << std::endl;
    }
    return true;
  }

 private:
  std::string configuration_;
};

CYBER_REGISTER_COMPONENT(ProductBeta)
