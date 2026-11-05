// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.

#include <iostream>
#include <memory>
#include <string>

#include "fixtures/runtime_fixture.h"

#include "wheelos_msgs/chassis_msgs/chassis.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"

class ProductAlpha : public apollo::cyber::TimerComponent {
 public:
  bool Init() override {
    if (!ReadFixture(ConfigFilePath(), &configuration_)) {
      return false;
    }
    writer_ = node_->CreateWriter<apollo::canbus::Chassis>(
        "/runtime_product/chassis");
    return writer_ != nullptr;
  }

  bool Proc() override {
    auto message = std::make_shared<apollo::canbus::Chassis>();
    message->set_speed_mps(0.0F);
    if (!writer_->Write(message)) {
      return false;
    }
    std::cout << "PRODUCT_ALPHA " << configuration_ << std::endl;
    return true;
  }

 private:
  std::string configuration_;
  std::shared_ptr<apollo::cyber::Writer<apollo::canbus::Chassis>> writer_;
};

CYBER_REGISTER_COMPONENT(ProductAlpha)
