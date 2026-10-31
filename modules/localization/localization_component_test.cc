// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "modules/localization/localization_component.h"

#include <memory>

#include "gtest/gtest.h"

#include "cyber/cyber.h"

namespace apollo::localization::openloc {
namespace {

TEST(LocalizationComponent, TemplateFailsInitializationBeforeSensorReaders) {
  ASSERT_TRUE(cyber::Init("openloc_component_template_test"));
  {
    auto component = std::make_shared<OpenLocLocalizationComponent>();
    cyber::proto::TimerComponentConfig config;
    config.set_name("openloc_component_template_test");
    config.set_interval(10);
    config.set_config_file_path(
        "modules/localization/conf/localization.pb.txt");
    EXPECT_FALSE(component->Initialize(config));
    component->Clear();
  }
  cyber::Clear();
}

}  // namespace
}  // namespace apollo::localization::openloc
