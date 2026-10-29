// Copyright 2026 WheelOS. All Rights Reserved.
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

#include "modules/drivers/gnss/parser/parser_factory.h"

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace gnss {
namespace {

TEST(ParserFactoryTest, OnlyCompiledBrandsCanBeConstructed) {
  config::Config config;
  config.mutable_data()->set_format(config::Stream::NOVATEL_BINARY);
#ifdef GNSS_WITH_NOVATEL
  EXPECT_NE(ParserFactory::Create(config), nullptr);
#else
  EXPECT_EQ(ParserFactory::Create(config), nullptr);
#endif
  config.mutable_data()->set_format(config::Stream::HUACE_TEXT);
#ifdef GNSS_WITH_HUACE
  EXPECT_NE(ParserFactory::Create(config), nullptr);
#else
  EXPECT_EQ(ParserFactory::Create(config), nullptr);
#endif
  config.mutable_data()->set_format(config::Stream::FORSENSE_TEXT);
#ifdef GNSS_WITH_FORSENSE
  EXPECT_NE(ParserFactory::Create(config), nullptr);
  config.mutable_data()->set_format(config::Stream::FORSENSE_BINARY);
  EXPECT_NE(ParserFactory::Create(config), nullptr);
#else
  EXPECT_EQ(ParserFactory::Create(config), nullptr);
  config.mutable_data()->set_format(config::Stream::FORSENSE_BINARY);
  EXPECT_EQ(ParserFactory::Create(config), nullptr);
#endif
}

TEST(ParserFactoryTest, UnsupportedFormatNeverFallsBack) {
  config::Config config;
  config.mutable_data()->set_format(config::Stream::RTCM_V3);
  EXPECT_EQ(ParserFactory::Create(config), nullptr);
}

}  // namespace
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
