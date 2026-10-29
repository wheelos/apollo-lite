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

#include <memory>

#include "cyber/common/log.h"
#ifdef GNSS_WITH_FORSENSE
#include "modules/drivers/gnss/parser/forsense/forsense_binary_parser.h"
#include "modules/drivers/gnss/parser/forsense/forsense_nmea_parser.h"
#endif
#ifdef GNSS_WITH_HUACE
#include "modules/drivers/gnss/parser/huace/huace_parser.h"
#endif
#ifdef GNSS_WITH_NOVATEL
#include "modules/drivers/gnss/parser/novatel/novatel_parser.h"
#endif

namespace apollo {
namespace drivers {
namespace gnss {

std::unique_ptr<Parser> ParserFactory::Create(const config::Config& config) {
  switch (config.data().format()) {
#ifdef GNSS_WITH_FORSENSE
    case config::Stream::FORSENSE_TEXT:
      return std::make_unique<ForsenseNmeaParser>(config);
    case config::Stream::FORSENSE_BINARY:
      return std::make_unique<ForsenseBinaryParser>(config);
#endif
#ifdef GNSS_WITH_HUACE
    case config::Stream::HUACE_TEXT:
      return std::make_unique<HuaceParser>(config);
#endif
#ifdef GNSS_WITH_NOVATEL
    case config::Stream::NOVATEL_BINARY:
      return std::make_unique<NovatelParser>(config);
#endif
    default:
      AERROR << "GNSS format is unsupported or not compiled into this build: "
             << config::Stream::Format_Name(config.data().format());
      return nullptr;
  }
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
