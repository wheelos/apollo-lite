/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/drivers/gnss/proto/config.pb.h"

#include "cyber/common/log.h"
#include "modules/drivers/gnss/parser/huace/huace_parser.h"
#include "modules/drivers/gnss/parser/novatel/novatel_parser.h"
#include "modules/drivers/gnss/parser/parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

class ParserFactory {
 public:
  static std::unique_ptr<Parser> Create(const config::Config &config) {
    static const std::unordered_map<
        config::Stream::Format,
        std::function<std::unique_ptr<Parser>(const config::Config &)>>
        parser_factories = {
            {config::Stream::NOVATEL_BINARY,
             [](const config::Config &cfg) {
               return std::make_unique<NovatelParser>(cfg);
             }},
            {config::Stream::HUACE_TEXT,
             [](const config::Config &cfg) {
               return std::make_unique<HuaceParser>(cfg);
             }},
            // Add more parser types here by mapping config::Stream to a lambda
            // that creates the corresponding derived parser instance.
        };

    auto factory_it = parser_factories.find(config.data().format());
    if (factory_it != parser_factories.end()) {
      AINFO << "Creating parser for format: "
            << config::Stream::Format_Name(config.data().format());
      return factory_it->second(config);
    }

    AERROR << "Unsupported data format: "
           << static_cast<int>(config.data().format());
    return nullptr;
  }
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
