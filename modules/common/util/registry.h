// Copyright 2025 WheelOS. All Rights Reserved.
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

//  Created Date: 2026-03-05
//  Author: daohu527

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace apollo {
namespace common {
namespace util {

template <typename Key, typename BaseClass>
class FactoryRegistry {
 public:
  using Creator = std::function<std::unique_ptr<BaseClass>()>;

  static FactoryRegistry& Instance() {
    static FactoryRegistry instance;
    return instance;
  }

  bool Register(const Key& key, Creator creator) {
    if (!creator) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);

    auto result = factory_map_.emplace(key, std::move(creator));
    return result.second;
  }

  std::unique_ptr<BaseClass> Create(const Key& key) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = factory_map_.find(key);
    return (it != factory_map_.end()) ? it->second() : nullptr;
  }

  std::vector<Key> GetRegisteredKeys() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Key> keys;
    keys.reserve(factory_map_.size());
    for (const auto& pair : factory_map_) {
      keys.push_back(pair.first);
    }
    return keys;
  }

 private:
  FactoryRegistry() = default;

  FactoryRegistry(const FactoryRegistry&) = delete;
  FactoryRegistry& operator=(const FactoryRegistry&) = delete;

  mutable std::mutex mutex_;
  std::map<Key, Creator> factory_map_;
};

}  // namespace util
}  // namespace common
}  // namespace apollo

#define DECLARE_FACTORY_MANAGER_BY_KEY(base_class, key_type)                 \
  class base_class##Manager {                                                \
   public:                                                                   \
    static std::unique_ptr<base_class> CreateInstance(const key_type& key) { \
      return ::apollo::common::util::FactoryRegistry<key_type,               \
                                                     base_class>::Instance() \
          .Create(key);                                                      \
    }                                                                        \
    static std::vector<key_type> GetList() {                                 \
      return ::apollo::common::util::FactoryRegistry<key_type,               \
                                                     base_class>::Instance() \
          .GetRegisteredKeys();                                              \
    }                                                                        \
  };

#define DECLARE_FACTORY_MANAGER(base_class) \
  DECLARE_FACTORY_MANAGER_BY_KEY(base_class, std::string)

#define REGISTER_PLUGIN_BY_KEY(base_class, derived_class, key_value)   \
  namespace {                                                          \
  __attribute__((constructor)) void Register##derived_class() {        \
    ::apollo::common::util::FactoryRegistry<decltype(key_value),       \
                                            base_class>::Instance()    \
        .Register(key_value,                                           \
                  []() { return std::make_unique<derived_class>(); }); \
  }                                                                    \
  }

#define REGISTER_PLUGIN(base_class, derived_class) \
  REGISTER_PLUGIN_BY_KEY(base_class, derived_class, std::string(#derived_class))
