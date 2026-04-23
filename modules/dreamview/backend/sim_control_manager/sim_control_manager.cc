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

//  Created Date: 2026-04-18
//  Author: daohu527

#include "modules/dreamview/backend/sim_control_manager/sim_control_manager.h"

#include "nlohmann/json.hpp"

namespace apollo {
namespace dreamview {

using Json = nlohmann::json;

std::string SimControlManager::Name() const {
  return FLAGS_sim_control_module_name;
}

Json SimControlManager::LoadDynamicModels() {
  auto *model_factory = DynamicModelFactory::Instance();
  return model_factory->RegisterDynamicModels();
}

void SimControlManager::Reset() {
  if (IsEnabled() && model_ptr_) {
    model_ptr_->Reset();
  }
}

void SimControlManager::ResetDynamicModel() {
  if (!current_dynamic_model_.empty()) {
    if (!model_ptr_) {
      auto *model_factory = DynamicModelFactory::Instance();
      model_ptr_ = model_factory->GetModelType(current_dynamic_model_);
    }
    if (model_ptr_) {
      model_ptr_->Stop();
    }
  }
  return;
}

bool SimControlManager::AddDynamicModel(const std::string &dynamic_model_name) {
  if (IsEnabled()) {
    auto *model_factory = DynamicModelFactory::Instance();
    return model_factory->RegisterDynamicModel(dynamic_model_name);
  } else {
    AERROR << "Sim control manager is not enabled! Can not download dynamic "
              "model to local!";
    return false;
  }
}

bool SimControlManager::ChangeDynamicModel(
    const std::string &dynamic_model_name) {
  auto *model_factory = DynamicModelFactory::Instance();
  auto next_model_ptr_ = model_factory->GetModelType(dynamic_model_name);
  if (!next_model_ptr_) {
    AERROR << "Can not get dynamic model to start.Use original dynamic model!";
    return false;
  }
  ResetDynamicModel();
  model_ptr_ = next_model_ptr_;
  next_model_ptr_ = nullptr;
  model_ptr_->Start();
  enabled_ = model_ptr_->IsEnabled();
  current_dynamic_model_ = dynamic_model_name;
  return enabled_;
}

bool SimControlManager::DeleteDynamicModel(
    const std::string &dynamic_model_name) {
  auto *model_factory = DynamicModelFactory::Instance();
  return model_factory->UnregisterDynamicModel(dynamic_model_name);
}

void SimControlManager::Start() {
  if (enabled_ && model_ptr_ != nullptr) {
    return;
  }

  auto *model_factory = DynamicModelFactory::Instance();
  if (!model_ptr_) {
    model_ptr_ = model_factory->GetModelType(FLAGS_sim_perfect_control);
  }
  if (!model_ptr_) {
    AERROR << "Failed to get default sim control model: "
           << FLAGS_sim_perfect_control;
    return;
  }

  current_dynamic_model_ = FLAGS_sim_perfect_control;
  model_ptr_->Start();
  enabled_ = model_ptr_->IsEnabled();
}

void SimControlManager::Start(double x, double y) {
  if (enabled_ && model_ptr_ != nullptr) {
    return;
  }

  auto *model_factory = DynamicModelFactory::Instance();
  if (!model_ptr_) {
    model_ptr_ = model_factory->GetModelType(FLAGS_sim_perfect_control);
  }
  if (!model_ptr_) {
    AERROR << "Failed to get default sim control model: "
           << FLAGS_sim_perfect_control;
    return;
  }

  current_dynamic_model_ = FLAGS_sim_perfect_control;
  model_ptr_->Start(x, y);
  enabled_ = model_ptr_->IsEnabled();
}

void SimControlManager::Restart(double x, double y) {
  // reset start point for dynamic model.
  if (!IsEnabled() || !model_ptr_) {
    AERROR << "Sim control is invalid,Failed to restart!";
    return;
  }
  model_ptr_->Stop();
  model_ptr_->Start(x, y);
  enabled_ = model_ptr_->IsEnabled();
  return;
}

void SimControlManager::RunOnce() {
  if (model_ptr_ != nullptr) {
    model_ptr_->RunOnce();
  }
}

void SimControlManager::Stop() {
  if (enabled_) {
    enabled_ = false;
    ResetDynamicModel();
    std::system(FLAGS_sim_obstacle_stop_command.data());
    model_ptr_ = nullptr;
    current_dynamic_model_ = "";
  }
}

}  // namespace dreamview
}  // namespace apollo
