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

#include "modules/canbus/vehicle/vehicle_factory.h"

#include "modules/canbus/proto/vehicle_parameter.pb.h"
namespace apollo {
namespace canbus {

VehicleFactoryRegistry& VehicleFactoryRegistry::Instance() {
  static VehicleFactoryRegistry registry;
  return registry;
}

std::unique_ptr<AbstractVehicleFactory> VehicleFactory::CreateVehicle(
    const VehicleParameter& vehicle_parameter) {
  auto abstract_factory = VehicleFactoryRegistry::Instance().CreateObject(
      vehicle_parameter.brand());
  if (!abstract_factory) {
    AERROR << "failed to create vehicle factory with "
           << vehicle_parameter.DebugString();
  } else {
    abstract_factory->SetVehicleParameter(vehicle_parameter);
    AINFO << "successfully created vehicle factory with "
          << vehicle_parameter.DebugString();
  }
  return abstract_factory;
}

}  // namespace canbus
}  // namespace apollo
