// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

namespace apollo {
namespace common {

// Steering angles are road-wheel angles in radians. Positive front and rear
// angles both turn their axle toward the vehicle's left. Acceleration is the
// longitudinal acceleration applied over the prediction horizon.
struct VehicleModelInput {
  double longitudinal_acceleration = 0.0;
  double front_steering_angle = 0.0;
  double rear_steering_angle = 0.0;
};

}  // namespace common
}  // namespace apollo
