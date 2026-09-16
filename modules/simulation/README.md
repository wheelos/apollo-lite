# Apollo Lite Simulation

This module provides a CyberRT vehicle simulation component backed by MuJoCo
or a lightweight kinematic backend. It publishes simulated chassis and
localization feedback and consumes `ControlCommand` messages. The dynamics
models currently supported are:

- **Ackermann** front-wheel steering
- **4WS** counter-phase four-wheel steering

The MuJoCo model is responsible for vehicle dynamics, wheel contact, steering
actuators, and drive/brake torques. The simulation adapter converts the
internal state into Apollo/WheelOS messages, including chassis speed, wheel
speeds, steering feedback, and VRF acceleration.

## Quick start

The commands below are intended to run inside the Apollo test container as the
`humble` user.

### 1. Start the container

From the repository root:

```bash
whl start test
whl enter
cd /apollo
```

### 2. Build the simulation component and Cyber mainboard

```bash
bazel build @core//cyber/mainboard:mainboard \
  //modules/simulation:libsimulation_component.so
```

### 3. Start the Ackermann simulation

The default DAG loads `simulation.conf`, which uses the MuJoCo backend and the
Ackermann model:

```bash
MAINBOARD="$(bazel cquery --output=files @core//cyber/mainboard:mainboard | awk 'NF { print; exit }')"
"${MAINBOARD}" \
  -d /apollo/modules/simulation/dag/simulation.dag \
  -p simulation \
  -s CYBER_DEFAULT
```

The default configuration is:

- MuJoCo backend
- Ackermann steering
- `0.002 s` physics step
- `0.02 s` control step
- throttle control mode
- model: `/apollo/modules/simulation/model/ackermann_vehicle.xml`

Use `simulation_throttle.conf` when explicitly testing throttle mode. The
component reads the vehicle configuration from
`/apollo/modules/common/data/vehicle_param.pb.txt` unless
`--vehicle_config_path` is overridden.

### 4. Start the 4WS simulation

For 4WS, use the dedicated configuration file. It enables the
four-wheel-steering vehicle model and the 4WS vehicle parameters:

```bash
"${MAINBOARD}" \
  -d /apollo/modules/simulation/dag/simulation.dag \
  -p simulation_4ws \
  -s CYBER_DEFAULT \
  --flagfile=/apollo/modules/simulation/conf/simulation_4ws.conf
```

The 4WS configuration requires a positive rear-steering limit and uses
counter-phase rear steering. The same MuJoCo vehicle XML supplies the rear
steering actuators.

## Runtime interfaces

### Input

The component subscribes to the control command topic configured by
`control_command_topic` (normally `/apollo/control`).

Throttle and brake commands use percentage units in `[0, 100]`. Steering
commands use percentage units in `[-100, 100]` and are converted to radians
using the configured maximum steering angle.

### Output

The component publishes:

- chassis feedback, normally `/apollo/canbus/chassis`
- localization feedback, normally `/apollo/localization/pose`
- chassis detail feedback, normally `/apollo/canbus/chassis_detail`

The longitudinal acceleration reported in
`LocalizationEstimate.pose.linear_acceleration_vrf.y` is the forward
acceleration under Apollo's VRF convention (`x=right`, `y=forward`).

## Configuration

Simulation flags are stored in `conf/`:

| File | Purpose |
| --- | --- |
| `simulation.conf` | Default MuJoCo Ackermann configuration |
| `simulation_throttle.conf` | Explicit throttle-mode configuration |
| `simulation_speed.conf` | Speed-control configuration |
| `simulation_4ws.conf` | MuJoCo 4WS configuration |

Important flags include:

```text
--sim_backend_type=mujoco
--sim_vehicle_model=ackermann
--sim_control_mode=throttle
--sim_model_path=/apollo/modules/simulation/model/ackermann_vehicle.xml
--sim_physics_dt=0.002
--sim_control_dt=0.02
--sim_command_timeout=0.20
```

The component validates vehicle geometry and steering limits against the
MuJoCo model during initialization. A geometry or steering-limit mismatch
causes initialization to fail instead of silently running with inconsistent
parameters.

## Tests

Run the simulation unit tests with:

```bash
bazel test \
  //modules/simulation:ackermann_model_test \
  //modules/simulation:mujoco_backend_test \
  //modules/simulation:cyber_adapter_test \
  //modules/simulation:simulation_engine_test \
  //modules/simulation:simulation_control_test
```

## Troubleshooting

- **The component exits during startup:** check the vehicle configuration path,
  MuJoCo model path, and the geometry/steering-limit validation messages.
- **No chassis or localization feedback:** confirm that Cyber is running and
  that the control and output topic flags match the active subscribers.
- **The vehicle does not move:** publish a valid `ControlCommand`, use drive
  gear, and keep sending commands within `--sim_command_timeout`.
