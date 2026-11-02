# Shared Cyber system

This is the final Bzlmod integration/installation root, not another middleware.
It installs core tools and `conf/cyber.pb.conf` once. Vehicle and sensor modules
contribute shared components, business configuration and DAG/launch files.
More algorithm modules can use the same `runtime_module` rule and be added to
the same `runtime_layout`; repository boundaries do not imply process boundaries. Runtime
resources use root `conf/`, `dag/`, `launch/` and `share/` paths; component
libraries live under `lib/components/<module>/`.

The native tools are in `bin`; use the environment-aware commands in
`bin/commands`. Every command uses this root's core, library dependencies and
configuration. Recorder and monitor preserve the caller's data directory.
See the [runtime contract](../../ad_agents/context/framework/build/modular-runtime-command-contract.md).

## Build and configure

Use Bazel 8.8.1 inside the managed container as the mapped non-root user.
The cross-repository regressions declare `vehicle_infra` and `sensor_infra`
as development dependencies. Until they and the updated core API are published,
local validation must pass explicit source overrides for both infra modules;
these are not implicitly discovered or generated release entries.
The default assembly has tools only and no implicit module startup:

```bash
bazel build --config=dev --config=cpu //tools/system:runtime
source bazel-bin/tools/system/runtime/bin/setup.bash
cyber_recorder info /absolute/data/example.record
cyber_monitor
```

Choose components and reviewed business configuration explicitly. These flags
are in addition to normal hardware backend/vehicle selections:

| Flag | Meaning |
| --- | --- |
| `--//tools/system:vehicle=true` | Include the vehicle component; select its brand/CAN backend using hardware flags |
| `--//tools/system:vehicle_config=//assets:canbus_config` | Required CAN/vehicle configuration |
| `--//tools/system:vehicle_parameters=//assets:parameters` | Required reviewed geometry/calibration |
| `--//tools/system:vehicle_flags=//assets:vehicle_flags` | Required reviewed business flags |
| `--//tools/system:sensors=camera\|lidar\|combined` | Include camera, Hesai, or both |
| `--//tools/system:camera_config=//assets:camera_config` | Required when camera is selected |
| `--//tools/system:lidar_config=//assets:lidar_config` | Required when LiDAR is selected |
| `--//tools/system:cyber_configuration=//assets:cyber_conf` | Optional replacement of the **whole system's** core configuration filegroup |

Asset labels above are examples, not shipped hardware configuration. Missing
required inputs fail at analysis. New business/resource files must be mapped
under their `conf/<module_name>/`, `dag/<module_name>/`,
`launch/<module_name>/` or `share/<module_name>/` namespace; modules cannot
replace core configuration or tools. Conflicting file/library contents fail instead of overwriting each
other. `share/system-manifest.json` lists modules and hashes installed contents;
it is not a release lock or ABI compatibility claim.

For a custom core configuration filegroup, retain `conf/` root-relative paths.
All participating shells/services must use the same explicitly coordinated
`CYBER_DOMAIN_ID`, `CYBER_IP`, clock mode and transport configuration. Scheduler
configuration can differ per process name under the common `conf/` directory.
Do not source separately assembled vehicle/sensor environments into one system.

## Start and deliver

After building a selected assembly and obtaining hardware startup approval:

```bash
source /opt/wheelos/system/bin/setup.bash
cyber_launch start launch/vehicle/vehicle.launch
cyber_launch start launch/sensors/sensors.launch
mainboard -d dag/vehicle/vehicle.dag -p vehicle
```

These are alternative process arrangements, not instructions to start the same
vehicle twice. A system-owned launch can reference both modules' DAGs, or one
mainboard can load multiple DAGs when shared execution is appropriate.
There is no central daemon requirement added by this assembly.

Use `//tools/system:package` for the archive and `//tools/system:install_dir`
with `-- --destdir=/absolute/path` for official rules_pkg installation.
Apply the same selection/configuration flags to runtime, package and install.
Archive, installed and development layouts are internally identical and
relocatable. Install never starts components.
`apollo.sh run|install|package system` is also registered; use explicit
`--config=<reviewed-profile>` for selection/assets and `--source=dev` for local
delivery validation. Release sources/locks still require actual publication.

Keep recordings/logs outside the install tree. One recorder can subscribe to
all selected modules. `record` observes; `play` publishes and requires an
isolated approved domain/topic set, especially when vehicle control exists.

## No-device validation

```bash
bazel test --config=dev --config=cpu --noignore_dev_dependency \
  --override_module=vehicle_infra=/absolute/vehicle-infra-source \
  --override_module=sensor_infra=/absolute/sensor-infra-source \
  --@hardware.build//tools/hardware:vehicle=devkit \
  --@hardware.build//tools/hardware:canbus=fake \
  --@vehicle_infra//products/vehicle:config=@vehicle_infra//tests/fixtures:vehicle-fake.pb.txt \
  --@vehicle_infra//products/vehicle:vehicle_parameters=@vehicle_infra//tests/fixtures:vehicle-parameters-fake.pb.txt \
  --flaky_test_attempts=1 --test_output=errors \
  //tools/hardware:materialize_runtime_test \
  //tools/system/tests:module_test //tools/system/tests:ownership_test \
  //tools/system/tests:integration
```

The integration consumes vehicle-infra's real module contribution with FakeCAN
and sensor-infra's private synthetic contribution as external Bzlmod repositories,
never physical CAN/camera/LiDAR. One recorder decodes
both message types; a PTY monitor sees both topics; sensor publication continues
after vehicle shutdown. The same checks run after archive extraction and
official installation/relocation. Synthetic fixtures are test-only and absent
from production module selections. These checks do not qualify physical
devices, control actuation, calibration or remote releases.
