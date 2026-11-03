# Shared Cyber system

This is the final Bzlmod integration/installation root, not another middleware.
It installs core tools and `conf/cyber.pb.conf` once. Vehicle and sensor modules
contribute shared components, business configuration and DAG/launch files.
More algorithm modules can use the same `runtime_module` rule and be added to
the same `runtime_layout`; repository boundaries do not imply process boundaries.
Runtime resources use root `conf/`, `dag/`, `launch/`, `models/` and `share/`
paths; component libraries live under `lib/components/<module>/`.

The native tools are in `bin`; use the environment-aware commands in
`bin/commands`. Every command uses this root's core, library dependencies and
configuration. Recorder and monitor preserve the caller's data directory.
See the [runtime contract](../../ad_agents/context/framework/build/modular-runtime-command-contract.md).

## Dependency ownership

Bzlmod selects one source dependency graph per integration root. A module
declares repositories it directly references, not every transitive dependency.
Removing a duplicate root declaration does not remove a library from the
resolved graph when another module still needs it.

| Category | Owner and examples |
| --- | --- |
| Build rules/toolchains | `rules_cc`, `rules_python`, `rules_pkg`, `rules_cuda`, `bazel_skylib`, `platforms` |
| Third-party libraries | BCR modules directly used by Apollo code, such as Boost, Protobuf, Eigen, gRPC and OpenCV |
| WheelOS modules | Core, messages, common, map and transform; dependencies follow their public BUILD targets |
| Hardware dependencies | Hardware modules own PortAudio, LiDAR SDKs, RTKLIB and GStreamer; Apollo declares only hardware modules it directly uses |
| Apollo-local integrations | `third_party/` extensions for IPOPT, cuDNN, TensorRT, Qt and LibTorch; Dreamview assets are a separate product input |

PortAudio belongs to `hardware.audio`, while ncurses/libuuid belong to Core.
FFmpeg's extension is owned by `hardware.build`; an integration root may use
that extension to select a checksummed release SDK for the whole graph. This
does not make the SDK an Apollo-owned source dependency. Development SDK
environment variables are opt-in validation inputs, not release defaults.
Root-only overrides and SDK/toolchain selections are not inherited from a
dependency's workspace configuration.

Rules used by exported build macros are regular dependencies, even when they
run only during the build. Reserve `dev_dependency` for tools and tests that
are not part of the module's public consumer contract. A public C++ header
including GoogleTest headers still needs a regular GoogleTest dependency.
Cross-product test fixtures consumed by this test root also retain their
required dependencies.

Declared versions are minimum requirements; Bzlmod MVS can select newer
transitive versions. A release must review the resolved graph and genuine
lockfile, not infer compatibility from matching dependency declarations.
Apollo, vehicle and sensor are peer product roots, not dependencies of one
another. Cross-product regressions have their own integration module in
`tools/system/tests/`, excluded from Apollo's target traversal by `.bazelignore`.
That test module consumes Core's public assembly rules and both external
product contributions directly; it needs no Apollo product dependency.
Neither independent product depends on an Apollo checkout or on the other
product.

## Build and configure

Use Bazel 8.8.1 inside the managed container as the mapped non-root user.
The cross-repository test module declares Core, `vehicle_infra` and
`sensor_infra` as dependencies. Until the updated APIs are published, local
validation passes explicit source overrides in that test root; these are not
implicitly discovered or generated release entries. Apollo's normal dependency
graph does not contain either peer product.
From the Apollo root, the default assembly has tools only and no implicit
module startup:

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
`launch/<module_name>/`, `models/<module_name>/` or `share/<module_name>/`
namespace; modules cannot
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
Model artifacts are never downloaded or included implicitly. Explicitly
declared model files are packaged; large or hardware-specific weights should
instead be provisioned under `models/<module_name>/` from a versioned artifact
source, with a separate checksum and runtime-compatibility manifest. Config
paths should be relative to the runtime root so the same runtime can be
relocated.

This layout is an assembly contract, not an automatic migration of every
legacy Apollo application. Existing source launch/DAG/config files can still
contain checkout paths and must be explicitly adapted before contributing
them to a relocatable runtime. In particular, CenterPoint's offline input
tool resolves relative model paths against its configuration directory,
whereas its component reads them from process cwd; its existing sample is
not yet a portable system contribution. Do not paper over that distinction
with filesystem searches or implicit environment fallbacks.

## No-device validation

Pass overrides only for modules in this test root's dependency graph; do not
copy Apollo's entire development override list into this independent root.

```bash
cd tools/system/tests
# Supply explicit local overrides for every unpublished shared/hardware module
# as well as both product roots. Dependency rc files are not inherited.
bazel test <EXPLICIT_SHARED_MODULE_AND_SDK_OPTIONS> \
  --override_module=vehicle_infra=/absolute/vehicle-infra-source \
  --override_module=sensor_infra=/absolute/sensor-infra-source \
  --@hardware.build//tools/hardware:vehicle=devkit \
  --@hardware.build//tools/hardware:canbus=fake \
  --@vehicle_infra//products/vehicle:config=@vehicle_infra//tests/fixtures:vehicle-fake.pb.txt \
  --@vehicle_infra//products/vehicle:vehicle_parameters=@vehicle_infra//tests/fixtures:vehicle-parameters-fake.pb.txt \
  --flaky_test_attempts=1 --test_output=errors \
  @core//runtime:materialize_runtime_test \
  //:module_test //:ownership_test //:model_module_test \
  //:vehicle_contribution_test //:sensor_contribution_test //:integration
```

The integration consumes vehicle-infra's real module contribution with FakeCAN
and sensor-infra's private synthetic contribution as external Bzlmod repositories,
never physical CAN/camera/LiDAR. One recorder decodes
both message types; a PTY monitor sees both topics; sensor publication continues
after vehicle shutdown. The same checks run after archive extraction and
official installation/relocation. Synthetic fixtures are test-only and absent
from production module selections. These checks do not qualify physical
devices, control actuation, calibration or remote releases.
