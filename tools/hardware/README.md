# Hardware-profile builds

For the simpler customer-facing entrypoint, see [product builds](../products/README.md).
The commands below expose advanced hardware selections and overlay packaging.

Run inside the managed development container as the mapped non-root user:

```bash
python3 scripts/bzlmod.py dev build --config=cpu --config=opt \
  --config=hw_x86_devkit //tools/hardware:hardware
python3 scripts/bzlmod.py dev test --config=cpu --config=opt \
  --config=hw_x86_devkit //tools/hardware:tests
```

The `pkgs/hardware` Git checkout contains independent Bzlmod roots:
`camera/`, `camera_gst/`, `canbus/`, `vehicle/`, `lidar/`, `radar/`, `gnss/`,
`hal/`, `image_decompress/`, `audio/`, `video/`, and `build/`, named
`hardware.<function>`. They are real modules, not aliases
of one umbrella dependency. `pkgs/transform` is `wheelos_transform`.
`.bazelrc.dev` overrides each module root; the product `MODULE.bazel` declares
its full inventory. A camera-only consumer declares `hardware.camera` without
resolving CAN, vehicle or LiDAR SDK modules. Shared settings live in
`hardware.build`; vehicle and CAN-based radar depend on `hardware.canbus`.
This Starlark-only support module also owns installation rules, templates and
the shared provider. It does not depend on transform or functional drivers.
GNSS depends on `hardware.hal`. Image decompression and GStreamer camera have
their own module identities; neither is an implicit dependency of V4L2 camera.
Code is directly in each module root (for example
`pkgs/hardware/audio/microphone_component.cc`); there is no physical
`modules/drivers` nesting. Native Bazel header/Proto prefixes preserve logical
include/import names, while `runtime_paths.bzl` preserves deployed DAG paths.
These new modules have not been published to a registry.
Normal registry/release mode is not available until their source revisions
and registry entries are published. Do not copy the checkout directories into
a binary distribution.

`hw_x86_devkit` selects camera, SocketCAN, Huace GNSS, Hesai, and Devkit.
`hw_orin_devkit` instead selects camera_gst and the existing Jetson Orin
platform. It requires the Orin toolchain and system libraries; selecting an
ARM platform on x86 is not itself a cross-compilation toolchain.

Profiles are versioned in `profiles.bazelrc`. Individual choices can be
overridden with typed flags:

```bash
python3 scripts/bzlmod.py dev build --config=cpu --config=opt \
  --config=hw_x86_devkit //tools/hardware:hardware \
  --@hardware.build//tools/hardware:gnss=novatel \
  --@hardware.build//tools/hardware:lidar_livox=true
```

LiDAR flags are independent: this example selects **both** Hesai and Livox.
Disable the former explicitly to replace it. A profile name identifies the
preset, while `manifest.json` records the effective overrides.

Legacy shorthand commands now route to the external source targets; see
[command migration](../products/README.md#legacy-command-migration).
`--define=vehicle=devkit` still selects the legacy vehicle factory when the
typed vehicle flag is `legacy`. Conflicting typed/legacy choices are rejected
by the hardware root. A selected vehicle cannot use `canbus=none`.
If a local rc still sets `--define=vehicle=devkit`, explicitly change that
define too when choosing another vehicle or `none`; it is not silently ignored.

## Source development: no archive required

The `hardware` root builds an explicit runtime directory, not `pkg_tar`.
It does not scan or install the contents of `bazel-bin`.
Run a reviewed DAG with its matching mainboard and library environment:

```bash
bazel-bin/tools/hardware/runtime/bin/mainboard-hardware \
  -d modules/drivers/camera/dag/camera.dag
```

This example opens cameras; use it only on an appropriately configured device.
To check ELF loading without initializing any sensor or vehicle component:

```bash
python3 tools/hardware/runtime_smoke.py bazel-bin/tools/hardware/runtime
```

The generated layout uses stable product paths, not external-repository names.
DAG/config references under `/apollo/` are rebased relative to the runtime root.
Other absolute paths (devices, recordings, calibration) remain explicit inputs.
Do not pass an unmodified source-tree DAG to mainboard after extraction.
The old Dreamview hardware mode paths likewise need a deployment-specific
configuration; they are not validated by this loader-only check.

## Binary packaging: the same runtime layout

```bash
python3 scripts/bzlmod.py dev build --config=cpu --config=opt \
  --config=hw_x86_devkit //tools/hardware:install
```

Only this explicit package request runs `pkg_tar`. Extraction yields the same
runtime directory contents and `bin/mainboard-hardware` entrypoint; it does not
recompile a second variant or perform an implicit host installation.
For a published product, repeat through the registry/release mode with immutable
module versions and no source overrides.

## Outputs and limits

- `bazel-bin/tools/hardware/manifest.json`: resolved choices and component list.
- `bazel-bin/tools/hardware/runtime/`: mainboard, selected plugin binaries,
  declared shared-library runfiles, rebased runtime examples, generated example
  configs, and `hardware/` metadata.
- `bazel-bin/tools/hardware/install.tar.gz`: selected component binaries under
  `bazel-bin/modules/...`, runtime files under `modules/...`, and manifest/build
  revision under `hardware/`, plus the matching mainboard and library runfiles.
- This is a matched **hardware runtime**, not a full autonomous-driving product.
  The target still needs compatible OS/SDK libraries and device/calibration
  resources. Undeclared system-library dependencies are not discovered by
  scanning the build machine. This archive cannot upgrade an arbitrary older
  runtime or establish ABI compatibility across separate builds.
- Generated GNSS/vehicle configs match the selected parser/brand but are
  **examples**, not discovered hardware settings. Vehicle examples start in
  manual mode. Review ports, CAN interface, receiver parameters, projection,
  calibration, and external assets before any hardware operation.
- Generic brand runtime filegroups contain example DAGs; optional external
  calibration/map resources are not automatically downloaded.
- Velodyne's existing example DAG references its driver, parser, compensator,
  and fusion components, so that selection includes all four. Splitting this
  legacy runtime DAG into optional capabilities is a later migration.
- camera_gst is not an Image-message replacement for camera; consumers must
  support its GPU-frame/RTP interface.

## Validation and timing

```bash
python3 -m unittest discover -s scripts/ci/tests -p test_hardware_build.py -v
python3 -m unittest discover -s scripts/ci/tests -p test_hardware_profile.py -v
python3 scripts/ci/hardware_build_benchmark.py \
  --output-dir /apollo/data/log/hardware_build_validation/repeated \
  --rebuild-local-sources
```

The benchmark preserves shared caches, warms each variant once, then retains
all three wall-clock samples, commands, BEP events, and Bazel profiles. It
stops on the first failure. Warm timings are not cold compile measurements.
Run builds sequentially on an otherwise idle machine for stronger attribution.
The optional local-source probe adds a fresh, unused per-file macro to matched
hardware-owned compilation actions, including bundled RTKLib sources. Each variant uses a
different probe value to prevent cross-variant action-cache reuse. Third-party
caches remain intact. This measures a scoped source-tree rebuild, **not** a
cold SDK/toolchain build; a single rebuild sample is not a statistical estimate.

The design, source evidence, staged acceptance, and measured limitations live
in [the durable build context](../../ad_agents/context/framework/build/hardware-profile-builds.md).
