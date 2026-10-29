# Hardware-profile builds

For the simpler customer-facing entrypoint, see [product builds](../products/README.md).
The commands below expose advanced hardware selections and overlay packaging.

Run inside the managed development container as the mapped non-root user:

```bash
bash apollo.sh build_opt hardware --config=hw_x86_devkit
bash apollo.sh build_opt hardware_package --config=hw_x86_devkit
bazel test --config=opt --config=hw_x86_devkit //tools/hardware:tests
```

`hw_x86_devkit` selects camera, SocketCAN, Huace GNSS, Hesai, and Devkit.
`hw_orin_devkit` instead selects camera_gst and the existing Jetson Orin
platform. It requires the Orin toolchain and system libraries; selecting an
ARM platform on x86 is not itself a cross-compilation toolchain.

Profiles are versioned in `profiles.bazelrc`. Individual choices can be
overridden with typed flags:

```bash
bash apollo.sh build_opt hardware --config=hw_x86_devkit \
  --//tools/hardware:gnss=novatel \
  --//tools/hardware:lidar_livox=true
```

LiDAR flags are independent: this example selects **both** Hesai and Livox.
Disable the former explicitly to replace it. A profile name identifies the
preset, while `manifest.json` records the effective overrides.

Legacy recursive commands are unchanged; see
[command migration](../products/README.md#legacy-command-migration).
`--define=vehicle=devkit` still selects the legacy vehicle factory when the
typed vehicle flag is `legacy`. Conflicting typed/legacy choices are rejected
by the hardware root. A selected vehicle cannot use `canbus=none`.
If a local rc still sets `--define=vehicle=devkit`, explicitly change that
define too when choosing another vehicle or `none`; it is not silently ignored.

## Outputs and limits

- `bazel-bin/tools/hardware/manifest.json`: resolved choices and component list.
- `bazel-bin/tools/hardware/install.tar.gz`: selected component binaries under
  `bazel-bin/modules/...`, runtime files under `modules/...`, and manifest/build
  revision under `hardware/`. No stale output directory is scanned.
- The package is a **hardware overlay**, not a standalone Apollo release. It
  needs the matching Cyber/runtime, external shared libraries, and deployment
  resources. Its original DAG paths assume an `/apollo` installation.
- Transitive project `_solib`/runfiles libraries are not bundled either; supply
  them from the same configured build. This archive alone cannot upgrade an
  arbitrary older runtime or prove deployment completeness.
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
`modules/drivers` and `modules/canbus` compilation actions. Each variant uses a
different probe value to prevent cross-variant action-cache reuse. Third-party
caches remain intact. This measures a scoped source-tree rebuild, **not** a
cold SDK/toolchain build; a single rebuild sample is not a statistical estimate.

The design, source evidence, staged acceptance, and measured limitations live
in [the durable build context](../../ad_agents/context/framework/build/hardware-profile-builds.md).
