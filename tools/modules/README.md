# Selected algorithm modules

For the simpler customer-facing entrypoint, see [product builds](../products/README.md).
The commands below expose advanced module selections.

Run inside the managed development container as the mapped non-root user:

```bash
bash apollo.sh build_opt algorithms --config=cpu --config=algo_rtk_open_space
bash apollo.sh build_opt algorithms --config=cpu \
  --//tools/modules:include=ndt_localization,planning,control
```

`algo_rtk_open_space` selects RTK localization, standalone open-space planning
and control. `algo_ndt_planning` selects NDT localization, public-road planning
and control. These are build inventories, not certified driving configurations.
An explicit `include` list replaces the preset list rather than appending to it.
Empty or unknown module selections fail instead of silently building nothing.

Hardware and algorithms remain independently selectable and can be combined:

```bash
bash apollo.sh build_opt hardware algorithms --config=cpu \
  --config=hw_x86_devkit --config=algo_rtk_open_space
```

`defs.bzl` maps each supported module name to its existing production component
or library. Add one entry to support another module. `local_planning` currently
exposes a planner library, not a Cyber component; its simulator is not selected.
GPU modules retain their existing platform/toolchain requirements.
Use explicit Bazel labels for targets outside this list.

Business modules resolve to independent `wheelos_*` Bzlmod capability packages
under `pkgs/`, like `transform`. Package boundaries do not merge component
processes or change algorithms:

| Package | Modules |
| --- | --- |
| `lidar_perception` | lidar_object_detection, lidar_semantic_segmentation, lidar_fusion; read-only perception_diagnostics |
| `camera_perception` | camera_semantic_segmentation |
| `multimodal_perception` | bevfusion |
| `world_model` | lane, traffic_light, storytelling, temporal/local-map world_model |
| `localization` | localization (OpenLoc host adapter), ndt_localization, rtk_localization |
| `planning` | planning, local_planning, open_space_planning, map adapters/tools/fixtures |
| `safety` | guardian, collision_guardian, monitor |
| `hmi` | dreamview |
| Independent | prediction, control, mission, routing, simulation |
| `perception` | Legacy perception, retained until its consumers migrate |

Mission owns task orchestration, Routing road-network routes, and Storytelling
environment facts (not Planning's selected execution scenario). Safety uses
Common's shared HMI configuration and mode loader, not the HMI implementation.
Monitor still requires an HMIStatus publisher; extraction does not introduce a
headless default or change safety gating.

Component shorthands such as `lane`, `monitor`, and `rtk_localization` select only
their module subtree. Capability names such as `world_model` select the
whole package. Existing same-name module build/test shorthands remain recursive.
Release shorthands select a module installer; capability-only names select its
aggregate installer. Explicit `@wheelos_world_model//:install_dir`,
`@wheelos_planning//:install_dir` and
`@wheelos_localization//:install_dir` select entire grouped packages.
The OpenLoc Cyber host sources belong to `pkgs/localization`; the estimator
library remains in `pkgs/openloc`. Use `--config=dev` to select the local
checkouts. Existing C++ includes and Proto imports retain their `modules/`
paths. Source DAG/launch files are under each checkout's `modules/` directory
and refer to the Apollo workspace's Bazel 8 component outputs.
These checkouts are ignored by the product Git repository and declared in
`wheelos.repos` as development sources tracking `main`. Tracking `main` is a
development policy, not a reproducible release pin. Bzlmod registry
publication is separate. Commit package source in each package repository;
keep the product repository limited to integration and manifest changes.

Planning/Control's joint contract regression stays in
`//tools/control/tests:motion_chain_contract_test`; the Control package has no
reverse Planning dependency. Common owns shared CUDA/TensorRT/LibTorch integration;
Planning owns IPOPT. Lane
retains GPU-only component selection; its CPU preprocessing/calibration tests
do not validate inference or a runtime engine.

LiDAR detection and Traffic Light still consume narrow legacy Perception
utilities. Legacy Perception tracking and semantic-map libraries also consume
Prediction, which consumes Planning's map adapter. Planning consumes World Model,
so the capability-module graph is cyclic even when selected Bazel target
closures remain acyclic. These transition dependencies must be removed before
retiring legacy Perception, rather than copied or hidden.
The shared `pnc_map` planning adapter keeps its `modules/map/pnc_map` include
prefix under Planning; Prediction and HMI declare that narrow library dependency.
HDMap remains in the independent Map package. The remaining Apollo `modules/map`
directory now lives in Planning; it does not merge HDMap into Planning.
Product-wide runtime defaults live in
[`products/apollo_lite/conf/`](../../products/apollo_lite/conf/README.md),
separate from module parameters and Common code. Build-time product selections
stay in `tools/products/`; Common SDK packaging is owned by
`@wheelos_common//tools:package_files`.

Only the selected roots and their real dependency closure are built. Internal
controllers, scenarios and estimators are not split or replaced. Required
shared libraries can still come from an otherwise unselected module; selecting
open-space planning does not forbid its existing shared planning helpers.
Tests, simulators, offline tools and broad install targets are not added as
independent build roots. Their explicit build/test commands remain available.

This target builds binaries/libraries only. It does not configure topics,
launch processes, select calibration/model/map assets, validate compatibility
between modules, or create a standalone deployment package.

See [legacy command migration](../products/README.md#legacy-command-migration)
for old mode aliases and recursive commands. Direct Bazel usage is equivalent:

```bash
bazel build --config=cpu --config=opt --config=algo_rtk_open_space \
  //tools/modules:algorithms
```
