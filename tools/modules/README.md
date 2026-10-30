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
