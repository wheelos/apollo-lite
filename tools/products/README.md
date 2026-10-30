# Product builds

Run builds and previews inside the managed development container as the mapped
non-root user. Help and profile listing also work outside the container.

```bash
bash apollo.sh profiles
bash apollo.sh build --product=devkit-x86 --show
bash apollo.sh build --product=devkit-x86
bash apollo.sh build --product=devkit-x86 --mode=dbg --jobs=4
```

The two **example build inventories** are:

| Product | Hardware | Algorithm modules | Compute / default mode |
| --- | --- | --- | --- |
| `devkit-x86` | `hw_x86_devkit` | RTK localization, standalone open-space planning, control | CPU / opt |
| `devkit-x86-ndt` | `hw_x86_devkit` | NDT localization, public-road planning, control | CPU / opt |

These are not hardware-qualified products or verified runtime combinations.
They do not start sensors or driving software. No last-selected product is
saved. `profiles.bazelrc` is the single product registry and composes the existing
hardware and algorithm presets; listing reads names only, while Bazel resolves
their actual values.

Every selected build first analyzes and shows the effective hardware choices,
module inventory, platform, mode and output paths. `--show` stops there: it does
not execute compile/link/package actions, but Bazel analysis may resolve/fetch
dependencies and update its analysis cache. Local rc overrides still apply;
conflicting hardware selections fail explicitly before compilation.

Product CPU/GPU and platform selection is explicit; the new path does not choose
a different inventory based on the host GPU or Orin device tree. Required
toolchains/libraries must still be installed. Selecting a platform does not
provide a cross-compilation toolchain.

## Custom selection

```bash
bash apollo.sh build --hardware=hw_x86_devkit \
  --modules=rtk_localization,open_space_planning,control --compute=cpu
bash apollo.sh build --modules=rtk_localization,control --compute=cpu --show
```

Custom builds accept hardware, modules or both, require `--compute=cpu|gpu`,
and default to `--mode=opt`. Hardware presets select a target platform;
module-only builds use Bazel's configured/default platform. Products cannot
be mixed with custom inventories or compute overrides. To change the inventory,
use a custom build or add a reviewed product preset.

`--mode=opt|dbg`, `--jobs=N`, `--cpus=N` and `--rams=MB` are the customer-facing
options. Advanced Bazel flags remain available through explicit
hardware/algorithm targets, not the product shorthand.

## Legacy command migration

Old commands remain supported; they are not silently redirected to selected
builds. Use `build` with options for new customer scripts. All commands below
start with `bash apollo.sh`.

| Old command / name | Recommended selected-build form |
| --- | --- |
| `build` or `build_opt` without a module | `build --product=devkit-x86`; use `build --all` only when a recursive build is intended |
| `build_opt planning` | `build --modules=planning --compute=cpu` |
| `build_dbg control` | `build --modules=control --compute=cpu --mode=dbg` |
| `build_opt hardware --config=hw_x86_devkit --config=cpu` | `build --hardware=hw_x86_devkit --compute=cpu` |
| `build_opt hardware algorithms --config=hw_x86_devkit --config=algo_rtk_open_space --config=cpu` | `build --product=devkit-x86` |
| `build_opt_cpu` / `build_dbg_gpu` and other mode aliases | Choose an inventory, then use `--mode=opt\|dbg` and `--compute=cpu\|gpu` for a custom build; products own their compute choice |
| `.custom.bazelrc` mentioned by old `config` help | Manually transfer needed settings to `.bazelrc.user`, which is actually imported; do not overwrite existing user settings |
| `config --clean` | Edit `.bazelrc.user` manually; `config` is now read-only and rejects `--clean` |

These are migration examples, not identical coverage promises: old module
commands request `//modules/<module>/...` plus runtime tools; selected builds
request production roots and necessary dependencies, not all tests/tools.
Old mode aliases without an explicit compute config retain host GPU inference;
new custom builds require `--compute`, and products specify compute/platform.
`build --all` defaults to CPU/opt, not the old host-inferred mode.

For driver brands or vehicle overrides, continue using
[advanced hardware flags](../hardware/README.md). `--define=vehicle=devkit`
still works when the typed vehicle setting is `legacy`. When choosing a
different typed vehicle, also update a conflicting legacy define in local rc
or on the command line; the hardware root reports the conflict instead of
silently overriding it. `hw_*` / `algo_*` / `product_*` rc names and direct
Bazel target labels are unchanged.

## Full builds, tests and packaging

Product testing and complete product packaging are **not implemented**. Use
existing explicit module test commands and `test hardware` until product test
inventories are defined. `hardware_package` still builds only a hardware overlay,
not a standalone runtime or an algorithm deployment archive.

Product selection never modifies user rc files.
