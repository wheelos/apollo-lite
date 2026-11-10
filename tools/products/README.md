# Product builds

## Runtime products: one component or many

Vehicle kit, sensors kit and the low-speed product are peers. The low-speed
product composes shared hardware and algorithm modules, not either kit's
installation.
Its intended baseline includes localization, LiDAR perception, planning,
prediction, control, HMI and safety. A build inventory alone does not qualify
this runtime combination.

Use `runtime_product` for both single-component development and composite
deployments. The same definition supplies the runtime tree, launcher, archive
and official installer. A component exports a `RuntimeModuleInfo` contribution;
it does not install its own Core or change another product's environment.

```starlark
load("@core//runtime:runtime.bzl", "runtime_module")
load("@core//runtime:product.bzl", "runtime_product")

runtime_module(
    name = "localization_module",
    module_name = "localization",
    components = ["@your_localization//:component"],
    files = {
        ":localization_config": "conf/localization/component.pb.txt",
        ":localization_dag": "dag/localization/component.dag",
    },
)

runtime_product(
    name = "localization",
    modules = [":localization_module"],
    default_dag = "dag/localization/component.dag",
)

runtime_product(
    name = "autonomy",
    modules = [":localization_module", ":perception_module", ":planning_module"],
    files = {":autonomy_launch": "launch/autonomy/autonomy.launch"},
    default_launch = "launch/autonomy/autonomy.launch",
)
```

The reusable macro is owned by Core, so peer product roots do not depend on
Apollo-Lite. `//tools/products:defs.bzl` remains a compatibility export.

Labels in this example are placeholders: supply real exported targets,
reviewed configurations and a matching launch graph. Module names, component
class names, channels and deployment destinations must not conflict.
Repository boundaries do not dictate process boundaries: one DAG may load
several components, or a launch file may select several mainboards.

For either product, run inside the managed container as the mapped user (for
example using the canonical `//products/low_speed:devkit_x86` or
`//products/low_speed:devkit_x86_ndt` definitions in
`products/low_speed/BUILD`):

```bash
bash apollo.sh build --product=//products/low_speed:devkit_x86 --show --config=product_devkit-x86
bash apollo.sh build --product=//products/low_speed:devkit_x86 --mode=dbg --config=product_devkit-x86
bash apollo.sh run --product=//products/low_speed:devkit_x86 --config=product_devkit-x86 \
  -- --launch launch/low_speed/devkit_x86.launch
bash apollo.sh package --product=//products/low_speed:devkit_x86 --source=dev --config=product_devkit-x86
bash apollo.sh install --product=//products/low_speed:devkit_x86 --source=dev --config=product_devkit-x86 \
  --destdir=/absolute/install/path
```

Replace only the product label to build or start a composite. Build and run
default to explicit development source overrides. Delivery defaults to release
sources; `--source=dev` opts into local delivery validation without claiming a
published release. `--config=<reviewed-config>` selects platform/business inputs;
`--mode=dbg|opt` selects compilation mode independently. `--show` performs
analysis, not compilation or startup.

The product label is its runtime tree. The generated sibling labels append
`_run`, `_package`, `_install` and `_files`. Installed or extracted products
start through `<runtime>/bin/run`, exactly as development does; installation
never starts components. A product can omit a default graph and require an
explicit `-- --dag <path>` or `-- --launch <path>` at startup. `default_dag` and
`default_launch` are mutually exclusive.

### Resource ownership and references

`wheelos.repos` selects source checkouts; Bzlmod resolves build dependencies;
the product definition selects delivered modules, tools, configuration and
assets. None substitutes for the other. Source updates must preserve dirty
checkouts; release acceptance requires immutable published versions and the
formal dependency lock, without local source overrides.

| Resource | Owner | Runtime namespace |
| --- | --- | --- |
| Core tools and middleware configuration | One final integration root consuming Core | `bin/`, `conf/cyber.pb.conf` |
| Components | Module | `lib/components/<module>/` |
| Component configuration and DAGs | Module defaults selected by the product | `conf/<module>/`, `dag/<module>/` |
| Product-wide configuration and topology | `products/<product>/` | `conf/`, `launch/<product>/` |
| Diagnostic tools | Module, explicitly included | `bin/modules/<module>/` |
| Models and other assets | Versioned artifacts selected by product | `assets/models/<module>/<artifact-id>/` |
| HMI static files and module resources | Module | `share/<module>/` |
| Calibration and deployment overrides | Reviewed instance inputs | Explicit external configuration/asset roots |
| Logs, recordings and writable state | Deployment | Outside the installation |

Build references use Bazel labels, never `pkgs/` paths. Runtime references use
stable keys, never Bazel output repository names. DAG component paths are
relative to the runtime root. Configuration fields containing nested paths
must be resolved explicitly by their owning loader; Core does not rewrite
arbitrary strings inside protobuf or flag files.

The launcher binds `APOLLO_ROOT_DIR` and `CYBER_PATH` to the same runtime. At build time, a
`runtime_product`'s `files` mapping may explicitly override a module's default
`conf/<module>/...`, `dag/<module>/...`, or `launch/<module>/...` file with
product-owned parameters. At startup, `WHEELOS_CONFIG_ROOT` is an optional
whole-file overlay preserving keys such as `conf/control/component.pb.txt`. An
absent optional override uses the packaged default; an invalid override fails.
Declare `required_configs` on a product for instance inputs which must exist in
the external configuration root and must never fall back to examples.

Assets default to `<runtime>/assets`; an explicit `WHEELOS_ASSET_ROOT` replaces
that asset root, not a search-path fallback. Load model keys through Core's
`ResourceManager::ResolveAssetPath`, for example
`models/lidar_perception/<artifact-id>/model.onnx`. Declare `required_assets`
as a map from these keys to SHA-256 digests. Startup validates them before
components start. Model provisioning is explicit and never a startup download.
TensorRT engines additionally need reviewed GPU/runtime/precision compatibility;
a digest does not establish inference correctness or hardware compatibility.

The low-speed product's source-owned defaults live in
[`products/low_speed/conf/`](../../products/low_speed/conf/README.md);
they are still installed under runtime `conf/`. Product launch topology is
source-owned under `products/low_speed/launch/`, with component launch
contributions installed under `launch/<module>/` and product launch files under
`launch/<product>/`.

One product supplies one Core/message dependency graph and tool environment.
Do not source several product environments into one process. Recorder and
monitor preserve the caller's working directory; component launchers use the
runtime directory. Packaged trees contain declared runfiles and libraries,
not inferred host libraries or references back to the source checkout.

For familiar interactive tools, explicitly select the same build label with
`WHEELOS_RUNTIME_PRODUCT=//products/example:localization` before sourcing
`setup.bash`. For deployment, select `WHEELOS_RUNTIME_ROOT=/absolute/runtime`
instead. The two selectors are mutually exclusive.

### No-device regression

The isolated module in `tests/` exercises a real Core timer publisher and
subscriber using `wheelos_msgs` in C++ and a Python diagnostic, not vehicle hardware.
It compares single/composite startup, archive
and installation contents, relocation, a diagnostic Python tool, configuration
overlays, external assets and failure before startup on invalid inputs.
The model file is only a resource fixture, not inference evidence.

From the managed development container:

```bash
cd tools/products/tests
bazelisk --nosystem_rc --nohome_rc --noworkspace_rc test \
  --registry=https://bcr.wheelos.cn/ --registry=https://bcr.bazel.build/ \
  --override_module=wheelos_core=/apollo/pkgs/core \
  --override_module=wheelos_msgs=/apollo/pkgs/wheelos_msgs \
  --lockfile_mode=off --cxxopt=-std=c++17 --jobs=4 \
  //:integration --test_output=errors
```

This local test does not qualify remote registry consumption, the full
autonomous-driving algorithm graph, physical devices or vehicle actuation.

## Legacy build inventories

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

For explicit runtime assemblies, use `bash apollo.sh run transform`,
`bash apollo.sh install transform --destdir=/absolute/path`, or
`bash apollo.sh package transform`. Currently `transform`, `hardware` and
`system` are registered runtime assemblies, not the `devkit-*` build inventories.
Development `run` uses source overrides; delivery defaults to release sources
and requires explicit `--source=dev` for local checkout validation.
Hardware and system require a reviewed `-- --dag ...` or `-- --launch ...`;
neither starts merely because it appears in a build inventory.
See [transform runtime](../transform/README.md) for installed startup and
systemd supervision. Pure libraries do not need a runtime entrypoint.

The legacy `devkit-*` inventories still have no product test inventory or full
deployment package. Use explicit module tests and `test hardware` for them.
Label-based `runtime_product` assemblies support installation and packaging
through the common runtime contract above. `hardware_package` remains a
hardware-specific assembly, not an algorithm deployment archive.

Product selection never modifies user rc files.

For one shared core/tool/configuration installation with dynamically loaded
vehicle, sensor and future algorithm modules, use the explicit
[system assembly](../system/README.md). The default contains tools only;
business configuration and hardware selections are explicit build inputs.
