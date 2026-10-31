# Transform runtime

`wheelos_transform` owns the query libraries, Cyber transport, static publisher
and vendored tf2. The product owns assembly and deployment. Build with the
managed container and the opt-in development overrides:

```bash
python3 scripts/bzlmod.py dev build --config=cpu //tools/transform:runtime
```

The shared runtime layout collects mainboard, plugin runfiles and DAG/launch
files. It preserves `modules/transform` deployment paths while resolving the
component from the external module. `:runtime` does not archive or install;
`//tools/transform:package` explicitly archives the same layout. The old
`:install` label has been removed.

For an explicit directory installation use the Bazel-maintained `pkg_install`:

```bash
python3 scripts/bzlmod.py dev run --config=cpu //tools/transform:install_dir \
  -- --destdir /absolute/path/to/transform-runtime
```

This copies the same declared runtime using rules_pkg. The shared
`:package_files` mapping supplies executable/data permission groups for both
directory installation and archives. It does not run during
source runtime builds. Legacy module installation scripts have been removed.

## One entrypoint for development and deployment

To retain bare `mainboard` / `cyber_launch` commands, configure once in the
existing project `.env.global` before container creation:

```bash
WHEELOS_RUNTIME_PRODUCT=transform
```

The selection is forwarded into the container. Its login and interactive
non-login shells load workspace `setup.bash`, which adds stable command
wrappers without injecting runtime libraries into the build environment.
Existing containers need recreation to receive the new bootstrap/environment.
Build `//tools/transform:runtime` explicitly, then use:

```bash
mainboard -d modules/transform/dag/static_transform.dag
cyber_launch start modules/transform/launch/static_transform.launch
```

The wrappers resolve the selected runtime on each invocation, prepare only
their child environment and exec the original tools. Missing runtimes fail;
builds do not implicitly change the selected product.
The same matched runtime also provides `cyber_recorder` and `cyber_monitor`.
Those two commands preserve the caller's working directory; relative recording
paths do not become paths inside the runtime. Use a writable recording directory
and SIGINT for graceful finalization. Monitor requires a TTY and target terminfo.
For a preinstalled runtime used from the workspace, choose an absolute
`WHEELOS_RUNTIME_ROOT` instead; do not set both selectors.
With neither selector, existing legacy shell setup is unchanged.
Wrappers default glog to stderr unless a logging destination is explicitly
configured; file logging requires a writable target directory.

Inside the managed container:

```bash
bash apollo.sh run transform
bash apollo.sh install transform --source=dev --destdir=/absolute/path/to/runtime
bash apollo.sh package transform --source=dev
```

`run` builds and executes `//tools/transform:run`, using official Python
runfiles to locate the runtime directory; it neither installs nor archives.
Install/archive commands delegate to the existing rules_pkg targets.
Without `--source=dev`, delivery uses registry/release sources and must fail
until the unpublished modules are available; development overrides are never
an implicit release fallback.
The frozen release lockfile must also be supported by the pinned Bazel version;
the current release check stops at that lockfile error before registry resolution.

Start an installed or extracted runtime without Bazel:

```bash
/absolute/path/to/runtime/bin/run
# Override the default DAG, or start a reviewed multi-process launch:
/absolute/path/to/runtime/bin/run --dag modules/transform/dag/static_transform.dag
/absolute/path/to/runtime/bin/run --launch modules/transform/launch/static_transform.launch
# Or activate familiar commands once per deployed shell:
source /absolute/path/to/runtime/bin/setup.bash
mainboard -d modules/transform/dag/static_transform.dag
cyber_launch start modules/transform/launch/static_transform.launch
```

The same `bin/run` is used during development. Single-process mode executes
the exact packaged mainboard; launch mode delegates to packaged Cyber launch,
which selects that mainboard rather than a stale executable in `PATH`.
Mainboard options follow `--`, for example `bin/run -- -p static_transform`.
Reviewed DAG/launch/config files must reside inside the runtime root; external
absolute config paths are not supported by Cyber's configuration resolver.
The legacy `bin/mainboard-hardware` remains available for direct mainboard arguments.
Bash, Python 3 and compatible system/SDK libraries are target prerequisites.
Library lookup starts at this runtime's `lib`; additional SDK paths require
explicit `WHEELOS_RUNTIME_LIBRARY_PATH`, not inherited `LD_LIBRARY_PATH`.

Supply the selected vehicle calibration environment described in the module's
README. Missing or invalid calibration must fail startup. A load-only check
does not validate calibration or TF publication:

```bash
python3 tools/hardware/runtime_smoke.py <runtime-directory> \
  --plugin bazel-bin/modules/transform/libstatic_transform_component.so
```

Run that check against the source runtime, the installed directory and an extracted
`:package` archive. Target-system libraries remain required. Source checkout validation
does not establish Registry or remote release acceptance.

`transform.service` is an optional systemd example, not automatically installed,
enabled or started. Adjust the example `wheelos` service account and `/opt` paths
for the target and provision its readable `/etc/wheelos/transform.env` calibration
environment before deployment. The example routes glog output to stderr/journald,
not the installed runtime tree; any additional file logging needs an explicit
writable target directory. `Type=exec`
reports execution, not calibration readiness or TF availability.
Single-process mainboard shuts down on SIGINT; multi-process Cyber launch
also handles SIGTERM and bounds child shutdown. The service uses SIGINT and an
outer stop timeout. Readiness requires separate application-level evidence.

The reusable migration rationale and contracts are documented in
[modular runtime commands](../../ad_agents/context/framework/build/modular-runtime-command-contract.md).
