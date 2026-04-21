# Planning, Routing, and Dreamview Build

This note captures the build workflow for `planning`, `routing`, and `dreamview` in Apollo Lite.

## Required Workflow

1. Enter the **explicit** dev container from the workspace root:

    ```bash
    bash docker/scripts/whl.sh enter
    ```

   If multiple `apollo_dev_*` containers are running, do not pick one with a
   naive `docker ps | grep '^apollo_dev' | head -n1` pipeline. That can select
   another user's container. For this workspace, target `apollo_dev_wfh`
   explicitly when you need deterministic build/test behavior.

2. Build inside the container from `/apollo` with the exact Bazel command below:

    ```bash
    bazel build --config=opt --config=gpu --copt=-mavx2 --host_copt=-mavx2 --jobs=16 --local_resources=cpu=8 --local_resources=memory=HOST_RAM*.7 -- //modules/planning/... //modules/routing/... //modules/dreamview/...
   ```

## Why The Command Must Stay Exact

- Run the build **inside the container**, not on the host.
- Do **not** casually change build options between iterations.
- The command line is part of Bazel's cache key, so changing options unnecessarily reduces cache reuse and slows repeated validation cycles.

## `apollo.sh build_opt_gpu planning` parity

- `./apollo.sh build_opt_gpu planning` expands to `scripts/ci/apollo_build.sh`
  with `--config=opt --config=gpu planning`.
- For x86_64, `apollo_build.sh` also adds the same low-level tuning used here:
  `--copt=-mavx2 --host_copt=-mavx2`, plus resource args for jobs, CPU, and
  memory.
- The important difference is **target scope**: `build_opt_gpu planning` builds
  `//modules/planning/...` only. It does not automatically build
  `//modules/routing/...`, `//modules/dreamview/...`, or
  `//cyber/mainboard:mainboard`.
- Borrow-lane runtime validation needs those additional runtime artifacts, so a
  planning-only build is not sufficient by itself.

## Scope

Use this command when you need to rebuild the `planning`, `routing`, and `dreamview` trees together for simulation or runtime validation work.
