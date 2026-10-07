---
name: "build"
description: "Apollo-Lite build workflow. Use when building Apollo modules in the managed dev container or diagnosing a Bazel build failure."
---

# Build and Compile

## When

Use this when building one or all Apollo modules or diagnosing a compile failure.

## Prerequisites

- From the repository root, enter the managed development container:

  ```bash
  bash docker/scripts/whl.sh enter
  ```

- `enter` defaults to `dev` and starts it if needed. Run build commands in the
  container from `/apollo`, as the mapped non-root user.
- Preserve the mapped user and shared cache. Never run Bazel as root or hide
  cache-permission problems with a temporary output root.

## Steps

- Build all modules with the exact wrapper command:

  ```bash
  ./apollo.sh build
  ```

- Reuse the Bazel repository cache and existing build outputs. Do not use
  `apollo.sh clean` for ordinary incremental builds.
- Use the wrapper for module builds:

  ```bash
  ./apollo.sh build <module>
  ```

- The wrapper selects the configured CPU/GPU mode. Use `./apollo.sh build_gpu
  <module>` or `./apollo.sh build_opt_gpu <module>` only when that build mode is
  specifically required.
- Use `bazel build //<target>` only when a precise target is required.
- Module tests use `./apollo.sh test <module>`.
- Do not rely on `./apollo.sh check` as the full pre-submit path until its lint
  argument is aligned: `apollo.sh` currently passes `--cpp`, which
  `scripts/ci/apollo_lint.sh` does not accept. Run build, tests, and lint as
  separate commands when each is needed.

## Acceptance

- The requested targets build successfully in the intended CPU/GPU mode.
- Required tests pass when behavior changes. Build success alone does not
  establish runtime or release acceptance.
- Verify expected artifacts and report any warnings affecting reuse or delivery.

## Failure handling

- Stop and identify the first actionable error before changing commands or
  source. Do not blindly retry or fix unrelated failures; request approval
  before expanding scope. If Bazel terminates unexpectedly, inspect its log
  and available memory before considering a retry.
- Do not compile Apollo targets outside the managed container.
- Do not hardcode a username or create root-owned Bazel outputs.
- Do not change verified build flags without a validation reason.
- Do not refresh `MODULE.bazel.lock` to recover from a missing module without
  first inspecting the registry error and getting approval for the lockfile
  change.

## Sources (SSOT)

- `apollo.sh`
- `docker/scripts/whl.sh`
- `scripts/ci/apollo_build.sh`
