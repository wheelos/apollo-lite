# Build and Compile

## When

Read this when compiling Apollo source modules or diagnosing a compile failure.

## Rules

- Use the repository script for container lifecycle. Use `dev` for source
  development and `test` for source validation:

  ```bash
  bash docker/scripts/whl.sh start dev
  bash docker/scripts/whl.sh enter dev
  ```

- Run Bazel commands inside the container from `/apollo`.
- Reuse the Bazel repository cache and existing build outputs. Do not use
  `apollo.sh clean` for ordinary incremental builds.
- Use the wrapper for module builds:

  ```bash
  bash apollo.sh build <module>
  ```

- Use `bash apollo.sh build` for all modules, `bash apollo.sh build_gpu
  <module>` for GPU builds, and `bash apollo.sh build_opt_gpu <module>` for
  optimized GPU builds.
- Use `bazel build //<target>` only when a precise target is required.
- Preserve the current user's mapped UID/GID in the container. Do not run Bazel
  as root or hide cache-permission problems with a temporary output root.
- On failure, locate the first real error before changing commands or retrying.
- Match validation to the change: module tests use
  `bash apollo.sh test <module>`; the full pre-submit check uses
  `bash apollo.sh check`.

## Do NOTs

- Do not compile Apollo targets outside the managed container.
- Do not hardcode a username or create root-owned Bazel outputs.
- Do not change verified build flags without a validation reason.

## Sources (SSOT)

- `apollo.sh`
- `docker/scripts/whl.sh`
- `docker/setup_host/setup_host.sh`
