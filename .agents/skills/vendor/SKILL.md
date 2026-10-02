---
name: "vendor"
description: "Apollo-Lite Bazel Bzlmod vendor workflow for preparing and validating offline source deliveries."
---

# Bazel Vendor and Offline Delivery

## When

Use this when preparing an Apollo-Lite source bundle that must build without
network access.

## Repository setup

- The root `.bazelrc` defines the opt-in `vendor` config:

  ```text
  common:vendor --vendor_dir=vendor
  ```

- Keep `MODULE.bazel.lock` ignored by Git when avoiding lockfile diffs in
  repository commits. Bazel still needs a matching local lockfile during
  dependency resolution. Preserve and include that exact lockfile in the
  offline delivery bundle alongside `vendor/`; do not omit it from the bundle.
- Keep dependency versions and module declarations in `MODULE.bazel`. Do not
  copy dependency sources there.

## Generate the vendor directory

Run Bazel in the managed dev container as the mapped non-root user:

```bash
bash docker/scripts/whl.sh enter
```

From `/apollo`, vendor the complete external dependency graph:

```bash
bazel vendor --config=vendor
```

For a deliberately scoped offline product, pass the complete set of target
patterns that the delivery must support instead. The vendored set must cover
all delivered build targets and their transitive dependencies.

The command requires Bzlmod dependency resolution to succeed first. If module
resolution reports a missing module or registry, stop at that first error;
do not refresh or alter the lockfile, change module versions, or retry with
different dependency flags without approval.

## Offline delivery and validation

Deliver the source tree, `vendor/`, and the matching ignored
`MODULE.bazel.lock` together. Keep the lockfile untracked in Git; it is still
part of the delivery inputs needed to reproduce the resolved graph.

Validate the exact delivered targets in a clean environment with network
access disabled, using the vendor config:

```bash
bazel build --config=vendor <delivery-target-patterns>
```

A successful build using a warm repository cache is not proof of offline
completeness. The validation environment must not be able to fetch missing
repositories. Re-vendor and repeat this validation when the dependency graph,
delivered target set, Bazel version, or target platform changes.

Vendor mode only localizes Bazel external repositories. System packages,
compilers, CUDA/cuDNN/TensorRT, drivers, model assets, maps, and other
non-Bazel inputs must be supplied and validated separately for each delivery
platform.

## Current repository blocker

The latest generation attempt stopped during module resolution because
`wheelos_map@0.1.0` was not found in either configured registry
(`bcr.wheelos.cn` or `bcr.bazel.build`). No vendor snapshot was generated.
Resolve the module's availability through the approved dependency source before
running the vendoring command again.

## Sources

- `.bazelrc`
- `MODULE.bazel`
- `.gitignore`
- `docker/scripts/whl.sh`
- [Bazel 7.6.1 Vendor Mode](https://bazel.build/versions/7.6.1/external/vendor)
- [Bazel 7.6.1 Lockfile](https://bazel.build/versions/7.6.1/external/lockfile)
