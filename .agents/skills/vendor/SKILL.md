---
name: "vendor"
description: "Apollo-Lite Bazel Bzlmod vendor workflow for preparing and validating offline source deliveries."
---

# Bazel Vendor and Offline Delivery

## When

Use this when preparing an Apollo-Lite source bundle that must build without
network access.

## Prerequisites

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
- Resolve all delivered module versions through approved sources. Development
  overrides are not proof that published versions can be vendored.

## Steps

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

Deliver the source tree, `vendor/`, and the matching ignored
`MODULE.bazel.lock` together. Keep the lockfile untracked in Git; it is still
part of the delivery inputs needed to reproduce the resolved graph.

Validate the exact delivered targets in a clean environment with network
access disabled, using the vendor config:

```bash
bazel build --config=vendor <delivery-target-patterns>
```

## Acceptance

- The bundle contains the source tree, matching lockfile, and vendor snapshot.
- All delivered targets build in the intended environment without network
  access, using the delivered inputs rather than undeclared warm-cache sources.
- Required non-Bazel dependencies are supplied for each delivery platform.

A successful build using a warm repository cache is not proof of offline
completeness. The validation environment must not be able to fetch missing
repositories. Re-vendor and repeat this validation when the dependency graph,
delivered target set, Bazel version, or target platform changes.

Vendor mode only localizes Bazel external repositories. System packages,
compilers, CUDA/cuDNN/TensorRT, drivers, model assets, maps, and other
non-Bazel inputs must be supplied and validated separately for each delivery
platform.

## Failure handling

Stop at the first missing module, registry, or offline build error. Do not
refresh the lockfile, change versions, add development overrides, or retry with
different dependency flags to hide the failure without approval.

Record failures in the task report with the date, exact command, active
registry/config, and first error; do not keep undated task status in this skill.

## Sources

- `.bazelrc`
- `MODULE.bazel`
- `.gitignore`
- `docker/scripts/whl.sh`
- [Bazel 7.6.1 Vendor Mode](https://bazel.build/versions/7.6.1/external/vendor)
- [Bazel 7.6.1 Lockfile](https://bazel.build/versions/7.6.1/external/lockfile)
