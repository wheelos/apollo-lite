---
name: multi-repo-dev
description: Apollo-Lite multi-repository checkout, local override validation, and versioned registry publication workflow.
---

# Multi-Repository Development

## When

Use when checking out or developing WheelOS dependency modules together with
the Apollo-Lite consumer, reviewing `.bazelrc.dev`, or preparing registry
publication after local validation.

## Prerequisites

- Host `vcs`, the root `wheelos.repos`, and the managed development container.
- Preserve dirty checkouts and existing dependency declarations.
- Run Bazel in `/apollo` as the current mapped non-root user; reuse existing
  toolchains, dependencies, and caches.

## Steps

### 1. Checkout sources

Run on the host from the Apollo-Lite root:

```bash
vcs validate < wheelos.repos
mkdir -p pkgs
vcs import pkgs < wheelos.repos
vcs status pkgs
```

Preserve existing local changes. Verify repository URLs, revisions, module
names, and BUILD targets. Checkout confirms source identity, not correctness.
`pkgs/` is ignored by the consumer; each repository has its own Git history.

### 2. Develop and validate locally

- Keep each module's `MODULE.bazel` and exported BUILD targets.
- Declare `bazel_dep` in the direct consumer before integration. A transitive
  dependency needs no redundant root declaration unless the root uses its
  labels directly. Independent module development can precede integration.
- `--override_module` replaces an existing dependency-graph module; it does
  not add a dependency. Registration is not required for local overrides.
- Use module names, not BUILD aliases: `wheelos_core` is the override key,
  while `@core` is its consumer label prefix.

The root `.bazelrc` imports `.bazelrc.dev`. Keep overrides opt-in:

```text
common:dev --registry=file://%workspace%/pkgs/bazel-central-registry
common:dev --override_module=wheelos_common=%workspace%/pkgs/common
common:dev --override_module=wheelos_map=%workspace%/pkgs/map
common:dev --override_module=wheelos_core=%workspace%/pkgs/core
common:dev --override_module=wheelos_msgs=%workspace%/pkgs/wheelos_msgs
```

Enter the managed container as the current mapped non-root user:

```bash
bash docker/scripts/whl.sh enter dev
```

From `/apollo`, resolve and verify the four `/apollo/pkgs/...` mappings,
then build the cross-module smoke target:

```bash
bazel mod graph --config=dev --lockfile_mode=off
bazel mod show_repo --config=dev --lockfile_mode=off \
  @core @wheelos_common @wheelos_map @wheelos_msgs
bazel build --config=dev --lockfile_mode=off \
  @wheelos_map//modules/map/hdmap:hdmap
```

Run focused tests for changed behavior. For wrapper builds, use
`bash apollo.sh build <module> --config=dev`. Reuse existing dependencies,
toolchains, and caches; do not run Bazel as root or hide cache ownership errors.
Stop at the first actionable failure; do not retry or expand scope blindly.

### 3. Publish source and register the version

After local validation, publish the tested source as an immutable revision or
release archive, dependencies before consumers. Commit/push/release actions
require explicit authorization.

If the exact module version is absent, add it in
`pkgs/bazel-central-registry` following that repository's `docs/README.md`:

- `modules/<name>/metadata.json`, listing the version.
- `modules/<name>/<version>/MODULE.bazel`, `source.json`, and `presubmit.yml`,
  plus referenced patches or overlays.

Match metadata to the released source and use retrievable immutable sources
with integrity hashes. Do not publish machine-specific `local_path` sources.
New source changes need a new version; never rewrite a published version.
Bzlmod resolves versions; it does not upload or release Git repositories.

For reproducible releases or handoffs, save an exact revision manifest:

```bash
vcs export --exact pkgs > wheelos.exact.repos
```

The export excludes uncommitted changes. Generate the final handoff manifest
after all included source and registry changes have been committed and published.

### 4. Validate registration and promote

Disable overrides for all modules in the release dependency set, retain the
local registry, and select the intended release versions in the validation
consumer's `bazel_dep` declarations before resolving/building/testing. Ensure
Bazel actually selects the edited registry: the local registry in
`.bazelrc.dev` follows the remote registries and is only a fallback.

Only after registry validation, submit its changes to the remote repository.
Confirm the configured remote registry serves the new version, update consumer
versions, and validate normal consumption without `--config=dev` or any other
local override. Local override success is not publication-path validation.

Restore any temporary validation edits to `.bazelrc.dev` and dependency
declarations without reverting pre-existing work. Retain only intended release
version updates; development mode remains opt-in.

## Acceptance

- Local validation: all four mappings point to `/apollo/pkgs/...`, the smoke
  target builds, and focused tests for changed behavior pass.
- Registry validation: exact release versions resolve, build, and pass relevant
  tests without overrides for the release dependency set.
- Publication: the configured remote registry serves those versions and normal
  consumer validation succeeds without local overrides or the local registry.
- Reproducible handoff: published revisions are recorded in the exact manifest.
  A successful Git push alone does not satisfy publication acceptance.

## Failure handling

Stop at the first actionable error and report the failing phase. Do not change
versions, lockfiles, registry sources, cache ownership, or install packages to
hide a failure. Request approval before expanding scope; preserve unrelated work.
