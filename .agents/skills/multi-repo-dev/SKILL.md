---
name: multi-repo-dev
description: Manage vcs source checkouts, local Bzlmod overrides, and versioned registry publication for multi-repository development.
---

# Multi-Repository Development

## When

Use for coordinated development across Bazel modules, consumer integration,
or promotion from local source validation to registry-based consumption.

## Prerequisites

Read the consumer's agent rules, repository manifest, MODULE declarations,
rc files, and relevant build/test entrypoints. Identify the checkout directory,
module names, repository aliases, registry order, and approved build environment.
Preserve dirty worktrees and reuse existing toolchains and caches.

## Steps

### 1. Checkout and inspect

Use the project's vcs manifest to validate and import repositories into its
designated source directory. Inspect existing work before importing/updating.
Verify URLs, revisions, `MODULE.bazel` identities, and exported BUILD targets.
Checkout confirms source availability and identity, not correctness.

### 2. Integrate and validate locally

Declare `bazel_dep` in each direct consumer before integration. A root need
not repeat a transitive declaration unless it uses the module's labels directly.
Independent module development can precede consumer integration.

Keep local overrides opt-in in the consumer's `.bazelrc.dev`, imported by its
root `.bazelrc`. The following is a template; substitute verified module names
and workspace-relative paths:

```text
common:dev --override_module=<module>=%workspace%/<checkout-path>
```

Use module names, not repository aliases, as override keys. An override replaces
a module in the dependency graph; it does not declare a dependency. Registration
is not required for the overridden module, but other dependencies must resolve.

In the approved environment, use `--config=dev` consistently for graph
inspection, repository mapping, builds, and tests. Verify mappings point to the
intended checkouts, then run the smallest consumer targets and focused tests
covering the coordinated changes. Follow the project's lockfile policy.

### 3. Publish source and register versions

After local validation, publish tested source as immutable revisions or release
archives, dependencies before consumers. Publishing requires authorization.

If the exact version is absent, add its entry following the registry's policy:
version metadata, matching MODULE declarations, retrievable source references,
required integrity hashes, and applicable patches or presubmit checks.
Never publish host-only source paths or rewrite a published version.
Bzlmod resolves dependencies; it does not upload or release Git repositories.

### 4. Validate registration and promote

Select intended release versions in the validation consumer. Disable local
overrides for the entire release dependency set and use the edited registry.
Inspect registry precedence so an earlier registry does not shadow the entry.
Resolve, build, and test through registry-fetched source before submitting it.

After authorized registry publication, confirm the configured remote registry
serves the versions and validate normal consumer operation without local
overrides or the local registry. Restore temporary validation configuration
without reverting pre-existing work; retain intended release version updates.

For reproducible handoffs, export exact vcs revisions after source and registry
commits are published. The export does not capture uncommitted changes.

## Acceptance

- Local: mappings use intended checkouts; relevant builds and tests pass.
- Registry: exact versions pass without source overrides masking dependencies.
- Publication: remote registry consumption passes without development sources.
- Handoff: exact published revisions are recorded.

Report the achieved phase. Local override success or a Git push alone does
not establish release acceptance.

## Failure handling

Stop at the first actionable error. Do not change versions, lockfiles, sources,
cache ownership, or install packages to hide it. Obtain approval before
expanding scope, and preserve unrelated work.
