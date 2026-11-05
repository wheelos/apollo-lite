---
name: unified-core-runtime
description: Assemble source-built modules into one Cyber runtime with shared tools and configuration; validate without operating devices.
---

# Unified core runtime implementation

## Agreed principles

- Bzlmod manages source dependencies. One final integration root resolves core,
  messages, tools, hardware and algorithm modules with the same toolchain.
- Install one core tool set: mainboard, cyber_launch, cyber_recorder and
  cyber_monitor. Modules contribute dynamically loaded components, not another
  middleware/tool installation.
- All processes use one authoritative Cyber configuration root and explicitly
  coordinated domain/network/clock settings. Process-specific scheduler files
  may differ within that root.
- Device, vehicle and algorithm configuration belongs to its module; it must
  not replace the common middleware configuration.
- Repository boundaries do not force process boundaries. Choose one or several
  mainboards according to execution and failure isolation requirements.
- Full-source integration is the current scope. Do not introduce binary ABI
  release frameworks, separate per-module Cyber roots, or standalone/managed
  delivery modes as prerequisites.
- Install declared files and dependency runfiles; never infer dependencies by
  scanning host libraries or concatenating separate product environments.
- Module selection/configuration must be explicit. Installation never starts
  devices. Recorder/monitor retain caller-relative data paths; recordings and
  logs are outside the installation.

## Implementation procedure

1. Read the
   [Core runtime contract](../../../pkgs/core/runtime/README.md), public
   `pkgs/core/runtime` rules, hardware compatibility exports, module targets
   and relevant configuration.
2. Declare module contributions with core's `runtime_module`/`RuntimeModuleInfo`:
   components, explicit component/runtime resource paths, dependency metadata,
   business configuration, DAG/launch and resources. Infra repositories export
   their `products/*:module` targets.
3. Compose those contributions in a final Bzlmod integration root, such as
   `tools/system`, adding core tools/config exactly once. Reject module-owned
   middleware/tool destinations and conflicts. The common runtime namespace is
   `conf/`, `dag/`, `launch/`, `share/`, `lib/components/`, and core-owned `bin/`;
   never derive deployment paths from repository names or Bazel output roots.
4. Reuse core's runtime layout, wrappers, runfiles and rules_pkg delivery.
   Keep source, archive and installed layouts equivalent and relocatable.
5. Validate inside the managed container as the mapped non-root user. Use
   `--config=dev` for approved source-checkout validation; do not rewrite the
   release lock or publish local paths. Unpublished APIs/modules require
   explicit source overrides; local success is not registry release validation.
6. Exercise FakeCAN and synthetic sensor modules together through one runtime:
   common configuration, pub/sub, one recorder capturing both schemas, monitor,
   independent shutdown, archive/install relocation and missing-input errors.

## Acceptance and boundaries

No physical CAN, camera or LiDAR is initialized during automated validation.
Synthetic fixtures are private/test-only and absent from production packages.
Local build/runtime success is not remote publication, hardware acceptance or
permission to actuate. Record failures and evidence outside this skill.

This skill is the requested design/implementation entry under `.agents`.
Long-term source-backed runtime details remain in the canonical context
document rather than a second knowledge tree.
