---
name: interface-migration
description: Evolve Proto, configuration, topic, or Bazel module interfaces across producers and consumers. Use for compatibility changes, deprecations, and coordinated cross-repository migrations.
---

# Interface Migration

## Procedure

1. Inventory the actual contract, direct producers/consumers, serialized data,
   defaults, configuration overlays, and independent checkouts. Read each
   affected repository's rules; use `multi-repo-dev` for dependency publication.
2. Specify old/new semantics and error behavior. Check units, frames, time,
   field presence, enum/default behavior, and persisted-record compatibility
   where applicable. Do not reuse removed Proto field numbers or names.
3. Decide whether coexistence is needed. Prefer an additive migration when
   independently deployed consumers must overlap; do not add indefinite
   compatibility layers when an atomic change is sufficient.
4. Order provider and consumer changes explicitly. Make unsupported versions
   fail visibly; define removal and rollback conditions before deletion.
5. Validate changed producers and consumers against the same source versions,
   plus old/new serialized fixtures when compatibility is claimed. Release
   validation must not be masked by local source overrides.

## Acceptance and failure handling

Report achieved compatibility, affected repositories, migration order, evidence,
and remaining consumers. Local compilation does not prove deployed compatibility.
Ask before changing public semantics, published versions, or dependency state
outside the approved migration; do not silently rewrite existing records.
