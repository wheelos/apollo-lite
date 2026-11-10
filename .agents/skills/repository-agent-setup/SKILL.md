---
name: repository-agent-setup
description: Add or evolve AGENTS.md and .agents skills, knowledge, and temporary notes while preserving repository rules and GitHub governance.
---

# Repository Agent Setup

```text
Repository/
  AGENTS.md             # Shared entrypoint and working rules
  .github/              # GitHub governance and provider integration
  .agents/
    skills/             # How to perform a task
  ad_agents/context/    # Existing durable knowledge authority
  Session workspace/    # Temporary task state and evidence
```

## When

Use when adding agent conventions to a repository or normalizing an existing
agent layout. Apply independently to each repository in a multi-repo workspace.

## Prerequisites

Read existing `AGENTS.md`, provider instructions, `.agents/`, relevant source
and CI commands, and Git status. Identify the durable knowledge authority before
editing.

## Steps

1. Keep root `AGENTS.md` short: working rules, verified command entrypoints,
   and links to the skill and knowledge indices. Do not copy architecture here.
2. Keep `.github/` for workflows, contribution policies, issue/PR templates,
   and provider integration. Point provider instructions to the shared agent
   entrypoint; preserve provider-specific rules.
3. Add `.agents/skills/<task>/SKILL.md` with `name` and a searchable
   `description`, then scope, prerequisites, steps, acceptance, and failure
   handling. Include only source-verified commands; keep core steps self-contained.
4. Link the existing durable authority instead of creating `.agents/knowledge/`.
   Keep facts source-backed, narrowly indexed, and separate from procedures.
   Preserve existing ownership; do not move established knowledge silently.
5. Keep temporary investigations in the session workspace. Never store credentials,
   personal data, or raw sensitive logs. Promote reusable conclusions only when
   authorized through the existing context-maintenance process.
6. Merge existing content; add only useful task skills and truthful indices.
   Do not fabricate architecture, commands, empty capability claims, or
   duplicate every skill across repositories.

## Acceptance

- Entry links resolve and every advertised skill exists.
- GitHub governance remains intact; shared rules have one authoritative home.
- Existing knowledge remains reachable with explicit migration boundaries.
- Temporary notes are ignored; reusable guidance is tracked in its owning repo.
- Run the [configuration check](../../../scripts/agents/README.md), review
  entrypoint/command consistency, and compare agent behavior on relevant
  evaluation cases. Syntax checks do not prove instruction effectiveness.
- Report which repositories changed. Documentation-only setup needs no builds.

## Failure handling

Stop for conflicting knowledge ownership, missing command evidence, or local
changes that cannot be merged safely. Ask before moving established knowledge,
changing executable configuration, or committing/pushing repository changes.
