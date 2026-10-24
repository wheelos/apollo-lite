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
    knowledge/          # Durable, verified repository facts
    notes/              # Ignored temporary investigations
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
4. Add `.agents/knowledge/README.md` indexing narrow, source-backed topics.
   Record facts, boundaries, evidence, and update triggers rather than transcripts.
   If a durable authority already exists, link it instead of duplicating facts.
   Migrate legacy topics individually and leave compatibility pointers; do not
   silently create competing sources of truth.
5. Add `.agents/notes/README.md` explaining temporary investigations. Ignore
   note contents by default; never store credentials, personal data, or raw
   sensitive logs. Promote validated reusable conclusions into knowledge or a
   skill, then remove obsolete notes.
6. Merge existing content; add only useful task skills and truthful indices.
   Do not fabricate architecture, commands, empty capability claims, or
   duplicate every skill across repositories.

## Acceptance

- Entry links resolve and every advertised skill exists.
- GitHub governance remains intact; shared rules have one authoritative home.
- Existing knowledge remains reachable with explicit migration boundaries.
- Temporary notes are ignored; reusable guidance is tracked in its owning repo.
- Report which repositories changed. Documentation-only setup needs no builds.

## Failure handling

Stop for conflicting knowledge ownership, missing command evidence, or local
changes that cannot be merged safely. Ask before moving established knowledge,
changing executable configuration, or committing/pushing repository changes.
