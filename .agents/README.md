# Agent resources

Repository-wide rules live in [AGENTS.md](../AGENTS.md). This directory contains
task procedures and workflow specifications used by compatible coding agents.

- [Skill index](skills/README.md) routes engineering tasks to focused procedures.
- [Workflow specifications](workflows/README.md) define multi-stage delivery and
  evidence gates.
- Durable repository facts and contracts remain in the internal
  `ad_agents/context/` checkout. It is intentionally ignored by this repository
  and is not duplicated here.

Load only the skill or workflow needed for the current task. A workflow does not
grant source-edit, build, hardware, publication or deployment authority.
