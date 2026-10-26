# Agent Guide

## Rules

- Follow `.github/copilot-instructions.md`; read relevant source and tests first.
- Keep changes scoped and preserve unrelated work.
- Run builds and lint in the managed container as the mapped non-root user.
  Reuse caches; do not hardcode usernames.
- Stop the failing action at the first actionable error; diagnose it, then
  recover safely in scope or offer the user options before consequential
  alternatives. Do not retry blindly. See
  `ad_agents/context/framework/validation/failure-recovery-and-user-choice.md`.
- Durable repository knowledge lives in `ad_agents/context/`.
- Each `pkgs/` checkout is independent: read its own `AGENTS.md` and skills.

## Skills

Before executing a matching task, load the full skill through the runtime or
read its `SKILL.md`. Select by `name`/`description`; load only what is needed.

- [Build](.agents/skills/build/SKILL.md)
- [Container image build](.agents/skills/container-build/SKILL.md)
- [Lint](.agents/skills/lint/SKILL.md)
- [Offline vendor delivery](.agents/skills/vendor/SKILL.md)
- [Multi-repository development](.agents/skills/multi-repo-dev/SKILL.md)
- [Repository Agent setup](.agents/skills/repository-agent-setup/SKILL.md)
- [Engineering solution design](.agents/skills/engineering-solution-design/SKILL.md)
- [Algorithm development](.agents/skills/algorithm-development/SKILL.md)
- [Performance optimization](.agents/skills/performance-optimization/SKILL.md)
- [Issue localization](.agents/skills/issue-localization/SKILL.md)
- [Simulation regression](.agents/skills/simulation-regression/SKILL.md)

Use the [skill index](.agents/skills/README.md) to route engineering tasks.

## Environment

Enter the development environment: `bash docker/scripts/whl.sh enter dev`.
Build and lint commands belong to their matching skills; host setup and
container lifecycle commands are in
`ad_agents/context/framework/build/build-and-test-command-registry.md`.
