# Agent Guide

## Rules

- Follow `.github/copilot-instructions.md`; read relevant source and tests first.
- Keep changes scoped and preserve unrelated work.
- Run builds and lint in the managed container as the mapped non-root user.
  Reuse caches; do not hardcode usernames.
- Stop at the first actionable failure; do not retry blindly.
- Durable repository knowledge lives in `wheelos-service/context/`.
- Each `pkgs/` checkout is independent: read its own `AGENTS.md` and skills.

## Skills

Before executing a matching task, load the full skill through the runtime or
read its `SKILL.md`. Select by `name`/`description`; load only what is needed.

- [Build](.agents/skills/build/SKILL.md)
- [Lint](.agents/skills/lint/SKILL.md)
- [Offline vendor delivery](.agents/skills/vendor/SKILL.md)
- [Multi-repository development](.agents/skills/multi-repo-dev/SKILL.md)
- [Repository Agent setup](.agents/skills/repository-agent-setup/SKILL.md)

## Environment

Enter the development environment: `bash docker/scripts/whl.sh enter dev`.
Build and lint commands belong to their matching skills; host setup and
container lifecycle commands are in
`wheelos-service/context/framework/build/build-and-test-command-registry.md`.
