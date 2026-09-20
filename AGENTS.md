# Agent Guide

## Commands

- Prepare the host: `sudo bash docker/setup_host/setup_host.sh`
- Start the dev container: `bash docker/scripts/whl.sh start dev`
- Enter the dev container: `bash docker/scripts/whl.sh enter dev`
- Build all modules: `bash apollo.sh build`
- Build one module: `bash apollo.sh build <module>`
- Run lint: `bash apollo.sh lint`

## Principles & Anti-Patterns

- **DO**: Read relevant source and tests first; follow existing Bazel targets,
  cache, and user-ownership conventions.
- **DO**: Read `.agents/skills/build/SKILL.md` before compiling.
- **DO NOT**: Compile Apollo targets outside the managed container, run Bazel as
  root, hardcode a container username, or change unrelated files.
- **DO NOT**: Disable caches or retry blindly without locating the first error.

## Skills

- `.agents/skills/build/SKILL.md` — Apollo-Lite build and compile workflow.
