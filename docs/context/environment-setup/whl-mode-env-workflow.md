# WHL Mode-Scoped Environment Workflow

Apollo Lite's `docker/scripts/whl.sh` uses a mode-scoped configuration model designed for deterministic local development and test isolation.

## Supported Instance Model

- One **`dev`** container per repository.
- One **`test`** container per repository.
- `dev` and `test` may run at the same time in the same repository.
- Multiple repositories may each run their own `dev` and `test` containers concurrently.

This workflow does **not** treat "multiple `dev` instances in the same repository" or "multiple `test` instances in the same repository" as first-class use cases. The generated runtime env file is mode-scoped (`docker/.env.dev.local`, `docker/.env.test.local`), so same-mode multi-instance orchestration inside one repository is intentionally out of scope.

## User Configuration vs Generated Runtime State

### User-maintained input: `.env.global`

Only keep values here that the user intentionally chooses, typically:

- `DEV_USE_GPU`
- `DEV_BAZEL_CACHE_DIR`
- `DEV_SHM_SIZE`
- `TEST_SERVER_PORT`
- `TEST_USE_GPU`
- `TEST_CPUS`
- `TEST_MEMORY`
- `TEST_BAZEL_CACHE_DIR`
- `TEST_SHM_SIZE`

Avoid placing auto-detected or derived values here.

### Generated runtime output: `docker/.env.<mode>.local`

`whl.sh` regenerates these files on every `start`, `enter`, `status`, `update`, and `stop` operation for the requested mode. They are not user configuration files.

Typical generated values include:

- `APOLLO_ROOT`
- `APOLLO_IMAGE`
- `USER_NAME`, `USER_ID`, `GROUP_ID`
- `TARGET_ARCH`
- `TZ`
- `DISPLAY`
- `USE_GPU_HOST`
- `SERVER_PORT`, `DREAMVIEW_PORT`
- `RUNTIME_ENV_FILE`

`test` mode also receives `TEST_CPUS` and `TEST_MEMORY` because those are runtime inputs used by the test compose overlay.

## Deterministic Defaults

When the user does not set an override in `.env.global`, `whl` uses deterministic defaults:

- Container name: `apollo_<mode>_<user>_<project-hash>`
- Bazel cache dir: `${PROJECT_ROOT}/.cache/bazel/<mode>/repo_cache`
- Dreamview/test port: calculated from user ID, project hash, and optional `WHL_PORT_OFFSET`
- GPU enablement: auto-detected from host GPU availability

The deterministic container-name rule is intentional. User-configurable container names were removed because they add surface area without improving the main supported workflow, and they make later `enter/status/stop` calls less predictable.

## Isolation Guarantees

Isolation is achieved by combining:

1. Repository path hash
2. Mode (`dev` / `test` / `prod`)
3. Mode-scoped runtime env files
4. Mode-scoped Bazel cache defaults
5. Mode-scoped Compose project names

As a result:

- `whl start dev` does not recreate `test`
- `whl stop test` does not stop `dev`
- `whl enter dev` and `whl enter test` resolve the correct mode-local container for the current repository

## Operational Guidance

- Prefer the default deterministic container names.
- Override cache directories only when you truly need custom storage placement.
- Treat `docker/.env.<mode>.local` as disposable generated state.
- If you need multiple same-mode test jobs in one repository, extend the design with an explicit instance identifier rather than overloading the current mode-scoped workflow.
