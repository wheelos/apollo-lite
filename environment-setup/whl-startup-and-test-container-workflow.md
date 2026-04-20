# WHL startup and test-container workflow

Use `whl` as the canonical way to start Apollo Lite containers, and treat
`test` as the default validation runtime.

## Recommended flow

From the repository root on the host:

```bash
whl start test
whl enter test
```

Use `whl start dev` / `whl enter dev` only when you need an interactive
build/debug environment with host networking, host PID access, or privileged
device access.

## Why this is the stable workflow

1. `whl` now generates mode-specific env files (`docker/.env.dev.local`,
   `docker/.env.test.local`, `docker/.env.prod.local`) from `.env.global` plus
   host detection.
2. `dev` and `test` get distinct Compose project identities and deterministic
   container names (`apollo_<mode>_<user>_<project-hash>`), so they can coexist
   without recreating each other.
3. The host-side test harnesses now resolve `apollo_test_*` by default, which
   makes `whl start test` the aligned entry point for integration and road-test
   validation.

## Operational rule

- **Validation**: run from `test`.
- **Interactive build/debug**: use `dev` when you need its extra privileges.
- **Explicit overrides**: only set `APOLLO_TEST_CONTAINER` when you truly need a
  non-default test target.

## Source Pointers

- `docker/scripts/whl.sh`
- `docker/scripts/env_setup.sh`
- `docker/services/docker-compose.dev.yml`
- `docker/services/docker-compose.test.yml`
- `data/integration_tests/run.sh`
- `data/road_tests/run.sh`
