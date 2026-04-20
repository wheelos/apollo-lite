# Container Modes for Runtime Validation

Apollo Lite's `dev` and `test` modes now have distinct roles. They are still
not interchangeable, but the repository's validation rule has changed.

## Recommended Rule

Use the managed **`test` container** for repeatable validation work:

- `./data/integration_tests/run.sh ...`
- `./data/road_tests/run.sh ...`
- headless routing / planning / sim_control scenario checks
- any result you want to treat as a regression signal

Keep **`dev`** for interactive build/debug tasks that benefit from host
networking, host PID access, or privileged device access.

## Why

- `test` mode is the isolation boundary that `whl` now manages explicitly:
  deterministic `apollo_test_*` names, separate env generation, separate Compose
  project, mode-scoped resource limits, and dynamic Dreamview port mapping.
- The harness scripts are now aligned with that contract and auto-select the
  running `apollo_test_*` container by default.
- Using `dev` for automated validation makes results less reproducible because it
  mixes privileged runtime state, ad hoc debugging changes, and user-specific
  long-lived processes with the thing being measured.

## Practical startup

From the repository root on the host:

```bash
whl start test
./data/integration_tests/run.sh check borrow-lane
./data/road_tests/run.sh check road-test-20260418074435-zt-0-1-7
```

If you need an explicit target, set `APOLLO_TEST_CONTAINER` to a managed
`apollo_test_*` container.

## Related anti-pattern

See `docs/context/anti-patterns/validating-in-dev-container.md`.

## Source Pointers

- `docker/scripts/whl.sh`
- `docker/scripts/env_setup.sh`
- `docker/services/docker-compose.dev.yml`
- `docker/services/docker-compose.test.yml`
- `data/integration_tests/run.sh`
- `data/road_tests/run.sh`
