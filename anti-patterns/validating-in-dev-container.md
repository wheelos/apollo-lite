# Validating in the dev container

Do **not** treat `apollo_dev_*` as the default target for automated validation
now that the workspace has a managed `test` mode.

## Why this is a problem

- `dev` is where interactive debugging, ad hoc rebuilds, and privileged runtime
  experiments accumulate.
- Scenario and road-test harnesses now resolve `apollo_test_*` by default, so
  manually steering checks back into `dev` breaks the repository's intended
  isolation boundary.
- Old examples like `APOLLO_TEST_CONTAINER=apollo_dev_wfh ...` are stale and can
  quietly make two engineers compare results from different runtime conditions.

## Preferred alternative

```bash
whl start test
./data/integration_tests/run.sh check borrow-lane
./data/road_tests/run.sh check road-test-20260418074435-zt-0-1-7
```

Only override `APOLLO_TEST_CONTAINER` when you need a specific managed
`apollo_test_*` instance.

## Source Pointers

- `data/integration_tests/run.sh`
- `data/road_tests/run.sh`
- `docker/scripts/whl.sh`
- `docs/context/run/container-modes.md`
