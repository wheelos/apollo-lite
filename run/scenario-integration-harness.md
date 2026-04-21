# Scenario Integration Harness

This note captures the reusable headless scenario-test framework under `data/integration_tests/`.

## What It Solves

The original borrow-lane validation flow could run end to end, but it mixed fixture setup, topic observation, scenario logic, and pass/fail heuristics inside bash manifests and grep rules. The current framework keeps the proven host/container execution path while reorganizing it around a gtest-style mental model:

1. **Fixture**: start and stop the scenario deterministically.
2. **Probe**: subscribe to runtime topics with reusable tools.
3. **Assertion**: evaluate business outcomes from logs and probe summaries.
4. **Report**: emit machine-readable artifacts for CI and AI triage.

## Layout

```text
data/integration_tests/
├── run.sh
├── run_in_container.sh
├── scenarios/
│   ├── borrow-lane/
│   │   ├── scenario.env
│   │   ├── meta.yaml
│   │   ├── assets/
│   │   ├── dags/
│   │   ├── flags/
│   │   └── routes/
│   └── borrow-lane-wide/
│       ├── scenario.env
│       ├── meta.yaml
│       └── flags/
├── tools/
│   ├── routing_request_sender.py
│   ├── subscribe_planning.py
│   ├── subscribe_sim_control_status.py
│   ├── scenario_catalog.py
│   ├── scenario_report.py
│   └── gen_blocking_obstacle.py
├── runs/
├── state/
└── latest/
```

## Commands

From the repository root on the host:

```bash
./data/integration_tests/run.sh list
./data/integration_tests/run.sh catalog
./data/integration_tests/run.sh describe borrow-lane
./data/integration_tests/run.sh run borrow-lane
./data/integration_tests/run.sh check borrow-lane
./data/integration_tests/run.sh run borrow-lane-wide
./data/integration_tests/run.sh start borrow-lane
./data/integration_tests/run.sh send-route borrow-lane
./data/integration_tests/run.sh summary borrow-lane
./data/integration_tests/run.sh stop borrow-lane
```

Behavior:

1. `run` and `check` do a clean restart, start both planning and sim-control probes, send the route, wait for the scenario outcome, emit structured results, and stop the scenario automatically.
2. `catalog` prints a JSON catalog of all scenarios, including suite/case identity and key fixture paths.
3. `describe <scenario>` prints one scenario as structured JSON, which is useful for tooling and AI-driven diagnosis.
4. `start/send-route/summary/stop` keeps the fast manual-debug loop when repeated route injections are needed.

Startup failure paths are also part of the fixture contract: if routing,
planning, or sim-control dies before startup completes, the harness now cleans
up any partially started processes instead of leaving stale PIDs behind.

## Scenario Contract

Each `scenario.env` is the executable fixture description. It defines:

- process names and DAGs
- standalone `sim_control` binary and flagfile
- route sender and probe commands
- gtest-like identity (`TEST_SUITE`, `TEST_NAME`, `TEST_TAGS`, `TEST_FIXTURE`)
- business-level expectations (`SUMMARY_REQUIRE_BORROW_ENTER`, `SUMMARY_FAIL_ON_COLLISION`, and so on)

Each `meta.yaml` is the durable, human-maintained summary of the scenario. This separation lets the harness source shell-native manifests while tools and reviewers can still reason about the scenario at a higher level.

When a scenario is meant to observe long-horizon behavior, its planning probe
must stay alive for the full observation window. Generated road-derived
integration scenarios now default the planning subscriber timeout to the same
value as `OBSERVATION_TIMEOUT_SECS` and reject shorter explicit values.

Road-derived regressions should enter here only after a road-test case has isolated the
business condition. The preferred workflow is:

1. reproduce and diagnose with `data/road_tests/`,
2. extract the recorded route intent from the bag,
3. replace recorded obstacles with a scenario-local synthetic obstacle asset, and
4. commit the resulting bag-free scenario under `data/integration_tests/scenarios/`.

When the generated integration scenario uses `sim_control_spawn_mode=routing_start`,
the `RoutingRequest` waypoint `pose` must be normalized to match its `lane id + s`.
In the `20260418074435` road-derived case, the recorded `RoutingRequest` carried
`Lane_80` waypoint ids with `pose` coordinates near `Lane_89`; if reused verbatim,
`sim_control` spawns from `waypoint(0).pose()` and the testcase diagnoses the wrong
reference-line segment.

For the same `20260418074435` road-derived regression, there is also a useful
comparison mode where the raw extracted `RoutingRequest` is kept unchanged and
only the synthetic obstacle is moved. A `0/2/4/6/8/10/12m` sweep of the parked
car along `Lane_80` kept `route_status=ok`, `collisions=0`, and
`borrow_status=absent` for every run. The stable planning reason stayed
`no borrowable neighbor lane`; the blocking metadata reported
`Lane_86:SOLID_WHITE` on the left and `Lane_80:SOLID_WHITE` on the right for
`0-8m`, then converged to `Lane_98:SOLID_WHITE` on both sides for `10-12m`.
This is a good sanity check when the question is “does obstacle longitudinal
distance alone unlock borrow-lane for this raw route?” rather than “is the
route pose normalized for routing-start spawn?”

## Probe and Report Artifacts

Every run directory now contains both raw logs and structured artifacts:

- `subscribe_planning.log`
- `subscribe_sim_control.log`
- `probes/planning.events.jsonl`
- `probes/planning.summary.json`
- `probes/sim_control.events.jsonl`
- `probes/sim_control.summary.json`
- `result.summary.kv`
- `result.json`
- `result.junit.xml`

The planning probe summarizes lane-borrow state transitions from `/apollo/planning`, the sim-control probe records spawn/prediction/runtime status from `/apollo/sim_control/status`, and the result reporter combines those probe outputs with routing/planning logs into a deterministic testcase result. This is the critical shift from “grep some logs” to “evaluate a scenario testcase.”

## Why This Fits Future Scenarios

This structure scales better to parking and other future scenarios because it separates what changes often from what should stay reusable:

- **Scenario-specific**: maps, routes, flags, obstacle assets, expected assertions.
- **Framework-level**: process lifecycle, topic probes, result emission, and scenario discovery.
- **AI/CI-facing**: scenario catalog, JSON summaries, JUnit XML, and stable run artifacts.

That makes it practical to treat each scenario as a typed testcase rather than a one-off shell script.
