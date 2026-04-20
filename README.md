# Project Context Index

This repository stores durable, reusable project context in `docs/context/`. The goal is to keep important engineering knowledge discoverable, categorized, and maintained beyond a single task or chat.

## Information Architecture

| Directory | Scope | Typical content |
| --- | --- | --- |
| `environment-setup/` | Environment bootstrap | container entry, local dependencies, host setup, toolchain quirks |
| `build/` | Build workflows | build commands, build profiles, packaging notes, artifact locations |
| `run/` | Runtime operations | service startup, restart flows, health checks, debug loops |
| `knowledge/` | Reusable engineering knowledge | architecture findings, subsystem behaviors, validated workflows |
| `anti-patterns/` | Pitfalls to avoid | misleading approaches, fragile workflows, known bad assumptions |

## Index

### environment-setup

- [WHL startup and test-container workflow](environment-setup/whl-startup-and-test-container-workflow.md) - canonical `whl` startup flow and the rule that validation should run in the managed `test` container.

### build

- [Planning, routing, and dreamview build](build/planning-routing-dreamview-build.md) - container-first build flow and the exact cache-stable Bazel command for these modules.

### run

- [Container modes for runtime validation](run/container-modes.md) - why automated validation should use the managed `test` container while build/debug work can stay in `dev`.
- [Scenario integration harness](run/scenario-integration-harness.md) - reusable host/container scenario-test runner under `data/integration_tests/` with centralized PID, run, and log management.
- [Road-test harness](run/road-test-harness.md) - generate and run bag-driven road-test scenarios with recorded planning/routing filtered out and fresh routing injected for live planning diagnosis.
- [Borrow-lane headless reproduction](run/borrow-lane-integration.md) - step-by-step to reproduce borrow-lane and proto-parity troubleshooting.

### knowledge

- [Planning debug signal principles](knowledge/planning-debug-signals.md) - minimal, high-signal rules for planning-topic debug fields consumed by subscribers and scenario probes.
- [Simulation context](knowledge/simulation.md) - backend simulation, routing injection, map selection, and sim control workflow notes.
- [Cross-map route migration](knowledge/cross-map-route-migration.md) - target-map localization should be authoritative when migrating road-derived scenarios across map versions like `zt_0.1.7` and `zt_0.1.6`.
- [Borrow-lane and pull-over interference](knowledge/borrow-lane-pull-over-interference.md) - why long-run borrow-lane regressions can degrade into mission pull-over after returning to `regular/self/`.
- [Borrow-lane obstacle regime and lookahead](knowledge/borrow-lane-obstacle-regime-and-lookahead.md) - how blocking-obstacle promotion, obstacle lateral placement, borrow lookahead, and self-lane candidate generation interact.
- [Local HDMap topology analysis](knowledge/local-hdmap-topology-analysis.md) - how to explain borrow-lane behavior from a small lane neighborhood instead of scanning the whole map.
- [Cyber Python publish and subscribe](knowledge/cyber-python-pubsub.md) - how to initialize the Python runtime, publish messages, and subscribe to planning or localization topics.
- [Logging rules](knowledge/logging-rules.md) - when to use rate-limited Cyber logging macros instead of per-cycle `AINFO`.
- [OSQP API update and matrix construction](knowledge/osqp-api-update.md) - OSQP 1.0 migration findings and Apollo Lite matrix construction behavior.

### anti-patterns

- [Cross-container debugging](anti-patterns/cross-container-debugging.md) - do not probe or switch to other users' dev containers; stay within the current session environment unless explicitly directed otherwise.
- [Validating in the dev container](anti-patterns/validating-in-dev-container.md) - do not let harnesses or manual checks drift back to `apollo_dev_*` when the repo rule is to validate in `apollo_test_*`.

## Authoring Rules

1. Keep one topic per file and use a kebab-case filename.
2. Put the document in the narrowest category that fits; prefer `knowledge/` for stable subsystem conclusions.
3. Update this index and the relevant category `README.md` whenever you add or move a document.
4. Prefer source-backed notes that point to concrete code paths, configs, or commands.
