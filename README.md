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

### Purpose
- Ensure host is prepared (Docker, NVIDIA runtime, OS tuning) and provide a simple workflow to start the Apollo container using the `whl` helper.

### Summary (recommended flow)
1. Run the host setup script (interactive): installs Docker, NVIDIA toolkit and then runs system configuration steps interactively.

```bash
sudo bash docker/setup_host/setup_host.sh
```

- `setup_host.sh` will install `whl` (system command linked at `/usr/local/bin/whl`) before running system configuration.
- After completing, the installer writes `/etc/wheelos_setup_host.done` to indicate host readiness.

2. Start or enter the container with `whl`:

```bash
# start in dev mode
whl start

# enter the dev container (starts it if needed)
whl enter
```

### Notes about interactive system configuration
- `config_system.sh` (invoked by `setup_host.sh`) is interactive: for each optional system tuning step (NTP/ptp/udev/uvcvideo/CAN/Jetson tuning/headless/autostart) it will ask you whether to apply it (Y/n). Hardware-related tuning defaults to conservative choices.

### Automation / non-interactive runs
- `config_system.sh` and `setup_host.sh` detect non-interactive stdin and use sensible defaults. To run unattended and accept defaults, redirect stdin from `/dev/null`:

```bash
sudo bash docker/setup_host/setup_host.sh < /dev/null
```

- If you need to fully automate and explicitly choose Yes/No for every prompt, use an automation tool or supply answers via stdin (careful: using `yes` will force all answers to `y`). Example (force yes for all prompts):

```bash
yes | sudo bash docker/setup_host/setup_host.sh
```

### Environment and files
- User-maintained overrides live in the project-root `.env.global`.
- Use mode-scoped keys such as `DEV_USE_GPU`, `DEV_BAZEL_CACHE_DIR`, `TEST_SERVER_PORT`, `TEST_CPUS`, `TEST_MEMORY`, `TEST_USE_GPU`, and `TEST_BAZEL_CACHE_DIR` so dev and test stay isolated and explicit.
- Generated env files used by `whl` are `docker/.env.dev.local`, `docker/.env.test.local`, and `docker/.env.prod.local`. `whl` regenerates the requested mode file on every run from `.env.global` plus host auto-detection before launching.
- Each mode also gets its own Compose project name, so `dev` and `test` can run side by side without recreating or stopping each other.
- Container names are deterministic (`apollo_<mode>_<user>_<project-hash>`) and are intentionally not user-configurable.
- `whl start <mode>` and `whl stop <mode>` are symmetric. Use `whl stop all` to tear down all managed modes.
- Host-ready marker: `/etc/wheelos_setup_host.done`
- `whl` helper location: `/usr/local/bin/whl`

---

## Copyright and License

Apollo-Lite is licensed under the [Apache License 2.0](LICENSE). Please comply
with the license terms when using or contributing to this project.

---

## Connect with Us

- ⭐ Star and Fork to support the project!
- 💬 Join our [community discussion group](http://apollo.auto/community) to chat
  with developers.
- 📧 For collaboration or business inquiries, contact: daohu527@gmail.com

---

Thank you for being part of Apollo-Lite's journey towards autonomous driving
innovation!
