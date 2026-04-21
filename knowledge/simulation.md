# Simulation Context

This note captures reusable knowledge for Apollo Lite simulation workflows, especially around `sim_control`, routing injection, and map selection.

## Key Findings

1. **Map selection is flag-driven.** The default map comes from `modules/common/data/global_flagfile.txt`, where `--map_dir=/apollo/modules/map/data/borregas_ave` is set, and that file is pulled in by runtime configs such as `modules/dreamview/conf/dreamview.conf`.
2. **Scenario-specific configs can override the default map.** Flags are evaluated in order, so a config that includes the global flagfile and then sets `--map_dir=...` later will win, as shown in `modules/dreamview/conf/sim_control_borrow_lane.conf`.
3. **Dreamview constructs `SimControlManager` at startup but does not automatically start simulation.** The manager is created in `Dreamview::Init()`, then started or stopped later through `ToggleSimControl` and restarted through the HMI callback path.
4. **Standalone backend simulation is supported.** `modules/dreamview/backend/sim_control_manager/BUILD` builds a `sim_control` binary, and `main.cc` starts `SimControlManager` directly for headless harness use; Dreamview still keeps the direct `Restart(x, y)` path for HMI-driven restarts.
5. **Custom obstacle playback should be configured per scenario.** Scenario-local sim-control flagfiles under `data/integration_tests/scenarios/*/flags/` enable `--enable_sim_control_custom_prediction=true` and point at scenario-owned obstacle assets instead of relying on a shared global sim-control config.
6. **Routing can be injected without Dreamview UI.** `modules/tools/whl-mock/send_routing.py` can load a `RoutingRequest` textproto, prepend the current localization as waypoint 0, and run in single-shot or loop mode.
7. **Routing injection needs service-discovery settling time.** `send_routing.py` intentionally waits before the first publish so subscribers discover the writer; skipping that wait makes first-send failures more likely.
8. **Latest routing results are republished on a history topic.** `RoutingComponent` republishes the most recent `RoutingResponse` to `/apollo/routing_response_history` using transient-local QoS every `1000 ms` by default.
9. **Planning message debugging is practical with Python subscribers.** After `source cyber/setup.bash`, Python tools can subscribe to `/apollo/planning` and read `debug.planning_data.lane_borrow` instead of relying only on textual logs.
10. **Borrow-lane sim control should use the borrow-lane obstacle file.** `modules/dreamview/conf/sim_control_borrow_lane.conf` should point at `modules/tools/whl-mock/blocking_obstacle_sm_borrow_lane.txt`, not the generic blocking obstacle file.
11. **Temporary mainboard dags need sibling config files.** When you run `mainboard -d /tmp/.../foo.dag`, mainboard also looks for a sibling `/tmp/.../foo.config.pb.txt`; copying only the dag is not enough.
12. **Routing and planning can silently override CLI map flags through their DAG flag files.** `ComponentBase::LoadConfigFiles()` calls `google::SetCommandLineOption("flagfile", ...)` from each DAG component config, so starting mainboard with `--map_dir=...` is not sufficient when the referenced `routing.conf` or `planning.conf` later re-import `modules/common/data/global_flagfile.txt`.
13. **Borrow-lane routing/planning now use scenario-specific flag files and DAGs.** `modules/routing/conf/routing_borrow_lane.conf` and `modules/planning/conf/planning_borrow_lane.conf` re-include the global flagfile and then override `--map_dir=/apollo/modules/map/data/san_mateo`; `routing_borrow_lane.dag` and `planning_borrow_lane.dag` point at those files so the override survives component initialization.
14. **Borrow-lane route definition must stay on the connected successor chain.** The canonical `modules/routing/conf/routing_request_borrow_lane.pb.txt` destination is now `x=559733.677883, y=4157326.669489, heading=2.294894` on lane `909_1_-3`; the previous far endpoint landed on `1266a_1_-1`, which is not connected from the start lane `243_1_-3` and caused routing failure.
15. **Borrow-lane tuning uses a slower planning config.** `modules/planning/conf/planning_borrow_lane.conf` lowers `--planning_upper_speed_limit` to `8.0` and `--default_cruise_speed` to `6.0` to make obstacle-avoidance validation easier to observe.
16. **Fresh retests now prove the borrow-lane maneuver itself works.** With the corrected destination and a clean restart of `routing`, `planning`, and standalone `sim_control`, routing succeeds from `243_1_-3` through `275_1_-3` to `909_1_-3`, planning switches from `SELF-LANE` to `LANE-BORROW` and later back to `SELF-LANE`, and no `Found collision with obstacle` lines appear; the remaining failure is a later terminal `PULL_OVER_RETRY_PARKING` issue near destination, not the borrow-lane obstacle-avoidance path itself.
17. **Headless AI scenarios should use routing-owned ego initialization.** The stable borrow-lane harness now sets `--sim_control_spawn_mode=routing_start` and sends a two-waypoint `RoutingRequest` whose first waypoint is the scenario start pose; this removes the need for `sim_control_start_x/y` gflags in scenario automation while preserving Dreamview's separate coordinate restart path.
18. **`sim_control` now exposes explicit spawn and prediction modes.** `sim_control_spawn_mode` makes `legacy`, `localization_start`, and `routing_start` selectable from flags, while direct `Start(x, y)` calls still report an explicit-start runtime status; `sim_control_prediction_mode` makes `legacy`, `empty`, `custom_file`, and `external_passthrough` explicit instead of relying on hidden coupling.
19. **`sim_control` now publishes structured runtime status.** `SimPerfectControl` writes `apollo.sim_control.SimControlStatus` on `/apollo/sim_control/status`, and the integration harness records it as `probes/sim_control.summary.json` so AI tooling can tell whether the ego start came from routing, localization, or a direct coordinate restart and which prediction mode was active.

## Recommended Workflow

For backend-focused simulation or repeatable scenario tests:

1. Choose the target map by overriding `--map_dir` after any `--flagfile=...` include when the default map is not the right one.
2. Start simulation through Dreamview control paths or run the standalone `sim_control` binary if UI coupling is unnecessary.
3. For borrow-lane headless tests, prefer `whl start test` followed by `./data/integration_tests/run.sh run borrow-lane` from the host; the harness now auto-selects the running `apollo_test_*` container that contains the scenario tools, falls back to `root` if the preferred container user is absent, sources `cyber/setup.bash`, manages exact PIDs, and archives the run under `data/integration_tests/runs/`.
4. `./data/integration_tests/run.sh check borrow-lane` is the current stable borrow-maneuver regression: it uses the narrow obstacle variant, requires routing success plus borrow enter/exit, and intentionally ignores the still-open destination-stage pull-over failure.
5. `./data/integration_tests/run.sh run borrow-lane-wide` keeps the wider obstacle available for A/B comparison under the same route and process topology.
6. If you need the raw building blocks, start `routing_borrow_lane.launch` and `planning_borrow_lane.launch` instead of the default launch files so routing/planning stay on `san_mateo`.
7. For AI/headless scenarios, send a two-waypoint `RoutingRequest` whose first waypoint is the intended spawn pose and pair it with `--sim_control_spawn_mode=routing_start`; avoid mixing that mode with `--use-localization-start`, because the route sender can otherwise capture a pre-routing placeholder localization.
8. Source `cyber/setup.bash` before using Python Cyber tools so protobuf modules and `_cyber_wrapper.so` resolve correctly.
9. Subscribe to `/apollo/planning` with a Python tool when you need lane-borrow and scenario debug fields from `ADCTrajectory.debug.planning_data`; if the debug field stays absent, planning logs may still show `path_lane_borrow_decider.cc` transitions.
10. Observe `/apollo/sim_control/status` if you need structured spawn source, route-start acceptance, prediction mode, and resolved ego pose during backend simulation.
11. Observe `/apollo/routing_response_history` if you need a stable stream of the latest routing output rather than only edge-triggered responses.

## Source Pointers

- `modules/common/data/global_flagfile.txt`
- `modules/dreamview/conf/dreamview.conf`
- `modules/dreamview/conf/sim_control_borrow_lane.conf`
- `cyber/component/component_base.h`
- `cyber/mainboard`
- `modules/dreamview/backend/dreamview.cc`
- `modules/dreamview/backend/simulation_world/simulation_world_updater.cc`
- `modules/dreamview/backend/sim_control_manager/BUILD`
- `modules/dreamview/backend/sim_control_manager/main.cc`
- `modules/dreamview/backend/sim_control_manager/proto/sim_control_internal.proto`
- `modules/dreamview/backend/sim_control_manager/common/sim_control_gflags.cc`
- `modules/dreamview/backend/sim_control_manager/dynamic_model/perfect_control/sim_perfect_control.cc`
- `modules/planning/conf/planning_borrow_lane.conf`
- `modules/planning/dag/planning_borrow_lane.dag`
- `modules/routing/conf/routing_borrow_lane.conf`
- `modules/routing/conf/routing_request_borrow_lane.pb.txt`
- `modules/routing/dag/routing_borrow_lane.dag`
- `modules/routing/tools/routing_request_sender.py`
- `modules/tools/whl-mock/send_routing.py`
- `modules/routing/routing_component.cc`
- `modules/routing/common/routing_gflags.cc`
- `data/integration_tests/tools/subscribe_sim_control_status.py`
