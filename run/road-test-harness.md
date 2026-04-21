# Road-test harness

This note captures the reusable路测 / 现场测试 workflow under `data/road_tests/`.

## What It Solves

`data/integration_tests/` is the right place for synthetic or scenario-owned inputs such as `sim_control` obstacle files and handcrafted routes. A路测 bag is different: the obstacle, prediction, localization, and chassis inputs already exist in the record, so the harness should replay those inputs directly and only replace the outputs we want to re-evaluate.

The road-test harness therefore keeps the same fixture / probe / assertion mindset, but swaps the fixture source:

1. **Generator**: create a scenario from only `bag + map`.
2. **Fixture**: start current `routing` and `planning`, then replay a recorded bag.
3. **Replay filter**: blacklist the recorded planning and routing topics so stale outputs do not pollute the diagnosis.
4. **Route restoration**: extract the original route intent from the recorded `RoutingResponse` payload and resend it as a fresh `RoutingRequest`.
5. **Probe/report**: reuse the planning subscriber and structured result artifacts to judge whether borrow-lane was observed.
6. **Regression handoff**: once the issue is understood, synthesize a stable `data/integration_tests` scenario from the road-test route and a scenario-local obstacle asset.

## Layout

```text
data/road_tests/
├── run.sh
├── run_in_container.sh
├── scenarios/
│   └── <road-case>/
│       ├── scenario.env
│       ├── meta.yaml
│       ├── dags/
│       └── flags/
├── tools/
│   ├── create_road_test_case.py
│   └── extract_routing_request_from_record.py
├── runs/
├── state/
└── latest/
```

## Standard Flow

From the repository root on the host:

```bash
whl start test

./data/road_tests/run.sh create \
  --record data/bag/20260418074435.record.00000 \
  --map-dir modules/map/data/zt_0.1.7 \
  --require-borrow-enter \
  --force

./data/road_tests/run.sh check road-test-20260418074435-zt-0-1-7
```

Generator behavior:

- normalize host-relative or `/apollo/...` bag/map paths
- derive a scenario id when one is not provided
- generate `scenario.env`, `meta.yaml`, scenario-local routing/planning flags, and DAGs
- point the scenario at the shared route extractor, route sender, and planning subscriber
- default planning subscriber timeout to the observation timeout and reject shorter explicit values so long replay observations keep their late planning events

Runtime behavior:

- replay the original record with `cyber_recorder play`
- blacklist `/apollo/planning`, `/apollo/planning/learning_data`, `/apollo/routing_request`, `/apollo/routing_response`, and `/apollo/routing_response_history`
- extract the latest successful recorded `RoutingRequest` from `RoutingResponseHistory` (fallback: `RoutingResponse`)
- resend that extracted request to the live routing component
- observe `/apollo/planning` with the existing planning probe and emit the same JSON / JUnit artifacts used by integration tests
- clean up partially started routing/planning processes if startup fails before the run fully begins

Regression handoff behavior:

- `./data/road_tests/run.sh create-integration <road-scenario> --obstacle-x ... --obstacle-y ...`
- extract the route from the recorded `RoutingResponse`
- generate a scenario-local synthetic obstacle (`blocking_lane`, `small_car`, `pedestrian`, or `bicycle`)
- write a bag-free integration testcase under `data/integration_tests/scenarios/`

## Current Validated Case

`road-test-20260418074435-zt-0-1-7` replays `data/bag/20260418074435.record.00000` on `/apollo/modules/map/data/zt_0.1.7`.

The latest strict check produced:

- `route_status=ok`
- `borrow_status=absent`
- `collision_status=absent`
- latest lane-borrow reason: `no borrowable neighbor lane`

Using the original recorded routing request in the road-test harness confirms the
routing itself is valid (`routing_failures=0`, `route_status=ok`). The stable root
cause is instead inside `PathLaneBorrowDecider`: once the blocking obstacle becomes
long-term enough, lane-borrow checks are evaluated on `Lane_98` around `s≈52.5-53.5`,
where both sides are blocked by `SOLID_WHITE` boundaries. The planning probe now
reports this directly via `left/right_blocking_lane_id=Lane_98` and
`left/right_blocking_boundary_type=SOLID_WHITE`, so the business failure for the
original road-test case is “current reference-line segment is not borrowable,” not
“record routing is wrong.”

Full-record localization replay confirms this is not a short-window artifact. The
`20260418074435` bag contains about `20.669s` of localization data, and the ego lane
sequence is only:

1. `Lane_89` from replay start until about `10.886s`
2. `Lane_98` from about `10.886s` until replay end

It never reaches `Lane_80` within this record, so even though `Lane_80` has a
borrowable left neighbor later in the map, this specific bag-driven road test never
progresses into that borrowable segment.

The first road-derived regression case is
`borrow-lane-zt-road-202604180744-static-car`, which reuses the extracted route and
replaces recorded obstacles with a scenario-local `small_car` asset at
`(760141.202886593, 3843326.624986003)`. Its current strict result is still
`route-ok-no-borrow`, with the same `no borrowable neighbor lane` planning reason.

Further validation on the same road-derived case found two separate failure modes:

1. Before route normalization, the generated integration testcase reused recorded
   waypoint `id/s` from `Lane_80` but kept `pose` near `Lane_89`. Because
   `sim_control` routing-start uses `waypoint(0).pose()` for spawn, planning ran on
   the `Lane_98`/`Lane_89` segment and `PathLaneBorrowDecider` rejected borrowing due
   to `SOLID_WHITE` boundaries (`left_block/right_block ~= Lane_98:SOLID_WHITE@54m`).
2. After normalizing the route pose back onto `Lane_80`, the testcase stopped
   failing on the wrong lane and instead immediately hit `Found collision with
   obstacle: 1`. For the coordinate `(760134.63, 3843326.58)`, the obstacle sits on
   `Lane_80` at about `s=18.265`, while the normalized start point is at
   `Lane_80 s=17.503`, so the longitudinal gap is only about `0.76m`.
3. The current generator handles this same-lane wraparound case by rebasing the
   replay route to the bag's first localization (`Lane_89 s≈16.244` here), then
   keeping the wrapped `Lane_80` entry/continuation waypoints and appending
   successor-chain continuation waypoints. This removes the raw `pose`/`id/s`
   split and avoids the late `pnc_map` terminal-fragment collapse of the
   byte-for-byte bag request.
4. With that replay-safe route, the road-derived regression recovered the
   expected borrow behavior again: `borrow_enter=1`, `borrow_exit=1`,
   `collisions=0`, `pull_over_errors=0`, and verdict `borrow-completed` in run
   `/apollo/data/integration_tests/runs/borrow-lane-zt-road-202604180744-static-car/20260418T180424Z`.
5. A later harness change added `OBSERVE_FULL_TIMEOUT_ON_SUCCESS=1`, so the same
   integration testcase can keep running after borrow exit instead of stopping at
   first success. Once observation was extended to `180s` and the replay route
   was further lengthened into a looped 32-waypoint continuation, the testcase
   still eventually degraded into `regular/pullover` with repeated
   `PULL_OVER_RETRY_PARKING` and verdict `borrow-completed-then-pull-over`
   (for example run
   `/apollo/data/integration_tests/runs/borrow-lane-zt-road-202604180744-static-car/20260418T183133Z`).
6. This means two different issues were separated by practice:
   - the original borrow-lane failure was a routing semantics problem
     (`pose`/`id,s` split plus same-lane wraparound terminal fragment), which is
     fixed by the replay-safe route builder;
   - the later stop after successful borrow is a different long-horizon mission
     behavior. `ParkDecider` uses
     `routing_request().waypoint().rbegin().pose()` as the destination and keeps
     `PULL_OVER` sticky once entered, so long-running observation scenarios need
     a destination semantic that does not look like a pull-over mission target.
7. Cross-map migration was also revalidated on `zt_0.1.6`. The same bag and
   obstacle coordinates project differently there: the first localization and
   raw routing waypoint poses land directly on `Lane_80` instead of
   `Lane_89/Lane_98/Lane_96`. That is why map-version migration must treat
   target-map localization as authoritative instead of reusing the old lane-id
   semantics verbatim. Using the same `create` + `create-integration` tools with
   `/apollo/modules/map/data/zt_0.1.6` produced:
   - road-test verdict `borrow-completed`
   - integration verdict `borrow-completed`
   - `pull_over_status=absent` even over a 180s observation window

## Why It Stays Separate

This harness is intentionally separate from `data/integration_tests/` because the source of truth is different:

- **integration tests** own their assets and scenario-local synthetic inputs;
- **road tests** depend on external recorded bags and should not pretend the bag is a reusable synthetic asset.

Keeping them separate preserves clean testcase semantics while still allowing both harnesses to share the same probe and reporting tools.
