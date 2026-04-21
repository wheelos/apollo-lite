# Borrow-lane and pull-over interference

Long-running borrow-lane regressions can be dominated by **post-borrow scenario
drift** after the obstacle bypass is already complete. In the road-derived
`borrow-lane-zt-road-202604180744-static-car*` scenarios, the vehicle can
successfully enter and leave lane borrow, then later be taken over by
`PULL_OVER` or `ESCAPE` because mission/end-state deciders keep evaluating
after borrow-lane itself has already succeeded.

## Validated runtime pattern

The strongest symptom chain is:

1. borrow-lane path is observed,
2. planning returns to `regular/self/`,
3. `ParkDecider` switches the scenario to `PULL_OVER`,
4. `PathBoundsDecider` emits `Destination is too close to ADC`,
5. selected self candidate shrinks to about `16m`, speed drops to `0`, and the
   run ends as `borrow-completed-then-pull-over`.

Recent runs that show this pattern:

- `data/integration_tests/runs/borrow-lane-zt-road-202604180744-static-car/20260419T041038Z`
- `data/integration_tests/runs/borrow-lane-zt-road-202604180744-static-car-zt-0-1-6/20260419T040727Z`
- `data/integration_tests/runs/borrow-lane-zt-road-202604180744-static-car-zt-0-1-6/20260419T041722Z`
- `data/integration_tests/runs/borrow-lane-zt-road-202604180744-static-car/20260419T110519Z`

## Why the scenario flips

`ParkDecider::CheckPullOver()` enters `PULL_OVER` when the ADC is on a single
reference line, the destination is on the current reference line, the distance
to destination is within the configured pull-over entry window, the destination
is not too close to
junction overlaps, and the lane is the rightmost driving lane.

Relevant code and config:

- `modules/planning/scenarios/deciders/park_decider.cc`
- `modules/planning/conf/scenario/pull_over_config.pb.txt`

Key thresholds:

- `pull_over_min_distance_buffer = 25.0`
- `start_pull_over_scenario_distance = 50.0`
- `max_distance_stop_search = 25.0`
- effective minimum entry distance is now the max of:
  - `pull_over_min_distance_buffer`
  - `max_distance_stop_search`
  - `front_edge_to_center + s_distance_to_stop_for_open_space_parking + max_valid_stop_distance`

## Why planning then degrades

After the scenario switches, `ScenarioManager::UpdateContextPullOver()` marks
`plan_pull_over_path=true`. That makes `PathBoundsDecider` try to generate a
`regular/pullover` boundary. In `PULL_OVER_APPROACH`, the path-bounds config
requires the routing destination to be at least `25m` ahead of the ADC:

- `pull_over_destination_to_adc_buffer = 25.0`

If the vehicle has already passed the routing endpoint or the endpoint becomes
effectively behind the ADC on the current reference line, the pull-over path
generation fails with:

- `Destination is too close to ADC. distance[...]`

The pull-over scenario is sticky once entered, so the run can remain in
`PULL_OVER_APPROACH` or retry stages even though borrow-lane has already
completed.

Relevant code:

- `modules/planning/scenarios/scenario_manager.cc`
- `modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.cc`
- `modules/planning/scenarios/park/pull_over/stage_approach.cc`

## Probe signals that distinguish this from a pure lane-borrow failure

The planning trajectory-health probe in
`data/integration_tests/tools/subscribe_planning.py` is useful when these runs
look like “stuck near obstacle” from motion alone.

High-signal metrics for this failure mode are:

- `pull_over_after_borrow_messages`
- `borrow_path_to_self_transition_count`
- `borrow_to_self_without_exit_transition_messages`
- `short_selected_candidate_messages`
- `pull_over_scenario_messages`
- anomaly samples showing:
  - `path_label=regular/self/`
  - `scenario_type=PULL_OVER`
  - `stage_type=PULL_OVER_APPROACH` or retry stages
  - `selected_candidate_path_length≈16m`
  - `current_speed_mps=0.0`

This pattern means the vehicle is no longer primarily blocked by the borrow-lane
decision itself; mission-end pull-over semantics have taken control of the
planner.

## Related scenario-switching branch: ESCAPE takeover

Recent `zt_0.1.6` long runs also showed a different post-borrow branch:

- `data/integration_tests/runs/borrow-lane-zt-road-202604180744-static-car-zt-0-1-6/20260419T044826Z`
- `data/integration_tests/runs/borrow-lane-zt-road-202604180744-static-car-zt-0-1-6/20260419T110626Z`

In that run:

- `borrow_status=entered-and-exited`
- `pull_over_status=absent`
- `planning_probe_escape_scenario_messages=1495`
- latest probe event was `scenario_type=ESCAPE`,
  `stage_type=ESCAPE_DEFAULT_STAGE`, `path_label=fallback`

This comes from `EscapeDecider`, which triggers after the vehicle stays nearly
stopped for more than `10s` while not being close to destination and not
waiting for a traffic rule.

Relevant code:

- `modules/planning/scenarios/deciders/escape_decider.cc`

For borrow-lane regression interpretation, this means a long run can still
“pass” the narrow borrow criterion while the planner has already drifted into a
different post-borrow scenario. If the goal is to validate stable recovery back
to normal cruising, both `PULL_OVER` and `ESCAPE` should be treated as scenario
interference unless the test explicitly expects them.

## Current test-suite consistency

With the lane-borrow scan horizon capped at the blocking obstacle's current `s`,
the two long `202604180744` road-derived scenarios again match their earlier
borrow behavior:

- `zt_0.1.7` static-car scenario: borrow completes, then can still drift into
  `PULL_OVER`
- `zt_0.1.6` static-car scenario: borrow completes, then can still drift into
  `ESCAPE`

So the current inconsistency is no longer “borrow never entered”; it is the
already-known **post-borrow scenario interference** branch.
