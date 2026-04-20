# Borrow-lane obstacle regime and lookahead

## Summary

- `front_static_obstacle_id` is a **path-blocking** result, not a raw
  perception-type result. A static obstacle tagged as
  `UNKNOWN_UNMOVABLE` still will not become the lane-borrow anchor unless
  `PathBoundsDecider` and `PathAssessmentDecider` promote it into the selected
  blocking obstacle.
- Narrow obstacles that sit near one side of the lane can stay in a
  nudge / not-side-passable regime. In the `20260419074214` road-derived case on
  `zt_0.1.7`, the original left-shifted `Lane_79` obstacle never produced a
  stable `front_static_obstacle_id`, while a same-`s` obstacle shifted toward
  lane center triggered borrow entry and exit successfully.
- Lane-borrow boundary scanning should be bounded by the obstacle region that
  actually matters for the pass. Scanning far into successor lanes can reject a
  locally valid side-pass because of unrelated future solid boundaries. For the
  current regression suite, the best-matching horizon is the blocking
  obstacle's current `s` (distance to the obstacle only), not the legacy fixed
  `100m` scan and not `blocking_obstacle.end_s + return_distance`.
- During an active borrow-lane scenario, the generated `regular/self` candidate
  still needs to include the ADC's current lateral position. Otherwise the
  planner cannot produce a valid self-lane candidate while the ADC is still
  partially outside the lane, `able_to_use_self_lane_counter` never rises, and
  borrow exit can stall.

## Code anchors

1. `PathAssessmentDecider` only increments
   `front_static_obstacle_cycle_counter` and sets `front_static_obstacle_id`
   when the selected path carries a real blocking obstacle.
2. `PathAssessmentDecider` also uses the selected path label to update
   `able_to_use_self_lane_counter`, so return-to-self depends on a valid
   `regular/self` candidate actually being generated and selected.
3. `PathBoundsDecider::GetBoundaryFromLanesAndADC()` should keep the self-lane
   candidate wide enough to include the current ADC position during active
   borrow-lane so `regular/self` can reappear before the ADC is perfectly back
   inside the lane.
4. `PathLaneBorrowDecider::CheckLaneBorrow()` should evaluate neighbor-lane
   borrowability only through the obstacle's current longitudinal position for
   the current road-derived borrow regressions. Scanning farther can veto a
   locally valid pass because of downstream solid boundaries that belong to the
   post-pass route segment rather than the actual side-pass window.

## Regression-tested scenarios

- `borrow-lane` still passes with `borrow_status=entered-and-exited`.
- `borrow-lane-zt-road-20260419074214-lane79-center-obstacle` still passes with
  `borrow_status=entered-and-exited`.
- `borrow-lane-wide` remains a diagnostic `route-ok-no-borrow` scenario and was
  not unintentionally changed by the obstacle-`s` horizon.
- `borrow-lane-zt-road-202604180744-static-car` and
  `borrow-lane-zt-road-202604180744-static-car-zt-0-1-6` both regressed to
  `route-ok-no-borrow` when the scan went beyond the obstacle, and both
  recovered to `borrow_status=entered-and-exited` when the scan was capped at
  the obstacle's current `s`.

## Practical interpretation

- If a road-test or integration case shows `blocking obstacle is not long-term`
  with an empty `front_static_obstacle_id`, the first question should be
  whether the obstacle is truly blocking the selected path, not whether
  perception typed it as unmovable.
- If a case shows a locally borrowable neighbor lane but still gets rejected by
  `CheckLaneBorrow()`, inspect whether the scan is hitting a downstream solid
  boundary after the obstacle rather than at the obstacle.
- If a previously passing long borrow scenario regresses to `route-ok-no-borrow`
  after changing the lane-borrow scan horizon, first test whether capping the
  scan at the obstacle's current `s` restores the pass.
- If borrow enters but does not cleanly exit, inspect whether `regular/self`
  candidates are being generated while the ADC is still laterally offset.

## Evidence

- `data/road_tests/scenarios/road-test-20260419074214-zt-0-1-7`
- `data/integration_tests/scenarios/borrow-lane-zt-road-20260419074214-lane79-center-obstacle`
- `modules/planning/tasks/deciders/path_assessment_decider/path_assessment_decider.cc`
- `modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.cc`
- `modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.cc`
