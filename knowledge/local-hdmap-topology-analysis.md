# Local HDMap topology analysis

This note captures a reusable way to diagnose planning behavior from a small
lane neighborhood instead of reading the full map.

## Why this matters

Borrow-lane failures are often caused by topology or lane-boundary details that
live in only a few adjacent lanes. The useful workflow is:

1. project the ego start, obstacle, and route waypoints onto the map;
2. extract only the touched lane chain and immediate neighbors;
3. confirm predecessor / successor / neighbor relations plus boundary types; and
4. align those local facts with the planning decider's lookahead logic.

For Apollo Lite, this is usually enough to explain a borrow-lane failure without
re-reading the entire `base_map.txt`.

## Reusable method

For a scenario on `zt_0.1.7`, the minimal local analysis should answer:

1. Which lane is the ego actually on now?
2. Which lane is the obstacle on?
3. What is the route chain ahead of the ego?
4. Does the current lane have a borrowable neighbor?
5. Do any successor lanes inside the decider's lookahead invalidate borrowing?

Useful inputs:

- route / obstacle coordinates projected with `data/integration_tests/tools/map_lane_utils.py`
- exact local lane blocks extracted from `data/zt_0.1.7/base_map.txt`
- `PathLaneBorrowDecider::CheckLaneBorrow()` in
  `modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.cc`

The crucial implementation detail is that `CheckLaneBorrow()` does **not** only
check the obstacle point or the current lane. It starts at
`reference_line_info.AdcSlBoundary().end_s()`, scans **100m forward** in **2m**
steps, and rejects borrowing on a side as soon as a sampled waypoint hits a
solid boundary on that side.

## Verified local topology for the road-derived borrow-lane case

For the road-derived integration case
`borrow-lane-zt-road-202604180744-static-car`, the verified local chain is:

```text
Lane_89 -> Lane_98 -> Lane_80 -> Lane_86
                     ^
                     |
                  left neighbor
                     |
                  Lane_79
```

Verified lane facts:

| Lane | Predecessor(s) | Successor(s) | Neighbor(s) | Left boundary | Right boundary |
| --- | --- | --- | --- | --- | --- |
| `Lane_89` | `Lane_90`, `Lane_95`, `Lane_96` | `Lane_97`, `Lane_98` | none | `SOLID_WHITE` | `SOLID_WHITE` |
| `Lane_98` | `Lane_89` | `Lane_80` | none | `SOLID_WHITE` | `SOLID_WHITE` |
| `Lane_80` | `Lane_98` | `Lane_86` | left forward `Lane_79` | `DOTTED_WHITE` | `SOLID_WHITE` |
| `Lane_86` | `Lane_80` | `Lane_57` | none | `SOLID_WHITE` | `SOLID_WHITE` |
| `Lane_79` | `Lane_97` | `Lane_85` | right forward `Lane_80` | `SOLID_WHITE` | `DOTTED_WHITE` |

This confirms the narrow map claim:

1. the obstacle can sit on `Lane_80`,
2. `Lane_80 -> Lane_79` is borrowable on the **left** in isolation, and
3. the route ahead still continues into `Lane_86`, whose **left** boundary is
   solid.

## Verified route / obstacle projections

With the raw RoutingRequest extracted from the record:

- waypoint ids remain `Lane_80`
- but the raw start pose projects near `Lane_89 s ~= 16.253`
- the raw end pose projects near `Lane_96 s ~= 16.848`

The synthetic parked-car obstacle used in the integration sweep projects onto:

- `Lane_80 s ~= 18.265` at the base coordinate `(760134.63, 3843326.58)`

An obstacle-only sweep moved that same car backward along the route direction to:

- `Lane_80 s ~= 16.265, 14.265, 12.265, 10.265, 8.265, 6.265`

## What the planning results mean

With the raw route kept unchanged and only the obstacle moved:

- `0m` through `8m` shifts all failed with
  `reason='no borrowable neighbor lane'`
- the latest blocking metadata stayed:
  - left: `Lane_86:SOLID_WHITE`
  - right: `Lane_80:SOLID_WHITE`
- `10m` and `12m` shifts still failed with the same reason, but both sides
  converged to:
  - left/right: `Lane_98:SOLID_WHITE`

This is the key local-map conclusion:

1. **`Lane_80` itself being left-borrowable is not sufficient.**
2. `CheckLaneBorrow()` scans forward across successor lanes, not just the
   obstacle's lane.
3. Once the scan includes `Lane_86`, its left `SOLID_WHITE` boundary vetoes
   left borrowing even though `Lane_80` has a left `DOTTED_WHITE`.
4. When the obstacle is moved closer to the `Lane_98 -> Lane_80` transition,
   the decider rejects even earlier on `Lane_98`, where both sides are already
   `SOLID_WHITE`.

So the correct explanation for this case is not “the map has no borrowable
neighbor at all.” It is:

> `Lane_80 -> Lane_79` is borrowable locally, but the borrow-lane decider's
> 100m forward scan still sees non-borrowable successor segments (`Lane_86` or,
> for earlier obstacle positions, `Lane_98`), so the overall left/right
> borrowability remains false.

## Validated fix direction

The minimal fix for this class of false negatives is to keep
`PathLaneBorrowDecider::CheckLaneBorrow()` focused on the borrowability window up
to the **blocking obstacle end-s**, instead of vetoing lane-borrow based on
farther successor-lane solids before the maneuver even starts.

This is safe in Apollo Lite because `PathBoundsDecider` still checks lane
boundary type point-by-point while constructing the borrow path. That downstream
logic can clamp the borrow corridor or force the path back to self-lane before a
later `SOLID_WHITE` segment, so the entry decider does not need to reject the
whole side-pass just because a later successor lane is not borrowable forever.

With that change applied, the same road-derived integration scenario
`borrow-lane-zt-road-202604180744-static-car` now validates as:

- `route_status=ok`
- `borrow_status=entered-only`
- `collisions=0`
- selected path label: `regular/left/forward`

That confirms the local topology diagnosis was correct: the old failure was an
overly aggressive borrow-entry veto, not a lack of a borrowable left neighbor on
`Lane_80`.
