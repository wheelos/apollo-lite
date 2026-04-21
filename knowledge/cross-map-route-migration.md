# Cross-map route migration

Road-derived scenarios cannot safely reuse lane ids across map versions, even
when the bag, waypoint coordinates, and obstacle coordinates stay the same.

## Why

For the `20260418074435.record.00000` borrow-lane case, the same coordinates
project very differently on `zt_0.1.7` vs `zt_0.1.6`:

| Point | `zt_0.1.7` projection | `zt_0.1.6` projection |
| --- | --- | --- |
| first localization | `Lane_89 s≈16.244` | `Lane_80 s≈17.494` |
| last localization | `Lane_98 s≈10.253` | `Lane_80 s≈36.997` |
| raw start waypoint pose | `Lane_89 s≈16.253` | `Lane_80 s≈17.503` |
| raw end waypoint pose | `Lane_96 s≈16.848` | `Lane_80 s≈0.171` |
| obstacle `(760134.63, 3843326.58)` | `Lane_80 s≈18.265` | `Lane_80 s≈59.500` |

So “same XY” does **not** imply “same routing semantics” or even “same
obstacle distance along lane.”

## Stable rule

When migrating a road-derived scenario to another map version:

1. treat the target map as authoritative,
2. re-project the first localization onto the target map,
3. rebuild same-lane wraparound requests from that target-map localization, and
4. only then append continuation waypoints / synthetic obstacles.

Do **not** trust the original lane ids alone. They came from a different map
version and may no longer describe the same semantic lane segment.

## Reusable tool flow

Use the existing tools instead of hand-editing scenario files:

```bash
whl start test

./data/road_tests/run.sh create \
  --scenario-id road-test-20260418074435-zt-0-1-6 \
  --record data/bag/20260418074435.record.00000 \
  --map-dir modules/map/data/zt_0.1.6 \
  --suite planning.borrow_lane.road \
  --case record_202604180744_zt_0_1_6 \
  --tags planning,borrow-lane,road-test,zt_0.1.6 \
  --require-borrow-enter \
  --force

./data/road_tests/run.sh create-integration road-test-20260418074435-zt-0-1-6 \
  --scenario-id borrow-lane-zt-road-202604180744-static-car-zt-0-1-6 \
  --obstacle-x 760134.63 \
  --obstacle-y 3843326.58 \
  --obstacle-template small_car \
  --require-borrow-enter \
  --require-borrow-exit \
  --observation-timeout 180 \
  --observe-full-timeout-on-success \
  --force
```

`create_case_from_road_test.py` now always applies the replay-safe
same-lane-wraparound rebuild from target-map localization, even when the target
map still projects the start onto the same raw lane id. This keeps map-version
migrations on the same reusable generator path.

## Validated result

The migrated `zt_0.1.6` scenarios were revalidated end to end:

- road-test scenario `road-test-20260418074435-zt-0-1-6`
  - `route_status=ok`
  - `borrow_status=entered-and-exited`
  - `pull_over_status=absent`
  - `verdict=borrow-completed`
- integration scenario `borrow-lane-zt-road-202604180744-static-car-zt-0-1-6`
  - `route_status=ok`
  - `borrow_status=entered-and-exited`
  - `pull_over_status=absent`
  - `observation_wait_secs=180`
  - `verdict=borrow-completed`
