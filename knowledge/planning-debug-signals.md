# Planning debug signal principles

This note captures the current design rules for adding planning-topic debug fields that are meant to be consumed by subscribers such as `data/integration_tests/tools/subscribe_planning.py`.

## Principles

1. **Prefer topic debug over repeated logs.** If the information is meant for automated diagnosis, add it to `ADCTrajectory.debug` instead of printing it every cycle.
2. **Only publish decision-grade signals.** Add fields that explain a planner decision: current state, gate results, chosen output, and a concise reason. Avoid dumping whole proto snapshots or large intermediate data.
3. **Keep the schema bounded.** Prefer booleans, enums, counters, IDs, and one short reason string. Do not add unbounded arrays, per-point traces, or large repeated debug payloads for hot-path decisions.
4. **Tie every field to an active consumer.** A field should exist only when a subscriber, test harness, or operator summary reads it. If there is no reader yet, defer the field.
5. **Expand by scenario, not by speculation.** For now, only the validated borrow-lane workflow should own new planning-topic debug fields. Parking, pull-over, and other scenarios can add their own focused signals later when their test fixtures and assertions exist.

## Current minimal scope

The current minimal planning-topic debug surface is `PlanningData.lane_borrow`. It is sufficient for borrow-lane diagnosis because it exposes:

- the lane-borrow state,
- the key gate outcomes,
- the selected side-pass direction,
- the selected path label,
- the transition reason,
- and, when lane borrowing is rejected for neighbor-lane availability, the bounded
  blocking lane metadata (`left/right_blocking_lane_id`, boundary type, and `s`).

This scope intentionally excludes destination and pull-over diagnostics for now.
