# Logging Rules

This note captures runtime logging rules for high-frequency modules such as planning.

## Rate-Limited Cyber Macros

`cyber/common/log.h` provides rate-limited logging macros:

- `AINFO_EVERY(freq)`
- `AWARN_EVERY(freq)`
- `AERROR_EVERY(freq)`

Use them when a log line can execute inside a high-frequency loop and the message is useful only occasionally.

## Rule Of Thumb

- Keep **state transitions**, **unexpected failures**, and **one-time initialization problems** as plain `AINFO`, `AWARN`, or `AERROR`.
- Convert **per-cycle informational logs** to `AINFO_EVERY(...)` when they otherwise spam `data/log/*.out`.
- Prefer structured planning debug data on `/apollo/planning` over verbose repeated logs for lane-borrow reasoning.

## Planning Example

In `modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.cc`, the reusable-path skip message is on a hot path and should be rate-limited, while lane-borrow enter or exit transitions remain normal `AINFO` logs because they represent meaningful state changes.

Other high-frequency planning examples that should stay rate-limited rather than per-cycle:

- startup or transient readiness messages such as `"routing not ready; skip the planning cycle."`,
- per-frame trace logs such as `"Planning start frame sequence id = [...]"`,
- scenario bidding summaries such as `decision.DebugString()` inside `ScenarioManager::ScenarioDispatch()`.
