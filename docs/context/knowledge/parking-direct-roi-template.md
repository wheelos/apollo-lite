# Direct parking ROI template

Direct valet parking now treats `parking_approach_preference` as a terminal-pose
selector, not a guaranteed gear-policy selector. `PARKING_APPROACH_PREFER_TAIL_IN`
selects a tail-in end pose, but Hybrid A* can still choose the maneuver sequence
that best satisfies the ROI and cost terms.

For direct parking, the hard optimizer ROI is generated after deterministic pose
selection from the ego footprint, selected goal footprint, parking slot polygon,
and the corridor between ego and goal. The ROI is bounded in the parking-slot
coordinate frame derived from the opening edge and slot depth axis, which keeps
the search region tied to the slot/opening instead of using a large world-axis
rectangle.

The template keeps a minimum maneuver envelope so Hybrid A* has enough room to
find feasible head-in/tail-in paths; over-tight corridor-only ROIs caused open-set
exhaustion in real San Mateo and Sunnyvale parking scenarios.

Parking-slot opening selection is ego/session aware. Map slots must be built with
a reference point, and direct parking caches the selected opening for the active
parking-space id so the entrance edge does not flip after the vehicle moves into
the slot. If the routing/start reference is already inside a slot, entrance
selection only falls back to slot-depth heading for angled-like slots; normal
perpendicular/parallel slots continue to use the nearest ego-side opening. This
prevents the San Mateo 3111 inside-slot start from selecting the rear edge while
also preventing Sunnyvale 11561 from flipping after the first frame.

Checked-in parking scenarios should start outside the target slot on the approach
side when they are meant to validate an entry maneuver. The San Mateo 3111 4.0m
scenario uses an approach-side start and a tail-in heading expectation so the
test validates the actual parking entry instead of starting from inside the
target polygon.
