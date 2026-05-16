# Direct parking architecture review

Current direct valet parking is organized as a compact pipeline:

1. Dreamview/routing sends a standard `RoutingRequest` carrying parking info.
2. `ScenarioManager` / `OnLanePlanning` enter direct `VALET_PARKING_PARKING`
   without depending on a prior on-lane reference-line stage.
3. `ParkingSlotProvider` normalizes the slot from HDMap/perception corners and an
   ego/session reference point, so opening-side semantics stay stable.
4. `ParkingPoseSelector` deterministically generates head-in/tail-in terminal
   pose candidates from slot type and config preference.
5. `OpenSpaceRoiDecider` builds the slot-aligned pit ROI plus start/goal template,
   validates the selected goal, and falls back to the alternate candidate if the
   preferred pose is invalid.
6. `HybridAStar` performs discrete search over motion primitives and uses
   `AnalyticExpansion()` to enumerate Reed-Shepp connections from the current
   node to the goal; Reed-Shepp is an analytic connector inside Hybrid A*, not a
   replacement for the graph search itself.
7. Open-space optimization / partition consume the warm-start result and publish
   the executable parking trajectory.

Production lessons from this refactor:

- ROI construction must stay slot-frame-aligned and ego-aware. Overly large
  world-axis rectangles make search inefficient, while over-tight corridor-only
  regions cause open-set exhaustion.
- Parking opening selection must be stable over the whole parking session.
  Recomputing the entrance edge from the current ego pose can flip opening/rear
  semantics after the vehicle moves into or across the slot mouth.
- The routed parking lane used to build the nearby path must be chosen by the
  earliest matching routed overlap lane, not by a cumulative index across
  unrelated overlap candidates.
- Preferred head-in/tail-in policy should remain deterministic, but ROI/goal
  validation should retry the alternate candidate before failing the parking
  attempt.

This architecture is now reasonable for production hardening because each module
has a single dominant responsibility: slot semantics, terminal pose generation,
ROI construction/validation, warm start, and trajectory execution are separated
instead of duplicating feasibility logic in multiple places.
