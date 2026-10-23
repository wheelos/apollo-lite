# Routing

## Introduction
  The Routing module generates high level navigation information based on requests.

  The Routing module depends on a routing topology file, usually named `routing_map.*` in apollo.

  The routing map can be genreated using this command:
  ```
  bash scripts/generate_routing_topo_graph.sh
  ```

## Input
  * Map data
  * Routing request (Start and end location)

## Output
  * Routing navigation information

## Bidirectional longitudinal routing

The topology generator preserves the map's existing `Lane.direction`
(`FORWARD`, `BACKWARD`, or `BIDIRECTION`). Regenerate `routing_map.*` after
updating the generator: old topology files default to `FORWARD`.

Directional routing searches permitted longitudinal connections using distance
and Dijkstra, independently of Planning. Native successor relations connect a
lane's tail to its successor's head; reversing a connection requires reverse
access on both lanes. It does not connect opposite directions inside a lane.
Non-branching consecutive bidirectional states are compressed into chains,
including a retained closing connection for cycles. Lateral connections prevent
compression across their endpoints. Waypoints and blocked intervals split the
query graph without changing the stored topology. Dispersed bidirectional lanes
remain separate.

Results expand chains back into original lane IDs. `LaneSegment.start_s` and
`end_s` remain ordered native map coordinates; `traversal_direction` specifies
whether to traverse the interval along or against the centerline. Optional
waypoint headings constrain the permitted direction. Forward-only maps retain
the existing lane-change search. On directional maps, existing forward
lane-change routing is a fallback for unconstrained forward-access waypoints;
mixed reverse/lateral maneuvers are not introduced by this longitudinal feature.

The new result field is defined in the local `pkgs/wheelos_msgs` checkout.
Local validation uses `--config=dev`; registry consumers need a published
compatible message version before deployment. Older consumers ignore the new
field and must not be used for reverse routes.

Planning consumes the result independently through PncMap and direction-aware
lane segments. Reference-line points follow traversal order while lane
waypoints retain native `s`; projections, left/right widths, overlap intervals,
and interpolation account for traversal direction. This is reference geometry,
not a gear, body-heading, open-space, or execution-mode contract.

For routes containing reverse segments, PncMap identifies each lane occurrence
by its position in the response, not just its lane ID. Extension, cropping,
interpolation, and stitching preserve that identity, so a repeated lane does
not select an earlier visit. Passage lookup uses the ordered route index;
extension walks the requested local window rather than rescanning the route
prefix. This metadata stays inside Planning and does not change the wire format.

Focused tests (inside the managed development container):

```bash
bazel test --config=dev \
  //modules/routing/core:directional_routing_test \
  //modules/routing/topo_creator:graph_creator_test
bazel test --config=dev \
  //modules/map/pnc_map:pnc_path_test \
  //modules/planning/reference_line:directional_reference_line_test
```
