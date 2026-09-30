# Trajectory Continuity

This package owns the validated trajectory output contract and temporal
continuity. It is downstream of `planning_context/` and independent of the
lane-follow generator, Scenario and world-model producer. `trajectory.h`
defines the current bounded lane-follow output and vehicle policy;
`trajectory_validation` checks the result against the admitted cycle,
kinematics, limits, sources and swept footprint without trusting
`LocalTrajectory::valid`. A future open-space generator needs its own
mode-appropriate validator before its trajectory can be accepted.

`TrajectoryContinuity::Accept(input, config, candidate)` validates a candidate
and stores it only on success; `Reset()` discards history after failure,
mode/epoch transition or explicitly invalidated planning input. A subsequent
`Start(input, tolerances, &start)` interpolates the stored trajectory at the
**current absolute planning time**. Source frame, clock, epoch, lane identity
and execution deadline must match. A tracking mismatch reanchors x/y/heading
and speed to measured ODOM while retaining bounded acceleration/curvature
references; a match yields a time-aligned seed. The report distinguishes
STITCHED, REANCHORED, expiry and source mismatch. No past trajectory is used
as a localization measurement or to enlarge newly observed boundaries.

This is single-state temporal alignment, not an optimization-based joint
space-time stitch, a copied trajectory prefix, or proof that a later
generated trajectory is safe. The downstream planner still validates each
candidate before publication. `planning_context/reference_continuity` is a
separate mechanism for world-geometry continuity; it does not store execution
trajectories.
