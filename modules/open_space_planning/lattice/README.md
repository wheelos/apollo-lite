# Lattice kernels

This package owns all lattice-specific implementations used by open-space
planning.

- `trajectory/` contains the first migrated, map-independent 1D polynomial
  kernels from the existing Frenet lattice.
- Future route-lattice search, motion primitives, local frames, samplers,
  combiners, and evaluators belong under this package or the owning layer.

No target here may depend on `//modules/planning/lattice/...`. Common
non-lattice utilities may temporarily depend on another planning package only
when its API contains no HDMap, reference-line, `Frame`, task, or scenario
types. Such dependencies are migration debt and candidates for
`modules/common/planning`.

