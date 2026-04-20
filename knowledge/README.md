# Knowledge Context

Store stable subsystem knowledge and cross-task engineering conclusions here.

## Index

- [Planning debug signal principles](planning-debug-signals.md) - rules for keeping planning-topic debug fields small, structured, and subscriber-oriented.
- [Simulation context](simulation.md) - backend simulation, routing injection, map selection, and sim control workflow notes.
- [Cross-map route migration](cross-map-route-migration.md) - why road-derived routes and obstacle semantics must be re-projected from target-map localization when map versions change.
- [Borrow-lane and pull-over interference](borrow-lane-pull-over-interference.md) - how long-run borrow-lane regressions can actually be dominated by mission pull-over entry and short self candidates after borrow completion.
- [Borrow-lane obstacle regime and lookahead](borrow-lane-obstacle-regime-and-lookahead.md) - blocking-obstacle promotion, obstacle lateral placement, borrow lookahead horizon, and return-to-self path generation.
- [Local HDMap topology analysis](local-hdmap-topology-analysis.md) - local lane-chain, boundary, and borrow-lane scan analysis without reading the whole map.
- [Cyber Python publish and subscribe](cyber-python-pubsub.md) - `cyber/setup.bash`, `create_writer`, `create_reader`, and planning-topic debugging.
- [Logging rules](logging-rules.md) - rate-limited logging guidance for high-frequency runtime loops.
- [OSQP API update and matrix construction](osqp-api-update.md) - OSQP 1.0 migration findings and Apollo Lite matrix construction behavior.
