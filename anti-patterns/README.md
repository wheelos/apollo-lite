# Anti-Patterns Context

Store workflows, assumptions, and implementation patterns that contributors should avoid repeating.

## Index

- [Cross-container debugging](cross-container-debugging.md) - avoid probing or switching to other users' containers during investigation.
- [Validating in the dev container](validating-in-dev-container.md) - avoid using `apollo_dev_*` as the default runtime-validation target now that `whl` manages an isolated `test` mode.
