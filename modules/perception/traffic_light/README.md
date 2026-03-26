# Traffic Light Module

This directory is the consolidation point for traffic light perception under
`modules/perception`.

Layering:
- `component/`: module-owned Cyber runtime entry
- `application/`: pipeline orchestration and use-case flow
- `domain/`: traffic light pipeline contracts and runtime state
- `infra/`: runtime factories and scene/preprocessor gateways
- `algo/`: module-owned algorithm stage implementations

Canonical runtime entry:
- `component/traffic_light_component.*` is now the module-owned Cyber entry
- `application/traffic_light_system.*` owns end-to-end runtime orchestration
- `application/traffic_light_perception_pipeline.*` owns model-stage flow

Current module shape:
- `component/`: Cyber lifecycle, readers, writers, runtime entry
- `application/`: end-to-end orchestration and model-stage pipeline
- `domain/`: module config, runtime state, result aggregation
- `infra/`: TF, HDMap, runtime stage factory, and preprocessor gateway
- `algo/`: detector/tracker/preprocessor stage implementations
- `proto/`: module-owned runtime configuration protocol (no onboard config
	schema coupling)

Quality baseline:
- module-level tests exist for domain decision aggregation and component
	configuration translation

Design direction:
- No direct dependency from business orchestration back into onboard traffic
	light code
- No detector/tracker orchestration in camera app layer
- All traffic light specific runtime coordination is centralized under this
	module path
