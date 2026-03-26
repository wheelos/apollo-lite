# Traffic Light Algo Layer

This layer is the consolidation point for traffic light algorithms.

Current shape:
- `detector/traffic_light_detector_stage.*`: module detector stage
- `tracker/traffic_light_tracker_stage.*`: module tracker stage
- `preprocessor/traffic_light_preprocessor_stage.*`: module preprocessor stage

Extension practice:
- keep stage lifecycle contract stable (`Init/Validate/Process/Shutdown`)
- evolve concrete model internals in this layer without changing application
  orchestration contracts
