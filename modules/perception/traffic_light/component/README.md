# Traffic Light Component Layer

This layer owns the Cyber component boundary for traffic light perception.

Canonical runtime entry:
- `traffic_light_component.h`
- `traffic_light_component.cc`

Responsibilities:
- load component config and pipeline config
- create camera and V2X readers
- publish traffic light detection results
- delegate all traffic light business logic to `application/traffic_light_system`

This directory is now the preferred runtime entry over the historical onboard
traffic light component.
