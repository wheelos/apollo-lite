# Migrated Tool Entry

This directory keeps a thin CLI wrapper for historical command paths.

Runtime architecture:
- uses module-owned runner in `modules/perception/traffic_light/tools/traffic_light_detection`
- no dependency on legacy `camera/app/traffic_light_camera_perception`

Recommended target:
- `//modules/perception/traffic_light/tools/traffic_light_detection:traffic_light_detection_tool`

Compatibility wrapper target:
- `//modules/perception/camera/tools/traffic_light_detection:traffic_light_detection`
