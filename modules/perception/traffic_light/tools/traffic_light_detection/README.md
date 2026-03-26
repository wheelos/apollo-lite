# Traffic Light Detection Tool (Module-Owned)

This tool is the module-owned offline runner for traffic light perception.

Architecture:
- `TrafficLightDetectionToolRunner`: tool application flow
- `TrafficLightSeedProvider`: pluggable seed strategy for map/projection input
- `TrafficLightResultWriter`: pluggable result sink for txt/image/json outputs

Current defaults:
- `FullFrameSeedProvider`: seeds one full-image ROI placeholder
- `LocalFileResultWriter`: writes `sample_id.txt` and `sample_id.jpg`

Run:
- `bazel build //modules/perception/traffic_light/tools/traffic_light_detection:traffic_light_detection_tool`
- `bazel-bin/modules/perception/traffic_light/tools/traffic_light_detection/traffic_light_detection_tool --pipeline_config_path=... --image_root_dir=... --test_list_path=...`

Extension points for your algorithm module:
- Replace `FullFrameSeedProvider` with HDMap/pose-aware seeding
- Replace `LocalFileResultWriter` with structured metrics and benchmark reporters
- Keep runner unchanged while evolving detector/tracker implementation
