# Operational and debugging tools

`modules/tools/` is retained as the transitional location for operational,
debugging and test utilities. The directory is intentionally unchanged while
tool ownership and packaging are migrated separately.

| Tool | Purpose |
| --- | --- |
| [whl-can](whl-can/README.md) | CAN keyboard control and vehicle CAN automation tests |
| [whl-debug](whl-debug/docs/data_extraction_tools.md) | Online and offline Cyber record data extraction |
| [whl-mock](whl-mock/README.md) | Message templates, publishing and routing helpers |
| [whl-nanoradar-visualizer](whl-nanoradar-visualizer/README.md) | Browser visualization for NanoRadar data |
| [whl-remote](whl-remote/README.md) | Remote-access installation and management |
| [whl-tf-tools](whl-tf-tools/README.md) | Coordinate-transform visualization, queries and conversion |

`whl-stream` is an NVIDIA/GStreamer host utility without a standalone README.
`gen_map`, `vehicle_calibration`, `whl-tools` and `whl_toolbox` are compatibility
or integration entries; follow the referenced external tool-kit documentation
where applicable.

These tools do not establish runtime, hardware or vehicle-safety readiness.
Review topics, recording paths and hardware configuration before use.
