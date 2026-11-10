# Documentation map

This directory is the repository-level documentation entry point. Detailed
contracts remain in the owning module or tool directory; this page only routes
readers to the canonical location.

## First-time setup

- [Host setup and container lifecycle](setup.md)
- [Development container build](../docker/build/README.md)
- [Host configuration scripts](../docker/setup_host/README.md)

## Build and delivery

- [Product builds and runtime products](../tools/products/README.md)
- [Hardware profile builds](../tools/hardware/README.md)
- [Algorithm module selection](../tools/modules/README.md)
- [Unified Cyber system assembly](../tools/system/README.md)
- [Transform runtime](../tools/transform/README.md)
- [Bzlmod and multi-repository development](../.agents/skills/multi-repo-dev/SKILL.md)
- [Offline vendor delivery](../.agents/skills/vendor/SKILL.md)

## Product configuration

- [Low-speed product configuration](../products/low_speed/conf/README.md)
- [Global runtime flags](../products/low_speed/conf/global_flagfile.txt)
- [Vehicle parameter template](../products/low_speed/conf/vehicles/default/vehicle_param.pb.txt)
- [Low-speed runtime products](../products/low_speed/BUILD)

## Tools

`modules/tools/` is retained as the transitional location for operational and
debugging utilities.

- [CAN keyboard controller](../modules/tools/whl-can/README.md)
- [CAN automation tests](../modules/tools/whl-can/auto_test/README.md)
- [Data extraction tools](../modules/tools/whl-debug/docs/data_extraction_tools.md)
- [Message publisher and routing tools](../modules/tools/whl-mock/README.md)
- [NanoRadar visualizer](../modules/tools/whl-nanoradar-visualizer/README.md)
- [Remote management](../modules/tools/whl-remote/README.md)
- [TF tools](../modules/tools/whl-tf-tools/README.md)

## Contributor and agent guidance

- [Repository Agent Guide](../AGENTS.md)
- [Agent resources](../.agents/README.md)
- [Skill index](../.agents/skills/README.md)
- [Workflow specifications](../.agents/workflows/README.md)
- [Agent tooling](../scripts/agents/README.md)
- [Copilot adapter](../.github/copilot-instructions.md)

## Source-generated documentation

The `apollo.sh doc generate` command uses the repository
[Doxygen configuration](../apollo.doxygen) and writes HTML to `.cache/docs`.
Generated output is not committed. Agent skills remain the procedure authority,
and the internal `ad_agents/context/` checkout remains the durable repository
knowledge authority for contributors who have access to it.

## Documentation ownership

| Content | Owner |
| --- | --- |
| Product goals, supported workflows and entry links | Root README and this map |
| Host/container procedures | `docs/setup.md` and `docker/` |
| Build and runtime contracts | The owning directory under `tools/` |
| Product defaults | `products/<product>/` |
| Tool usage | The owning directory under `modules/tools/` |
| Agent policy, procedures and workflows | `AGENTS.md` and `.agents/` |
| Durable source-backed repository facts | Internal `ad_agents/context/` |
