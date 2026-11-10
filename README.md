<div align="center">

<h1>
  <a href="#"><img src="docs/images/wheelos.jpg" width="100%" alt="WheelOS"/></a>
</h1>

**Apollo-Lite low-speed autonomous-driving integration**

[English](README.md) | [中文](README.zh-cn.md)

[![Lint and format](https://github.com/wheelos/apollo-lite/actions/workflows/lint-format.yml/badge.svg?branch=main)](https://github.com/wheelos/apollo-lite/actions/workflows/lint-format.yml)
[![License](https://img.shields.io/github/license/wheelos/apollo-lite)](LICENSE)

</div>

## What this repository provides

Apollo-Lite is a product-integration root for a low-speed autonomous-driving
system. It combines independently versioned hardware, perception, localization,
planning, control, runtime and tool repositories through Bzlmod.

The repository owns:

- low-speed product configuration and launch topology;
- hardware, module and runtime-product selection;
- the managed Ubuntu development environment;
- runtime assembly, packaging, installation and cross-module tests;
- contributor and agent workflows.

It does not own the implementation of every algorithm module. Those modules
remain in their source repositories listed in [`wheelos.repos`](wheelos.repos).

## Choose your path

| Goal | Start here |
| --- | --- |
| Prepare the host and enter the development container | [Host setup](docs/setup.md) |
| Preview or build a product inventory | [Product builds](tools/products/README.md#legacy-build-inventories) |
| Inspect the low-speed product definition | [products/low_speed](products/low_speed/BUILD) |
| Build a complete runtime product | [Runtime products](tools/products/README.md#runtime-products-one-component-or-many) |
| Select hardware or sensor backends | [Hardware builds](tools/hardware/README.md) |
| Select algorithm modules | [Selected modules](tools/modules/README.md) |
| Assemble one shared Cyber system | [System assembly](tools/system/README.md) |
| Use operational and debugging tools | [Tool documentation](modules/tools/README.md) |
| Contribute code or investigate failures | [Agent Guide](AGENTS.md) |
| Browse all documentation | [Documentation map](docs/README.md) |

## Quick start

Prerequisites:

- Ubuntu 22.04 host with Docker, or a compatible environment prepared by
  [`setup_host.sh`](docker/setup_host/setup_host.sh);
- at least 8 CPU cores and 8 GB RAM recommended;
- an NVIDIA Turing or newer GPU for the GPU workflow;
- a local checkout of the repositories declared in [`wheelos.repos`](wheelos.repos)
  when using local source overrides.

Prepare the host, start the container and enter it:

```bash
sudo bash docker/setup_host/setup_host.sh
whl start dev
whl enter dev
```

Preview a low-speed build inventory before compiling:

```bash
bash apollo.sh profiles
bash apollo.sh build --product=devkit-x86 --show
```

Build and run the explicit low-speed runtime product:

```bash
bash apollo.sh build \
  --product=//products/low_speed:devkit_x86 \
  --config=product_devkit-x86 \
  --show

bash apollo.sh run \
  --product=//products/low_speed:devkit_x86 \
  --config=product_devkit-x86 \
  -- --launch launch/low_speed/devkit_x86.launch
```

`devkit-x86` is a build inventory. `//products/low_speed:devkit_x86` is the
runtime-product label used for build, run, package and install. They serve
different purposes and should not be treated as interchangeable deployment
qualification.

For local delivery validation, add `--source=dev`:

```bash
bash apollo.sh package \
  --product=//products/low_speed:devkit_x86 \
  --source=dev \
  --config=product_devkit-x86

bash apollo.sh install \
  --product=//products/low_speed:devkit_x86 \
  --source=dev \
  --config=product_devkit-x86 \
  --destdir=/absolute/install/path
```

Do not start vehicle or sensor programs until the target hardware, calibration,
domain, topic and safety configuration have been reviewed.

## Repository map

| Path | Responsibility |
| --- | --- |
| [`products/low_speed/`](products/low_speed/) | Low-speed product defaults, vehicle template and launch topology |
| [`tools/products/`](tools/products/README.md) | Customer-facing build inventories and `runtime_product` assembly |
| [`tools/hardware/`](tools/hardware/README.md) | Hardware profile selection, runtime assembly and packaging |
| [`tools/modules/`](tools/modules/README.md) | Algorithm-module selection across external repositories |
| [`tools/system/`](tools/system/README.md) | One shared Core/tool/configuration system with explicit modules |
| [`tools/transform/`](tools/transform/README.md) | Transform runtime assembly and service example |
| [`docker/`](docker/) | Host setup, container definitions and container lifecycle |
| [`scripts/`](scripts/) | Build, runtime, CI and agent tooling |
| [`modules/tools/`](modules/tools/README.md) | Transitional operational and debugging tools |
| [`.agents/`](.agents/README.md) | Task procedures and workflow specifications |
| [`docs/`](docs/README.md) | Repository documentation map and setup guides |

## Scope and safety

All `devkit-*` products are example integration inventories unless explicitly
qualified for a deployment. They are not vehicle safety certification.

- Installation never starts components.
- Hardware and sensor startup requires reviewed DAGs, configuration and
  calibration.
- Release delivery must use immutable source versions and the formal Bzlmod
  registry path.
- Road, HIL, and control-actuation runs require explicit approval and the
  applicable safety procedure.

## Documentation and contribution

- [Documentation map](docs/README.md)
- [Host setup and container lifecycle](docs/setup.md)
- [Build and runtime products](tools/products/README.md)
- [Hardware profiles](tools/hardware/README.md)
- [Agent Guide](AGENTS.md)
- [Skills](.agents/skills/README.md)
- [Workflows](.agents/workflows/README.md)
- [Agent tooling](scripts/agents/README.md)
- [Issues and support](https://github.com/wheelos/apollo-lite/issues)

## License

Apollo-Lite is licensed under the [Apache License 2.0](LICENSE).

For collaboration and business inquiries, contact `daohu527@gmail.com`.
