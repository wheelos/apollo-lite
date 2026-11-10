<div align="center">

<h1>
  <a href="#"><img src="docs/images/wheelos.jpg" width="100%"/></a>
</h1>

[English](README.md) | [中文](README.zh-cn.md) | [한국어](README.ko.md) | [日本語](README.ja.md)

<p>
  <a href="https://github.com/wheelos/apollo-lite/actions/workflows/lint-format.yml?query=main" alt="GitHub Actions">
    <img src="https://img.shields.io/github/actions/workflow/status/wheelos/apollo-lite/lint-format.yml?branch=main">
  </a>
  <a href='https://readthedocs.org/projects/apollo-lite/badge/?version=latest' alt='Documentation Status'>
      <img src='https://readthedocs.org/projects/apollo-lite/badge/?version=latest' alt='Documentation Status' />
  </a>
  <a href="https://github.com/wheelos/apollo-lite/blob/main/LICENSE" alt="License">
    <img src="https://img.shields.io/github/license/wheelos/apollo-lite">
  </a>
</p>

A high-performance autonomous driving system
</div>

## Table of Contents

- [Introduction](#introduction)
- [Repository Guide](#repository-guide)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Production Mode (Docker)](#production-mode-docker)
- [Copyright and License](#copyright-and-license)
- [Connect with Us](#connect-with-us)

---

## Introduction

Apollo-Lite provides powerful modules and features for autonomous driving development.
Before getting started, please ensure your environment meets the prerequisites and follow the installation instructions below.

For a deeper understanding, refer to the following documents:
- Design Document: [[中文](https://fcn5tm1hmy9p.feishu.cn/wiki/J5ujwMD44iz6IlkD7etcFfrinZf?fromScene=spaceOverview) | [English]]
- Integration Document: [[中文](https://fcn5tm1hmy9p.feishu.cn/wiki/WQVmwCVw6i93wOk68QRchUQMnwe?fromScene=spaceOverview) | [English]]
- Development Process: [[中文](https://fcn5tm1hmy9p.feishu.cn/wiki/XdlSwmdLiiDXBdkuF9qcwKFun2d?fromScene=spaceOverview) | [English]]
- Tools Document: [[中文](https://fcn5tm1hmy9p.feishu.cn/wiki/SQVtw66pCiJlOTkZBoCc4tzlnzg?fromScene=spaceOverview) | [English]]
- Product Manual: [[中文](https://fcn5tm1hmy9p.feishu.cn/wiki/Y4WKw4oX4iCfQ8kmmabccgEUnhf?fromScene=spaceOverview) | [English]]
- Issues / FAQ: [[中文](https://fcn5tm1hmy9p.feishu.cn/wiki/IS9Vw11zcir6u5k8xFIc3BXJnEf?fromScene=spaceOverview) | [English]]
- WEP Proposal: [[中文](https://fcn5tm1hmy9p.feishu.cn/wiki/WSgLwkMOkir6aSkaJGWcYa5ZnJb?fromScene=spaceOverview) | [English]]
- Company Introduction: [[中文](https://fcn5tm1hmy9p.feishu.cn/wiki/Fp3WwaoZ9iUuw8kD6sgcEsERnSe?fromScene=spaceOverview) | [English]]

---

## Repository Guide

Apollo-Lite integrates independent repositories listed in [`wheelos.repos`](wheelos.repos).
Use this guide to find the repository that owns a capability. Module names below
describe content inside each repository; they are not necessarily separate
repositories. The capability groupings are for discovery and do not imply that
the repositories are merged or depend on one another.

### Autonomous-driving capabilities

| Capability | Repository | Example modules or contents |
| --- | --- | --- |
| LiDAR perception | [lidar_perception](https://github.com/wheelos-euc/lidar_perception) | `lidar_object_detection`, `lidar_semantic_segmentation`, `lidar_fusion` |
| Camera perception | [camera_perception](https://github.com/wheelos-euc/camera_perception) | `camera_semantic_segmentation` |
| Multimodal perception | [multimodal_perception](https://github.com/wheelos-euc/multimodal_perception) | `bevfusion` |
| World model | [world_model](https://github.com/wheelos-euc/world_model) | `world_model` (including local map), `lane`, `traffic_light`, `storytelling` |
| Localization | [localization](https://github.com/wheelos-euc/localization) | `localization`, `ndt_localization`, `rtk_localization` |
| Planning | [planning](https://github.com/wheelos-pnc/planning) | `planning`, `local_planning`, `open_space_planning`, `map` |
| Prediction | [prediction](https://github.com/wheelos-euc/prediction) | `prediction` |
| Navigation | [mission](https://github.com/wheelos-pnc/mission), [routing](https://github.com/wheelos-pnc/routing) | Task orchestration and route services; these are separate repositories. |
| Control | [control](https://github.com/wheelos-pnc/control) | Control and execution |
| Safety | [safety](https://github.com/wheelos/safety) | `guardian`, `collision_guardian`, `monitor` |
| Simulation | [simulation](https://github.com/wheelos/simulation) | Simulation |
| Operations and observability | [hmi](https://github.com/wheelos/hmi), [safety](https://github.com/wheelos/safety) | Dreamview is in `hmi`; `monitor` is in `safety`. |
| Existing Perception | [perception](https://github.com/wheelos-euc/perception) | Separate existing repository; see its README for its current scope and transition dependencies. |

### Platform, integration, and tools

| Area | Repository | Example contents |
| --- | --- | --- |
| Runtime | [core](https://github.com/wheelos/core) | Cyber runtime and tools |
| Shared libraries | [common](https://github.com/wheelos/common) | Common utilities and shared components |
| Vehicle and sensor integration | [hardware](https://github.com/wheelos/hardware) | Hardware-facing integrations |
| Map | [map](https://github.com/wheelos/map) | Map repository |
| Coordinate transforms | [transform](https://github.com/wheelos/transform) | Transform components |
| Message contracts | [wheelos_msgs](https://github.com/wheelos/wheelos_msgs) | Shared message definitions |
| Developer tools | [tool_kit](https://github.com/wheelos/tool-kit) | WheelOS tools and utilities |
| Build registry | [bazel-central-registry](https://github.com/wheelos/bazel-central-registry) | Bazel module registry |

The `map` repository is distinct from the `map` module inside the planning
repository. For build and product integration, see the Bzlmod and selected
product build guidance below.

---

## Prerequisites

- **Machine:** Minimum 8-core CPU, 8GB RAM
- **GPU:** NVIDIA Turing GPU recommended for acceleration
- **Operating System:** Ubuntu 20.04 LTS

---

## Host setup and container startup (quick guide)

### Purpose
- Ensure host is prepared (Docker, NVIDIA runtime, OS tuning) and provide a simple workflow to start the Apollo container using the `whl` helper.

### Summary (recommended flow)
1. Run the host setup script (interactive): installs Docker, NVIDIA toolkit and then runs system configuration steps interactively.

```bash
sudo bash docker/setup_host/setup_host.sh
```

- `setup_host.sh` will install `whl` (system command linked at `/usr/local/bin/whl`) before running system configuration.
- After completing, the installer writes `/etc/wheelos_setup_host.done` to indicate host readiness.

2. Start or enter the container with `whl`:

```bash
# start in dev mode
whl start dev

# enter the dev container (starts it if needed)
whl enter dev

# stop the dev container but preserve it for a later start
whl stop dev

# stop and remove the container and its Compose network
whl rm dev

# pull the configured image, update the container, and wait for initialization
whl pull dev
```

### Notes about interactive system configuration
- `config_system.sh` (invoked by `setup_host.sh`) is interactive: for each optional system tuning step (NTP/PTP/udev/uvcvideo/CAN/Jetson tuning/headless/container and Dreamview boot services) it will ask whether to apply it (Y/n). Hardware-related tuning defaults to conservative choices.

### Automation / non-interactive runs
- `config_system.sh` and `setup_host.sh` detect non-interactive stdin and use sensible defaults. To run unattended and accept defaults, redirect stdin from `/dev/null`:

```bash
sudo bash docker/setup_host/setup_host.sh < /dev/null
```

- If you need to fully automate and explicitly choose Yes/No for every prompt, use an automation tool or supply answers via stdin (careful: using `yes` will force all answers to `y`). Example (force yes for all prompts):

```bash
yes | sudo bash docker/setup_host/setup_host.sh
```

### Environment and files
- User-maintained overrides live in the project-root `.env.global`.
- Use mode-scoped keys such as `DEV_USE_GPU`, `DEV_BAZEL_CACHE_DIR`, `TEST_SERVER_PORT`, `TEST_CPUS`, `TEST_MEMORY`, `TEST_USE_GPU`, and `TEST_BAZEL_CACHE_DIR` so dev and test stay isolated and explicit.
- Generated env files used by `whl` are `docker/.env.dev.local`, `docker/.env.test.local`, and `docker/.env.prod.local`. `whl` regenerates the requested mode file on every run from `.env.global` plus host auto-detection before launching.
- Each mode also gets its own Compose project name, so `dev` and `test` can run side by side without recreating or stopping each other.
- Container names are deterministic (`apollo_<mode>_<user>_<project-hash>`) and are intentionally not user-configurable.
- `whl start <mode>` starts using an image already available locally and waits for container initialization; it never pulls. Run `whl pull <mode>` first if the image is not present.
- `whl stop <mode>` stops and preserves the container for a later start. Use `whl rm <mode>` to stop and remove a container and its Compose network, or `whl rm all` for all configured modes. Host bind mounts are preserved.
- `whl pull <mode>` pulls the configured image, updates/recreates the container, and waits for initialization. This does not start or verify autonomous-driving services.
- `whl exec <mode> -- <command>` runs a non-interactive command as the mapped host user in a running, healthy container. It does not start the container.
- `whl-container.service` manages only the `prod` container. `whl-dreamview.service` is a separate optional application unit; it requests Dreamview launch start/stop through the existing `scripts/cyber_launch.sh` lifecycle helper.
- To manage the units manually, use `systemctl start|stop|status whl-container.service` and `systemctl start|stop|status whl-dreamview.service`. Dreamview requires the container unit. Host setup asks separately whether to enable each service at boot; enabling Dreamview also enables its container dependency.
- The Dreamview unit uses `Type=oneshot` with `RemainAfterExit=yes`: its start/stop script waits for the Dreamview process transition, but systemd does not supervise ongoing process health. This is not autonomous-driving readiness. The full autonomous-driving module graph still requires a profile-aware runtime supervisor.
- Host-ready marker: `/etc/wheelos_setup_host.done`
- `whl` helper location: `/usr/local/bin/whl`

---

## `@core` runtime (cyber tools)

### Bzlmod consumer modes

Apollo-Lite is the product integration root. `wheelos.repos` fetches independent
repositories into `pkgs/`; `MODULE.bazel` declares direct build dependencies;
`tools/` selects product modules and delivery targets. Apollo depends on
`hardware.*` modules, not the peer `vehicle_infra` or `sensor_infra` products.

Algorithm implementation, SDK integrations and module assets belong to their
own repositories. The root keeps Apollo-Lite product configuration and topology
(`products/apollo_lite/conf/`, `products/apollo_lite/launch/`),
build/runtime entrypoints (`apollo.sh`, `setup.bash`, `scripts/`), the managed
environment (`docker/`) and cross-module tools/tests. The standalone
visualizer and roadlog tools are owned by `wheelos/tool-kit`; Apollo-Lite keeps
their product-facing aliases and launcher integration, while the tool-kit
owns their Bazel and Qt dependencies.

Run these commands in the managed container as the mapped non-root user:

```bash
# Local source integration; never updates the formal consumer lockfile.
python3 scripts/bzlmod.py dev mod graph
python3 scripts/bzlmod.py dev build @core//tools:mainboard

# Candidate registry first, without developer or home rc files/source overrides.
python3 scripts/bzlmod.py registry build @core//tools:mainboard

# Remote registries only, with strict lockfile checking.
python3 scripts/bzlmod.py release build @core//tools:mainboard

# Only after every declared module is available from the formal registries:
python3 scripts/bzlmod.py lock-update mod deps

# Record revisions AND dirty state; this does not publish or freeze local edits.
python3 scripts/bzlmod.py snapshot > .cache/bzlmod-provenance.json
```

Commit `MODULE.bazel.lock` only after formal registry resolution succeeds.
Local source integration is not release acceptance. Perception operators and
model I/O are packages in `wheelos_common`, not separate Bzlmod modules.
Use registry mode to validate the local candidate while remote publication is
pending; release checks must not fall back to `pkgs` sources or the local registry.
Select GPU/manual tests explicitly; SM89 coverage does not establish Orin support.

The production path is the Bazel module dependency `wheelos_core` (`@core//...`).
Use the unified runtime entrypoint after building with the module dependency already resolved:

```bash
source setup.bash
```

In managed Docker containers, the login shell sources `setup.bash`
automatically after the container user and Bazel output ownership are
initialized.

If you intentionally need to validate a local source checkout, set the override explicitly and keep it temporary:

```bash
export APOLLO_CORE_ROOT=/path/to/core
source setup.bash
```

Then build the cyber runtime targets from this workspace:

```bash
./apollo.sh build cyber
```

This builds the stable `@core//cyber` runtime surface: `cyber_core`, `mainboard`, and the `cyber_tools` binaries used by the repo.

## Selected product builds

Inside the managed development container, select a product instead of recursively
building every module:

```bash
bash apollo.sh profiles
bash apollo.sh build --product=devkit-x86 --show
bash apollo.sh build --product=devkit-x86
```

For a runtime product declared with `runtime_product`, use the same label for
`build`, `run`, `install` and `package`, for example
`bash apollo.sh run --product=//products/example:runtime`. A single component is
a one-module product; composite products use the same launcher and relocatable
configuration/tool/asset layout. Example labels must be replaced with actual
product definitions. See [runtime products](tools/products/README.md#runtime-products-one-component-or-many).

`--show` displays the effective Bazel configuration and outputs without compiling.
See [product builds](tools/products/README.md) for custom hardware/module selection
and resource limits. The presets are example build inventories, not qualified
runtime deployments. Legacy commands remain available; `build --all` explicitly
requests a full build.

---

## Copyright and License

Apollo-Lite is licensed under the [Apache License 2.0](LICENSE). Please comply
with the license terms when using or contributing to this project.

---

## Connect with Us

- ⭐ Star and Fork to support the project!
- 💬 Join our [community discussion group](http://apollo.auto/community) to chat
  with developers.
- 📧 For collaboration or business inquiries, contact: daohu527@gmail.com

---

Thank you for being part of Apollo-Lite's journey towards autonomous driving
innovation!
