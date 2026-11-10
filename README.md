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

Apollo-Lite prepares the development environment, integrates hardware and
algorithm repositories, and assembles, packages, and publishes products. The
runtime, hardware interfaces, and algorithms remain owned by the independent
repositories listed in [`wheelos.repos`](wheelos.repos); this workspace owns
their product-level configuration and integration.

Use the guide below to find each capability's owning repository. Module names
are examples, not necessarily separate repositories, and the groupings do not
imply that repositories are merged or depend on one another.

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
repository.

---

## Prerequisites

- **Machine:** Minimum 8-core CPU, 8GB RAM
- **GPU:** NVIDIA Turing GPU recommended for acceleration
- **Operating System:** Ubuntu 20.04 LTS

---

## Host setup and container startup (quick guide)

Prepare the host once. This installs or checks Docker and the NVIDIA Container
Toolkit, installs `whl`, and prompts for optional system configuration:

```bash
sudo bash docker/setup_host/setup_host.sh
```

Start and enter the development container:

```bash
whl start dev
whl enter dev
```

`whl start` uses an image already available locally; if needed, run
`whl pull dev` first (it may recreate the container). `whl enter dev` starts
the container if necessary. Use `whl stop dev` to stop it without removing it.

Overrides belong in `.env.global`. Run `whl --help` for available commands and
options; see [Host Setup](docker/setup_host/README.md) for host configuration.

---

## Product integration and delivery

The Core repository provides the Cyber runtime; Apollo-Lite consumes it through
`@core` and integrates it with selected hardware and algorithm modules. For
Core runtime and tool usage, see the [Core repository](https://github.com/wheelos/core).

Inside the managed development container, inspect and build a product selection:

```bash
bash apollo.sh profiles
bash apollo.sh build --product=devkit-x86 --show
bash apollo.sh build --product=devkit-x86
```

Use `--show` to review the selected configuration before compiling. For
hardware/module selection and product `build`, `run`, `install`, and `package`
workflows, see [Product Builds](tools/products/README.md). The `devkit-*`
presets are build inventories, not release-qualified products.

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
