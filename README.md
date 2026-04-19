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
whl start

# enter the dev container (starts it if needed)
whl enter
```

### Notes about interactive system configuration
- `config_system.sh` (invoked by `setup_host.sh`) is interactive: for each optional system tuning step (NTP/ptp/udev/uvcvideo/CAN/Jetson tuning/headless/autostart) it will ask you whether to apply it (Y/n). Hardware-related tuning defaults to conservative choices.

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
- Use mode-scoped keys such as `DEV_CONTAINER_NAME`, `DEV_USE_GPU`, `DEV_BAZEL_CACHE_DIR`, `TEST_CONTAINER_NAME`, `TEST_SERVER_PORT`, `TEST_CPUS`, `TEST_MEMORY`, `TEST_USE_GPU`, and `TEST_BAZEL_CACHE_DIR` so dev and test stay isolated and explicit.
- Generated env files used by `whl` are `docker/.env.dev.local`, `docker/.env.test.local`, and `docker/.env.prod.local`. `whl` regenerates the requested mode file on every run from `.env.global` plus host auto-detection before launching.
- Each mode also gets its own Compose project name, so `dev` and `test` can run side by side without recreating or stopping each other.
- `whl start <mode>` and `whl stop <mode>` are symmetric. Use `whl stop all` to tear down all managed modes.
- Host-ready marker: `/etc/wheelos_setup_host.done`
- `whl` helper location: `/usr/local/bin/whl`

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
