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

## Quick Start

We recommend using the **CPU version for quick experience and verification**.
- The CPU-based image is smaller and has simpler dependencies, making it suitable for beginners and basic planning modules.
- The **GPU version** has a larger image size and more complex dependencies, and is more appropriate for **deep development scenarios** such as deep learning and GPU-accelerated workflows.

---

### 1. Install Deployment Tool

```bash
pip install whl-deploy
```

`whl-deploy` is a tool that simplifies deployment of WheelOS environments using **bundle packages** or **`manifest.yaml`**. For detailed usage, ref [docs](https://github.com/wheelos-tools/whl-deploy/blob/main/README.md)

---

### 2. Setup Host Environment

Run the following to prepare your host machine:

```bash
whl-deploy run
```

### 3. Start Docker Container

Download and start the Apollo container image (only required once):

```bash
bash docker/scripts/dev_start.sh -d testing
```

To enter the running container environment in subsequent sessions:

```bash
bash docker/scripts/dev_into.sh
```

### 4. Build Apollo

To build the entire Apollo project:

```bash
./apollo.sh build_cpu
```

To build a specific module:

```bash
./apollo.sh build_cpu <module_name>
# Example:
./apollo.sh build_cpu planning
```

#### Notes and Troubleshooting

- **Out of Memory (OOM) Issues:** If the build process is terminated due to
  insufficient memory, try limiting the number of CPU threads used during the
  build:

  ```bash
  ./apollo.sh build_cpu dreamview --cpus=2
  ```

- **Slow Download Speeds:** If you experience slow downloads, you can manually
  download the required archive from the following link:
  [Caiyun Cloud Drive](https://caiyun.139.com/w/i/2oxwFbadL3byc) (Extraction
  code: `jfwu`). After downloading, place the archive in the `.cache/distdir`
  directory within your codebase.

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
