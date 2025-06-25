# Welcome to Apollo's GitHub page!

[Apollo-Lite](https://github.com/wheelos/apollo-lite) is a high-performance, flexible architecture accelerating the development, testing, and deployment of Autonomous Vehicles.

---
> We choose to go to the moon in this decade and do the other things,
> not because they are easy, but because they are hard.
>
> -- John F. Kennedy, 1962
---

## Table of Contents

- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Build & Compile](#build--compile)
- [Copyright and License](#copyright-and-license)
- [Connect with us](#connect-with-us)

---

## Introduction

Apollo is packed with powerful modules and features designed for autonomous driving development. Before you take it for a spin, please ensure your environment is properly calibrated and configured by following the prerequisites and installation instructions below.
For a deeper dive, check out Apollo's [architecture overview](http://apollo.auto/docs/architecture_overview.html) to understand its core technologies and platform design.

---

## Prerequisites

- **Machine:** 8-core CPU, 8GB RAM minimum
- **GPU:** NVIDIA Turing GPU strongly recommended for acceleration
- **Operating System:** Ubuntu 20.04 LTS

---

## Installation

> Detailed installation steps and scripts are provided for ease of setup.

---

## Quick Start

### 1. Setup Host Environment
Run the following scripts in order to prepare your host machine, which will perform the following steps sequentially:
1. Install Docker (checks if already installed, then proceeds)
2. Install NVIDIA Container Toolkit (checks if already installed, depends on Docker)
3. Perform host system configurations

```bash
# setup host
bash docker/setup_host/setup_host.sh
```

### 2. Start Docker Container
Download and start the Apollo container image (only needs to be done once):

```bash
# docker pull & start
bash docker/scripts/dev_start.sh
```

Enter the running container environment in subsequent sessions with:

```bash
# docker into
bash docker/scripts/dev_into.sh
```

### 3. Build Apollo

Inside the container, install build dependencies:

```bash
# update bazel & gcc
sudo bash docker/build/installers/install_bazel.sh
sudo bash docker/build/installers/install_gcc.sh
```

> **Note:** Current container image does not have the latest Bazel and GCC. You need to manually install/upgrade them. Future container versions will include these by default.

Build the entire Apollo project with:

```bash
./apollo.sh build
```

To build a single module, use:

```bash
./apollo.sh build <module_name>
# example:
./apollo.sh build planning
```

---

## Copyright and License

Apollo is licensed under the [Apache License 2.0](LICENSE). Please adhere to the licensing terms when using or contributing to this project.

---

## Connect with us

- ⭐ Star and Fork to support the project!
- 💬 Join our [community discussion group](http://apollo.auto/community) to chat with developers.
- 📧 For collaboration or business inquiries, contact: daohu527@gmail.com

---

Thank you for being part of Apollo's journey towards autonomous driving innovation!
