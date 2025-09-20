# Welcome to Apollo-Lite's GitHub Page!

[Apollo-Lite](https://github.com/wheelos/apollo-lite) is a high-performance,
flexible architecture designed to accelerate the development, testing, and
deployment of Autonomous Vehicles.

---

> We choose to go to the moon in this decade and do the other things, not
> because they are easy, but because they are hard.
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
- [Connect with Us](#connect-with-us)

---

## Introduction

Apollo-Lite provides powerful modules and features for autonomous driving
development. Before getting started, please ensure your environment meets the
prerequisites and follow the installation instructions below. For a deeper
understanding, refer to the
[architecture overview](http://apollo.auto/docs/architecture_overview.html) to
learn more about the core technologies and platform design.

---

## Prerequisites

- **Machine:** Minimum 8-core CPU, 8GB RAM
- **GPU:** NVIDIA Turing GPU recommended for acceleration
- **Operating System:** Ubuntu 20.04 LTS

---

## Installation

> Detailed installation steps and scripts are provided for a smooth setup.

---

## Quick Start

**Note:** For quick startup and verification, only the CPU-based image
(simulation planning module) is provided. The full GPU-dependent tutorial will
be released later. We recommend starting with the CPU image because the GPU
image is large and has complex dependencies, which may not be suitable for
beginners.

### 1. Install Deployment Tool

```bash
pip install whl-deploy
```

### 2. Setup Host Environment

Run the following scripts to prepare your host machine. These steps will:

1. Install Docker (checks if already installed)
2. [skip] Install NVIDIA Container Toolkit (checks if already installed, depends
   on Docker)
3. [skip] Perform host system configurations

```bash
whl-deploy setup docker
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

Set environment variables:

```bash
source cyber/setup.bash
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
