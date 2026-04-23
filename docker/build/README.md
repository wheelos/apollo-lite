# Apollo Docker Image Build Guide

## Overview

Apollo runs within Docker containers, with images tailored for development and
deployment. We currently support `x86_64` and `aarch64` architectures.

**Image Types:**

- **Cyber (Base/Cyber)**: Core CyberRT framework, ideal for CyberRT-focused
  development.
- **Dev**: Full Apollo project with development toolchain, for building and
  running the entire Apollo stack.
- **Runtime**: Optimized, minimal image for production deployment.

## Quick Start

Use the `./build_docker.sh` script to build images.

### 1. Build CyberRT Image

Builds the base development environment including CUDA/CuDNN/TensorRT and the
CyberRT framework.

```bash
cd docker/build

# Build x86_64 CyberRT image (default: download pre-built dependencies)
./build_docker.sh -f cyber.x86_64.dockerfile

# For users in mainland China (accelerated mirrors)
./build_docker.sh -f cyber.x86_64.dockerfile -g cn

# Build all dependencies from source (takes longer)
./build_docker.sh -f dev.x86_64.cpu.dockerfile -m build
```

### 2. Build Apollo Dev Image

Builds the full Apollo development image, based on a CyberRT image.

```bash
# Build x86_64 Dev image
./build_docker.sh -f dev.x86_64.dockerfile

# Build aarch64 Dev image (ensure qemu-user-static is configured for cross-arch builds)
./build_docker.sh -f dev.aarch64.dockerfile -m download
```

### 3. Build Apollo Runtime Image

Used for lightweight production deployments. Requires a prior Apollo Release
Build.

```bash
# 1. Generate required APT packages from Apollo Release Build
./apollo.sh release -c -r

# 2. Navigate to the Docker build directory and copy package list
cd docker/build
cp /apollo/output/syspkgs.txt .

# 3. Build the Runtime image (x86_64 only currently)
# cp runtime.x86_64.dockerfile.sample runtime.x86_64.dockerfile # if file doesn't exist
bash build_docker.sh -f runtime.x86_64.dockerfile
```

---

## Script Arguments

Run `./build_docker.sh --help` for full options:

```
Usage:
    build_docker.sh -f <Dockerfile> [Options]

Options:
    -f, --dockerfile   Path to the Dockerfile (e.g., 'cyber.x86_64.dockerfile').
    -c, --clean        Disable Docker build cache (--no-cache=true).
    -m, --mode         Installation mode: 'download' (default, use pre-built), 'build' (build from source).
    -g, --geo          Enable geo-specific mirrors ('cn' or 'us', default 'us').
    -t, --timestamp    Timestamp of the previous stage image (YYYYMMDD_HHMM).
    --dry              Dry run (print commands without execution).
    -h, --help         Show help message and exit.
```

---

## Advanced Tips

### Cross-Architecture Builds

For cross-architecture builds (e.g., building `aarch64` on `x86_64`), ensure
`qemu-user-static` is configured:

```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

**Recommendation:** Utilize `docker buildx` for a more robust and efficient
multi-platform build experience.

### Optimize Build Speed

- **Local HTTP Cache:** Setting up a local HTTP server to cache downloaded
  packages can significantly speed up repeated builds without affecting final
  image size.

  **Steps:**

  1.  **Download prerequisites:** Identify and download all necessary
      prerequisite packages to a local directory (e.g., `$HOME/apollo_cache`).
      URLs are typically listed in the build log (`/opt/apollo/build.log`). Pay
      attention to checksums for integrity.
  2.  **Start local HTTP server:** Navigate to your cache directory and start a
      simple HTTP server on port **8388**:
      ```bash
      mkdir -p "$HOME/apollo_cache" && cd "$_"
      nohup python3 -m http.server 8388 &
      ```
  3.  **Rerun build:** Execute `build_docker.sh` as usual. The Docker build
      process will attempt to fetch packages from
      `http://host.docker.internal:8388` (or directly from
      `http://<your_host_ip>:8388` if `host.docker.internal` is not available),
      greatly reducing download times.

  - **Benefit:** Even if a cached package is missing or broken, the build
    process will fall back to downloading from the original URL.

- **Avoid `--no-cache` (`-c`):** Only use when strictly necessary, as it forces
  re-downloading and re-building all layers, significantly increasing build
  time.

### Debugging Builds

Build logs are located at `/opt/apollo/build.log` inside the container,
providing detailed information on downloads and installation steps.

---
