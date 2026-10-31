---
name: "container-build"
description: "Build, check, and publish Apollo-Lite WheelOS development container images with Docker Buildx Bake."
---

# Container Image Build

## When

Use this skill to build, check, or publish WheelOS development container
images. It covers image construction only; compiling Apollo source inside a
running image is covered by [Build](../build/SKILL.md).

## Scope and prerequisites

- Run commands from the repository root.
- Docker Buildx Bake constructs the WheelOS images; it does not compile Apollo
  source.
- Supported targets:
  - `dev-amd64-cpu-u22`: x86_64 CPU, Ubuntu 22.04.
  - `dev-amd64-cuda-u22`: x86_64 GPU, CUDA 12.8 / Ubuntu 22.04.
  - `dev-orin-jp621-l4t3643`: Orin, JetPack 6.2.1 / L4T 36.4.3.
- The default Bake group builds x86 CPU and GPU. Build Orin on a native
  Orin/JetPack 6.2.1 host; QEMU cannot validate Jetson device-tree or GPU
  runtime requirements.

## Workflow

1. Inspect the Bake graph before an unfamiliar or production build:

   ```bash
   docker buildx bake -f docker/build/docker-bake.hcl --print
   ```

2. Use `docker/build/build_docker.sh` for safe Dockerfile checks and normal
   local builds. Supported legacy Dockerfile names include:

   ```bash
   bash docker/build/build_docker.sh -f dev.x86_64.cpu.dockerfile --check
   bash docker/build/build_docker.sh -f dev.x86_64.cpu.dockerfile
   bash docker/build/build_docker.sh -f dev.x86_64.cuda.dockerfile
   bash docker/build/build_docker.sh -f dev.aarch64.orin.dockerfile --geo cn
   ```

   The wrapper selects one exporter: local Docker image store for local builds,
   registry for pushes, and cache-only for checks. Use `--check` rather than a
   raw Dockerfile check that could export an incomplete image to a local tag.

3. Select the registry and package mirrors with `--geo cn` (Hangzhou registry
   and China mirrors) or `--geo us` (Docker Hub and public package sources).
   `APOLLO_REPO` overrides the image repository.

4. For a versioned validation build or release push, provide
   `--release-version`. Add `--push` only when publishing:

   ```bash
   bash docker/build/build_docker.sh -f dev.x86_64.cpu.dockerfile \
     --geo cn --release-version v1.2.3
   bash docker/build/build_docker.sh -f dev.x86_64.cuda.dockerfile \
     --geo cn --release-version v1.2.3 --push
   ```

   Keep mutable `dev-*` tags for development. Do not overwrite published
   version tags; promote the exact validated image digest instead of rebuilding
   for each environment.

5. Reuse the default Buildx cache. `--clean` is for cache diagnosis, not
   routine builds. BuildKit layer cache, HTTP artifact cache, and Bazel caches
   are separate; `--cache-server URL` selects the HTTP artifact cache and
   `CACHE_REGISTRY=...` configures shared BuildKit registry cache.

## Acceptance

- The requested target builds or passes the wrapper's `--check`.
- Confirm the intended platform and versioned tag before publishing.
- A successful Orin cross-build is not proof of Jetson device-tree or GPU
  runtime correctness; require native Orin validation for those claims.

## Failure handling

- Stop at the first actionable build or check failure. Inspect its output
  before changing options or retrying; do not disable cache or change
  registries as a blind workaround.
- Do not claim Orin runtime validation from QEMU or x86 builds.
- Do not publish mutable development tags as releases or overwrite published
  version tags.

## Source of truth

- [`docker/build/README.md`](../../../docker/build/README.md)
- `docker/build/docker-bake.hcl`
- `docker/build/build_docker.sh`
