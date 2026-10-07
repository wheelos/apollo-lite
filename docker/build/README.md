# Apollo developer images

Developer images are built with Docker Buildx Bake. Each published image is
built directly from its platform base stage in the same Dockerfile; there are
no separately published `base` or `cyber` images.

## Build targets

Run Bake commands from the repository root:

```bash
docker buildx bake -f docker/build/docker-bake.hcl --print
docker buildx bake -f docker/build/docker-bake.hcl dev-amd64-cpu-u22
docker buildx bake -f docker/build/docker-bake.hcl dev-amd64-cuda-u22
docker buildx bake -f docker/build/docker-bake.hcl dev-arm64-cpu-u22
docker buildx bake -f docker/build/docker-bake.hcl dev-orin-jp621-l4t3643
```

The first command prints the expanded build graph without building images.
Without `--push`, selected developer images are loaded into the local Docker
image store. Add `--push` to publish them to the registry selected by
`GEOLOC`. The default is `GEOLOC=cn`, which uses
`registry.cn-hangzhou.aliyuncs.com/wheelos/apollo`; `GEOLOC=us` uses
`wheelos/apollo`. Set `APOLLO_REPO` to explicitly override the image
repository for both builds and `whl` startup.

| Bake target | Platform/base | Published tag |
| --- | --- | --- |
| `dev-amd64-cpu-u20` | amd64, Ubuntu 20.04 | `dev-x86_64-20.04-cpu` |
| `dev-amd64-cpu-u22` | amd64, Ubuntu 22.04 | `dev-x86_64-22.04-cpu` |
| `dev-amd64-cuda-u22` | amd64, CUDA 12.8 / Ubuntu 22.04 | `dev-x86_64-22.04-gpu` |
| `dev-arm64-cpu-u22` | ARM64 CPU, Ubuntu 22.04 | `dev-aarch64-22.04-cpu` |
| `dev-orin-jp621-l4t3643` | Jetson Orin, JetPack 6.2.1 / L4T 36.4.3 | `dev-aarch64-orin-jp6.2.1-l4t36.4.3-gpu` |

The x86 CUDA image is based on Ubuntu 22.04/CUDA 12.8. The old x86 Ubuntu
20.04/CUDA 11.8 target was removed because its configured PyTorch 2.10
`cu118` artifact does not exist; x86 GPU hosts use the Ubuntu 22.04 CUDA image.
CPU images retain the x86 Ubuntu 20.04 and 22.04 options.

The base images are referenced by vendor tags rather than digests. This keeps
vendor security updates available but means a tag can resolve to different
content over time. Release builds that require bit-for-bit reproducibility
should pin verified digests and refresh them through an explicit update
process; digest pinning was not changed as part of this static cleanup.

The generic ARM64 CPU image is also the CPU-only image for 64-bit Raspberry Pi
and RK3588 Linux hosts, provided Docker can run `linux/arm64` containers. It
does not include CUDA or board-vendor NPU runtimes. Raspberry Pi Hailo inference
and RK3588 RKNN inference are not supported by this image; each needs a
separate device-specific runtime, driver/API integration, and model format.
Do not use an Orin/L4T image as a generic ARM image: Jetson containers are
coupled to the host JetPack/L4T stack. Orin GPU image selection checks
`/etc/nv_tegra_release` and accepts only L4T 36.4.3.

## Runtime image selection

`whl` selects from the built developer-image matrix. The default container
Ubuntu is 22.04 and does not follow the host Ubuntu release. x86_64 CPU may
explicitly request Ubuntu 20.04 or 22.04; x86_64 CUDA and generic ARM64 CPU
require Ubuntu 22.04. ARM64 GPU selection is independent of the generic Ubuntu
choice and requires a Jetson host reporting L4T 36.4.3. Unsupported
architecture/version/accelerator combinations fail instead of silently
selecting a different image. Use `--image` or the mode-specific
`*_APOLLO_IMAGE` override only when intentionally selecting a custom image.
Buildx and `whl` resolve the same `APOLLO_REPO` from `GEOLOC` and use the full
image reference: locally loaded images are checked under that exact tag
before any registry pull. `GEOLOC` consistently selects the image registry,
Ubuntu APT mirror, and Python package index.

For cross-architecture builds, Buildx may use QEMU where configured. Prefer
native amd64 and Orin builders for package installation and compilation.
QEMU build success does not validate GPU operation; run the Orin GPU/TensorRT
smoke test on an Orin device with the matching host L4T release.

## Legacy wrapper

`build_docker.sh` remains as a compatibility wrapper for older developer
commands:

```bash
bash build_docker.sh -f dev.x86_64.u22.dockerfile --dry
bash build_docker.sh -f dev.x86_64.u22.dockerfile --push
bash build_docker.sh -f dev.aarch64.l4t.dockerfile \
  --cache-server http://<reachable-cache-host>:8388
```

The wrapper maps old developer Dockerfile names to Bake targets. All old
`base.*` Dockerfiles are replaced by the non-published `base` stage inside each
direct developer target; standalone base outputs are no longer produced.
Cyber Dockerfile aliases fail because cyber images were intentionally removed.
The old Xavier NX alias fails rather than selecting an incompatible Orin image.
Generic ARM CUDA also fails; generic ARM is CPU-only. These removed outputs
are not artifact-compatible replacements for consumers that pulled the
standalone base or cyber tags. New CI and developer automation should invoke
Bake targets directly. The legacy `--mode` option was removed because none of
the active installers consumed it; use the fixed pinned/downloaded inputs
declared by the target and installer scripts.

## Package and build caches

Three caches serve different purposes:

1. **HTTP artifact cache** serves installer downloads such as LibTorch and
   source archives. The aarch64 CPU target uses the pinned official PyTorch
   2.10 CPU wheel; its wheel filename is the cache key. Point `LOCAL_HTTP_ADDR`
   or `--cache-server` at a reachable
   read-only HTTP cache. The installer default is
   `http://172.17.0.1:8388`; BuildKit/remote builders may need a host-reachable
   address instead. Downloaded artifacts continue to be checked against
   SHA256 values in the installers. The Orin LibTorch wheel is a custom
   aarch64 artifact and must be present in this cache under the exact filename
   and checksum declared by `install_libtorch.sh`; a missing or invalid wheel
   fails the Orin image build instead of triggering a lengthy source build.
   The old hard-coded private HTTP host is no longer used.
2. **BuildKit layer cache** is retained by the selected builder. For shared CI
   cache, set `CACHE_REGISTRY` to a registry namespace where the builder can
   read and write cache tags. Cache references are separated by platform and
   image variant.
3. **Bazel cache** remains configured by the Apollo workspace in `.bazelrc`.
   It is independent of both HTTP artifact caching and the Docker layer cache.

Example shared cache configuration:

```bash
CACHE_REGISTRY=registry.example.com/team docker buildx bake \
  -f docker/build/docker-bake.hcl dev-amd64-cuda-u22
```

Do not routinely use `--no-cache`; it discards reusable image layers and can
cause package/source downloads to run again. Use it only to diagnose stale
layers.

## Source selection

Set `GEOLOC=cn` to use the Hangzhou image registry and version- and
architecture-specific Tsinghua Ubuntu sources for APT, Tsinghua PyPI for
Python, and `registry.npmmirror.com` for npm/Yarn when those tools are present.
For example:

```bash
GEOLOC=cn docker buildx bake -f docker/build/docker-bake.hcl dev-amd64-cuda-u22
```

The China APT lists use Tsinghua for security updates as well as main, updates,
and backports. Orin's NVIDIA Jetson APT source remains separate and verifies
packages with NVIDIA's published signing key. Node.js/npm/Yarn are not
installed by the current developer-image graph, so registry configuration
does not currently shorten that image build. Docker Hub, CUDA, and NVIDIA base
image registries are vendor/distribution endpoints, not Ubuntu mirrors; retain
their trust roots. Configure a Docker/BuildKit registry mirror at the
builder/daemon level when appropriate rather than rewriting vendor image
references in Bake.

`GEOLOC=us` selects the public image repository, version-specific US Ubuntu
sources, official PyPI, and `registry.npmjs.org` for npm/Yarn when present.
Do not fall back from Jammy/Focal to the generic Bionic source list. The
TypeScript archive recorded in `MODULE.bazel.lock` is fetched by Bazel directly
from `registry.npmjs.org`; npm's registry setting does not redirect Bazel's
HTTP downloader. Use the configured Bazel repository/dist caches for repeat
builds, or separately configure a verified Bazel downloader mirror when one
is required.

## Download and build speed

- The two dependency-install `RUN` steps use a BuildKit pip-cache mount.
  `pip3_install` leaves pip caching enabled so retries and dependent pip
  installs on the same builder can reuse downloads without storing that cache
  in the image layer.
- `download_if_not_cached` uses `LOCAL_HTTP_ADDR` for selected checksum-pinned
  archives and wheels. Git clone paths bypass this cache, and other direct
  downloads (including NVIDIA signing metadata and Bazel module archive URLs)
  use their upstream endpoints.
- Registry-backed BuildKit layer cache is opt-in via `CACHE_REGISTRY`; without
  it, layer reuse is limited to the selected builder's retained cache. Keep
  cache references isolated by target/platform and reuse native builders.
- Bazel repository/dist caches are separate from BuildKit and artifact caches.
  Reuse the existing `.cache` mounts in the managed container rather than
  redownloading Bzlmod repositories.
- The `install_ordinary_modules.sh --all` step is the main source-build /
  compile hotspot. A changed input to its Docker layer can rerun many native
  dependency builds; keep frequently changing inputs out of earlier layers
  and avoid `--no-cache` except for diagnosis.
- APT metadata is refreshed by each `apt_get_update_and_install` call. This
  creates avoidable network work across installer groups; consolidating APT
  operations or adding an APT cache mount is a separate optimization and must
  preserve index freshness and package-manager locking.
- The current image does not install Node/Yarn or run npm installs. npm
  registry configuration therefore does not affect this image build.

## Dependency ownership

Bzlmod dependencies should be built through Bazel rather than duplicated by
installer scripts when Bazel owns the dependency. Obsolete standalone
installers for OSQP, PROJ, and other unused legacy dependencies have been
removed; the workspace resolves OSQP and PROJ through Bzlmod. FFmpeg and
LibTorch remain installed because their Bzlmod extensions expose local
installation directories (`/opt/apollo/sysroot` and `/usr/local/libtorch`)
rather than downloading those libraries. The shared image installer owns
FFmpeg once; module dependencies do not invoke it again. `install_ordinary_modules.sh
--all` owns module-scoped Dreamview and driver dependencies, so the Dockerfile
does not call those installers separately. The planning module owns LibTorch
installation for both CPU and GPU images, receiving the GPU variant through
`GPU_SUPPORT`; the later GPU-support step installs only its additional system
libraries. Perception installs `liblz4-dev` and `libleveldb-dev` for targets
that link against those system libraries. The old cyber-dependency wrapper was removed;
the pinned Protobuf release installer remains an explicit image step to
provide `protoc` and the Python package used by integration tools. The
separate source-building Protobuf installer is unused and was removed.

Do not remove an installer solely because a similarly named `bazel_dep` or
module extension exists. Check whether it supplies system tools, Python
packages, runtime shared libraries, or a local path consumed by a repository
rule before changing the image contract.

## Build context and resources

The image build context is the repository root so the image uses the
workspace's `.bazelversion`. `dev.dockerfile.dockerignore` allowlists only that
version file, installer scripts, and rcfiles; local PyPI/APT mirror files and
private signing-key files are excluded. Add small, versioned immutable
application assets explicitly to the image. Keep mutable or large data such as
maps, calibration, logs, recordings, and models in runtime mounts or separately
versioned artifacts. Never bake credentials into build arguments, environment
variables, or image layers.
