# WheelOS Developer Images

Docker Buildx Bake builds the final WheelOS development images directly from
their platform bases. It does not compile Apollo sources; Apollo builds run
inside the started development container with `apollo.sh`.

## Supported targets

| Bake target | Platform | Base | Image tag |
| --- | --- | --- | --- |
| `dev-amd64-cpu-u22` | x86_64 | Ubuntu 22.04 | `dev-x86_64-22.04-cpu` |
| `dev-amd64-cuda-u22` | x86_64 | CUDA 12.8 / Ubuntu 22.04 | `dev-x86_64-22.04-gpu` |
| `dev-orin-jp621-l4t3643` | Orin | JetPack 6.2.1 / L4T 36.4.3 | `dev-orin-jp6.2.1` |

The default Bake group builds the two x86 images. Build Orin as a separate
target on a native Orin/JetPack 6.2.1 builder. QEMU can execute ARM64 userland,
but cannot satisfy Jetson L4T's real device-tree and GPU runtime requirements.

## Build

Run from the repository root. First inspect the selected build graph:

```bash
docker buildx bake -f docker/build/docker-bake.hcl --print
```

Build one target or the default x86 group:

```bash
GEOLOC=cn docker buildx bake -f docker/build/docker-bake.hcl dev-amd64-cpu-u22
GEOLOC=cn docker buildx bake -f docker/build/docker-bake.hcl dev-amd64-cuda-u22
GEOLOC=cn docker buildx bake -f docker/build/docker-bake.hcl dev-orin-jp621-l4t3643
```

Local builds load into the Docker image store and use the mutable `dev-*` tags.
The compatibility wrapper selects exactly one exporter: Docker for local builds
or the registry for pushes. Set `GEOLOC=cn` for the Hangzhou registry
(`registry.cn-hangzhou.aliyuncs.com/wheelos/apollo`) and China package mirrors;
set `GEOLOC=us` for Docker Hub (`wheelos/apollo`) and public package sources.
`APOLLO_REPO` overrides the image repository for either region.

For production validation, provide a release version. The generated tag keeps
the OS, architecture, accelerator, and Jetson runtime distinct:

| Target | Example tag for version `v1.2.3` |
| --- | --- |
| x86 CPU | `v1.2.3-amd64-u22-cpu` |
| x86 GPU | `v1.2.3-amd64-u22-cuda12.8` |
| Orin | `v1.2.3-orin-jp6.2.1-l4t36.4.3` |

Use the wrapper to push a versioned image; without `--push`, the same versioned
tag is built locally for validation:

```bash
bash docker/build/build_docker.sh -f dev.x86_64.cpu.dockerfile \
  --geo cn --release-version v1.2.3
bash docker/build/build_docker.sh -f dev.x86_64.cuda.dockerfile \
  --geo cn --release-version v1.2.3 --push
```

Do not overwrite a published version tag. Configure registry-side tag
immutability where supported, and promote the exact validated image digest
rather than rebuilding separately for each environment. Keep the mutable
`dev-*` tags for development only.

Check a Dockerfile through the wrapper so Buildx does not export a check-only
image over a local tag:

```bash
bash docker/build/build_docker.sh -f dev.x86_64.cpu.dockerfile --check
bash docker/build/build_docker.sh -f dev.x86_64.cuda.dockerfile --check
```

The compatibility wrapper accepts the supported legacy Dockerfile names:

```bash
bash docker/build/build_docker.sh -f dev.x86_64.cpu.dockerfile --dry
bash docker/build/build_docker.sh -f dev.x86_64.cuda.dockerfile
bash docker/build/build_docker.sh -f dev.aarch64.orin.dockerfile --geo cn
```

For direct Bake checks, override the target output with
`--set '<target>.output=type=cacheonly'`. For direct pushes, use
`--set '<target>.output=type=image,push=true'`; do not combine the configured
Docker exporter with `--push`, which adds a second exporter. Prefer the wrapper
for both operations so output behavior stays explicit.

Use `--cache-server URL` for the checksum-verified HTTP artifact cache,
`CACHE_REGISTRY=...` for a shared BuildKit registry cache, and `--set
key=value` to pass a Bake override. Keep the default BuildKit cache enabled;
`--clean` is for cache diagnosis, not routine builds. HTTP artifact cache,
BuildKit layer cache, and Bazel caches are separate mechanisms.

## Build and runtime boundaries

Image construction installs the development toolchain and platform/module
dependencies. It does not run `apollo.sh build`. Start the matching image with
`whl`, then compile in `/apollo` as the mapped non-root user. Buildx cache
reuses image layers; Bazel repository and disk caches persist separately under
the workspace `.cache` directory.
`DEV_BAZEL_CACHE_DIR` can override the Bazel cache root; the build wrapper
uses its `repo_cache` and `disk_cache` subdirectories and fails explicitly if
they are not writable instead of silently falling back to `/tmp`.

`GEOLOC` selects the registry and package mirrors consistently. NVIDIA CUDA
and Jetson package endpoints remain vendor-controlled; only Ubuntu sources
switch between China and public mirrors.
