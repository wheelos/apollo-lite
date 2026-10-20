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

## Dependency ownership

Bzlmod downloads and builds the C++ dependencies declared by `MODULE.bazel`
(including Boost, Protobuf/gRPC, Eigen, OpenCV, PCL, PROJ, OSQP, and the lidar
SDKs). Do not install matching Ubuntu `-dev` packages just to satisfy those
labels. System packages below are deliberate exceptions or image tools:

| Consumer | Image supplier | Notes |
| --- | --- | --- |
| Bazel/Bazelisk | `install_bazel.sh`, `.bazelversion` | Reuses the workspace Bazel version |
| Buildifier | `install_buildifier.sh` | Version 10.1.0, SHA256-verified amd64/arm64 binaries; CI uses the same installer |
| C++/shell/Python lint | `install_llvm_clang.sh`, `install_minimal_environment.sh`, `py3_requirements.txt` | Clang-Format, ShellCheck, and the CI-pinned Black/isort/Flake8 versions |
| FFmpeg | `install_modules_base.sh` → `install_ffmpeg.sh` | Source installation under `/opt/apollo/sysroot`, including headers and shared libraries |
| GStreamer/GLib | `install_minimal_environment.sh`, `install_drivers_deps.sh` | `pkg-config`, core/app/video development packages, and runtime base/good/bad plugins |
| pcap/OpenSSL | `install_drivers_deps.sh` | Explicit driver development dependencies; do not rely on a GPU base's transitive OpenSSL installation |
| Qt/OpenGL | `install_visualizer_deps.sh` → `install_qt.sh` | x86 Qt 5.12.9 SDK; Orin uses Ubuntu Qt with `/usr/local/qt5/include` mapped to system headers |
| IPOPT | planning → `install_ipopt.sh` | `coinor-libipopt-dev` |
| LibTorch | planning/perception/prediction → `install_libtorch.sh` | Module installer deduplicates the shared script; CPU/CUDA/Orin artifacts differ |
| LZ4/LevelDB | perception → `install_perception_deps.sh` | Used by existing `-llz4`/`-lleveldb` targets |
| CUDA/cuDNN/TensorRT | CUDA or Jetson base plus `dev.dockerfile` | Not installed in the CPU image; GPU targets require a matching GPU image |
| NVIDIA spconv | Explicit external SDK delivery | Not installed by the image; supply the compatible library via `--repo_env=APOLLO_NVIDIA_SPCONV_LIBRARY=/path/to/libspconv.so` |
| Protoc/Python Protobuf | `install_protobuf_release.sh` | Image tooling/runtime, separate from Bazel's C++ Protobuf build |
| Python tools | `install_python_modules.sh` | Includes GEOS/HDF5 development packages for Python dependencies, not Bazel C++ replacements |
| Packaging | `install_release_deps.sh` | patchelf |

The active image no longer invokes the old Dreamview installer (which only
installed `nasm`) or builds a separate OpenH264 2.1.1 SDK: no active Bazel
consumer links that SDK, FFmpeg enables other codecs, and GStreamer's Ubuntu
plugins bring their own packaged runtime dependencies. Repeated apt package
names are idempotent, not additional library installations. Some overlap remains
intentional: Python extension builds need system headers, system Qt is separate
from Bazel's C++ dependencies, and the ARM LibTorch installer needs OpenBLAS even
when run separately. The ARM Qt installer still lists source-build-style `-dev`
packages despite installing binary Qt packages; trimming that list needs native
Orin validation.
The GPU installer currently requests OpenBLAS, ATLAS, and LAPACK; these are
overlapping providers, not evidence that one is safely removable for every
external SDK. Do not change BLAS alternatives without platform validation.

The optional `FFMPEG_BUILD_MODE=package` path only installs the FFmpeg executable;
it is not a replacement for the development SDK consumed by `@ffmpeg`.
Python runtime requirements other than the lint tools remain largely unpinned,
and `grpcio-tools` may change the Python Protobuf version. The image's `protoc`
version alone therefore does not prove Python runtime compatibility.

## Local SDK configuration

The SDK repositories are local configuration repositories, not hermetic SDK
downloads. Use them inside the matching development image. Version headers are
read through Bazel's repository API; cuDNN/TensorRT no longer guess a version
when their headers are missing or malformed. Missing GPU SDKs fail only when
those repositories are requested, rather than producing empty repositories.

FFmpeg and Qt expose their selected shared libraries as `cc_import` inputs.
`APOLLO_SYSROOT_DIR` selects the FFmpeg installation prefix, and
`APOLLO_QT5_ROOT` selects Qt headers, libraries, and moc/uic/rcc together.
GStreamer's `APOLLO_SYSROOT_DIR` is also an installation prefix, **not**
`PKG_CONFIG_SYSROOT_DIR`. The latter is reserved for an actual sysroot.
GStreamer requires working `.pc` metadata for `gstreamer-app-1.0` and
`gstreamer-video-1.0`; missing metadata and invalid include paths are errors,
not a reason to fall back to guessed linker flags.

Pass local configuration explicitly with `--repo_env`. GStreamer tracks
`PATH`, `PKG_CONFIG_PATH`, `PKG_CONFIG_LIBDIR`, `PKG_CONFIG_SYSROOT_DIR`, and
`APOLLO_SYSROOT_DIR`, and executes pkg-config with an otherwise cleared
environment. `PKG_CONFIG_PATH` entries take precedence over prefix paths.
Unset `PKG_CONFIG_LIBDIR` preserves pkg-config's built-in system search path.
Native `/usr` installations are not prefixed again as `/usr/usr`.

For a container already running from an older image, install the pinned
Buildifier without rebuilding the image:

```bash
sudo bash docker/build/installers/install_buildifier.sh
```

Run source builds and checks afterward as the mapped non-root user, not through
sudo. The GStreamer app/video SDK smoke test is
`bazel test --config=dev //third_party/gstreamer:local_sdk_test`.
Source/header lists in first-party C++ targets are explicit; resource/test-data
globs and SDK header globs remain intentional. Buildifier checks include
`MODULE.bazel`, `BUILD.bazel`, and SDK BUILD templates, but exclude `.bazelrc`,
which is not Starlark.

## Visualizer display

`cyber_visualizer` requires a working X11 connection and OpenGL 3.3 or newer.
Mounting `/tmp/.X11-unix` does not create a display server: `DISPLAY=:0` works
only if that display exists and authorizes the mapped container user. For an
interactive desktop, set `WHL_DISPLAY` to your own graphical session's `DISPLAY`
before starting the dev container and provide the corresponding X11
authorization. Do not select another user's display or disable access control.

For headless startup verification, the visualizer installer includes Xvfb,
xauth, Mesa software rendering, and `glxinfo`. Run inside dev as the mapped user:

```bash
source /apollo/setup.bash
xvfb-run -a -s '-screen 0 1280x800x24 +extension GLX -nolisten tcp' \
  env LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa \
  /apollo/bazel-bin/modules/tools/visualizer/cyber_visualizer
```

This explicitly uses a private, authenticated virtual display and CPU rendering;
it does not change the container's desktop `DISPLAY` or validate NVIDIA
rendering or desktop forwarding. Stop the foreground command when finished.
To inspect the software OpenGL capabilities, run the same command with
`glxinfo -B` in place of the visualizer binary.
