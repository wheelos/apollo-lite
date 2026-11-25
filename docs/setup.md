# Host setup and container lifecycle

Use the managed development container for repository builds, tests and lint.
The host scripts prepare Docker and system integration; they do not start the
autonomous-driving runtime.

## Prerequisites

- Ubuntu 22.04 host;
- Docker with the NVIDIA Container Toolkit for GPU work;
- at least 8 CPU cores and 8 GB RAM recommended;
- an NVIDIA Turing or newer GPU when selecting the GPU development image.

## Prepare the host

The recommended interactive setup installs Docker, the NVIDIA container
toolkit and optional system integrations:

```bash
sudo bash docker/setup_host/setup_host.sh
```

For unattended runs that accept conservative defaults:

```bash
sudo bash docker/setup_host/setup_host.sh < /dev/null
```

Review [`config_system.sh`](../docker/setup_host/config_system.sh) before using
the fully automated `yes` pipeline. Optional udev, CAN, clock, jetson, headless
and boot-service changes affect the host outside this repository.

After successful setup, the readiness marker is:

```text
/etc/wheelos_setup_host.done
```

## Container lifecycle

The `whl` helper is the public container entrypoint:

```bash
whl start dev
whl enter dev
whl exec dev -- bash -lc 'python3 --version'
whl stop dev
whl rm dev
whl pull dev
```

Useful semantics:

- `start` uses an already available image and waits for initialization;
- `enter` starts the container when needed and opens a shell;
- `stop` preserves the container;
- `rm` removes the container and its Compose network but preserves bind-mounted
  host data;
- `pull` updates the configured image and recreates the matching container.

Development and test modes use separate Compose projects and generated
environment files. User overrides belong in the project-root `.env.global` with
mode-scoped keys such as `DEV_USE_GPU` and `DEV_BAZEL_CACHE_DIR`.

## Optional host-local Bazel remote cache

On the Docker host, start the shared cache separately from `whl`:

```bash
bash scripts/start_bazel_remote.sh
docker logs --tail 20 bazel-remote
```

The script publishes HTTP only on `127.0.0.1:19090`, stores cache data under
the invoking user's `~/.cache/bazel-remote`, and limits it to 5 GiB by default.
Use a dedicated directory, not Bazel's existing repository or disk cache
directory. Run `bash scripts/start_bazel_remote.sh --help` for overrides.
Docker may pull the pinned image if it is not already available. An existing
container with the same name is never replaced; use `docker start bazel-remote`
to restart a stopped container after inspecting it.

The loopback service has no authentication: all local users able to reach it
can read and write cached artifacts. Use it only with mutually trusted users.
Containers on the same Docker network may also reach its internal HTTP port.
The dev container uses host networking; the test container does not, so its
`127.0.0.1` does not reach this service. Repository and local disk caches
remain separate. Configure clients separately and verify their effective
settings with `--announce_rc`.

## System services

`whl-container.service` manages only the production container.
`whl-dreamview.service` is an optional application unit and depends on the
container unit. Service start does not establish autonomous-driving readiness.

Inspect service state with:

```bash
systemctl status whl-container.service
systemctl status whl-dreamview.service
```

## Next steps

After entering the development container:

```bash
bash apollo.sh profiles
bash apollo.sh build --product=devkit-x86 --show
```

See [Product builds](../tools/products/README.md) and the
[documentation map](README.md) for build, runtime and delivery guidance.
