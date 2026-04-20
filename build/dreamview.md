## Dreamview Build And Run

- Dreamview frontend source is under `data/frontend`, which is a nested Git repository used for local frontend iteration.
- For Dreamview work, enter the development container from the workspace root with `bash docker/scripts/whl.sh enter`.
- Inside the container, use `./apollo.sh build_opt_gpu` for the GPU-optimized build requested by this workspace.
- Restart the full Monitor + Dreamview stack from the workspace root inside the container with `./scripts/bootstrap.sh restart`.
- `./scripts/bootstrap.sh` starts both Monitor and Dreamview, then performs an HTTP health check.

## Frontend Asset Linkage

- Production Dreamview keeps the original path resolution: `scripts/dreamview.sh` refreshes `modules/dreamview/frontend/dist` to Bazel's packaged frontend assets before launch, and `dreamview.launch` still starts with the global flagfile.
- `data/frontend` now makes `yarn build` resilient to a symlinked or non-writable `dist` directory by recreating a local writable `dist` before webpack runs.
- Test-mode Dreamview is isolated in `modules/dreamview/conf/dreamview_test.conf`, which sets `--static_file_dir=/apollo/data/frontend/dist` without touching the production launch path.
- To run the frontend test mode after `cd data/frontend && yarn build`, use `cd /apollo && DREAMVIEW_SCRIPT=./scripts/dreamview_test.sh ./scripts/bootstrap.sh restart`.
- When validating point cloud rendering changes, prefer this fast local-debug loop inside the container:
	1. `bash docker/scripts/whl.sh enter`
	2. `./apollo.sh build_opt_gpu`
	3. `cd data/frontend && yarn build`
	4. `cd /apollo && DREAMVIEW_SCRIPT=./scripts/dreamview_test.sh ./scripts/bootstrap.sh restart`
- If you need to inspect the active production asset directory, read `modules/dreamview/conf/dreamview.conf` and list `/apollo/modules/dreamview/frontend/dist` inside the container. For test mode, inspect `modules/dreamview/conf/dreamview_test.conf` and `/apollo/data/frontend/dist`.
