# Low-speed product configuration

This directory owns low-speed product-wide runtime defaults, separate from
component packages and shared Common libraries. `global_flagfile.txt` selects
the default vehicle template in `vehicles/default/vehicle_param.pb.txt`. The
template retains
the previous vehicle parameters; it is not acceptance of a particular vehicle.

Run source commands from the workspace root. Installed runtimes preserve the
`conf/vehicles/default/` path, so the global flagfile uses a relative vehicle
configuration path in both layouts. The source global flagfile's map/log paths
remain development defaults, not portable deployment selections.

Configuration responsibilities:

| Owner | Responsibility |
| --- | --- |
| `products/<product>/conf/` | Product-wide runtime defaults and vehicle templates |
| `pkgs/*/modules/*/conf/` | Component-owned schemas, parameters and example configurations |
| `tools/products/`, `tools/hardware/`, `tools/modules/` | Build-time product and capability selection |
| Deployment input | Reviewed vehicle calibration, map/model assets, topics, launch graph and overrides |

Build-time selection does not activate a runtime configuration. Deployment
overrides must be explicit; do not silently select a vehicle or map on failure.
Do not put model binaries or site map bundles in shared Common libraries.

`//products/low_speed/conf:package_files` installs these defaults under `conf/`.
`@wheelos_common//tools:package_files` packages shared Common code separately.
Existing `--flagfile=modules/global_config/global_flagfile.txt` invocations must
use `--flagfile=conf/global_flagfile.txt`.
