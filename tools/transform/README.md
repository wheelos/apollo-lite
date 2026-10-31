# Transform runtime

`wheelos_transform` owns the query libraries, Cyber transport, static publisher
and vendored tf2. The product owns assembly and deployment. Build with the
managed container and the opt-in development overrides:

```bash
python3 scripts/bzlmod.py dev build --config=cpu //tools/transform:runtime
```

The shared runtime layout collects mainboard, plugin runfiles and DAG/launch
files. It preserves `modules/transform` deployment paths while resolving the
component from the external module. `:runtime` does not archive or install;
`//tools/transform:install` explicitly archives the same layout.

The shared launcher is currently named `bin/mainboard-hardware`. From the
runtime directory, start the actual static publisher with:

```bash
bin/mainboard-hardware -d modules/transform/dag/static_transform.dag
```

Supply the selected vehicle calibration environment described in the module's
README. Missing or invalid calibration must fail startup. A load-only check
does not validate calibration or TF publication:

```bash
python3 tools/hardware/runtime_smoke.py <runtime-directory> \
  --plugin bazel-bin/modules/transform/libstatic_transform_component.so
```

Run that check against both the source runtime and an extracted `:install`
archive. Target-system libraries remain required. Source checkout validation
does not establish Registry or remote release acceptance.
