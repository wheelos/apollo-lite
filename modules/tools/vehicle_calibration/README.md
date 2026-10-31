# Vehicle calibration validation

This tool validates the selected vehicle bundle across Common's selection and
identity contract, Transform's static extrinsics, auxiliary LiDAR height, and
Perception metadata/camera models. It is intentionally outside the Transform
runtime module.

Set `WHEELOS_ASSET_ROOT`, `WHEELOS_VEHICLE_PROFILE_KEY` and the deployment's
`WHEELOS_CALIBRATION_MANIFEST_SHA256`, then run in the managed container:

```bash
bazel run --config=dev //modules/tools/vehicle_calibration:validate_calibration
```

Invalid selection or calibration exits nonzero. Success prints the selected
identity and inventory; it does not certify physical accuracy or consistency
across running processes.

The install target places the executable under `tools/vehicle_calibration`.
The old Transform validator target and old Transform-local configuration APIs
have been removed, without aliases.
