# Synthetic cargo calibration fixture

This bundle is test data only. It is not measured cargo calibration and must
never be selected for a real vehicle. Every translation is zero, every
quaternion is identity, and the camera intrinsics are placeholders.

The fixture contains the canonical sensor frame names for one IMU, one GNSS
antenna, one top LiDAR, and all eight camera positions. Each camera has a synthetic identity
transform from its `camera_*_link` frame to its matching
`camera_*_optical_frame`, plus placeholder intrinsics. The manifest's
`required_sensor_frame_id` list is intended to exercise naming and registry
validation, not to assert that this hardware is installed on a cargo vehicle.
The GNSS antenna translation is a synthetic static TF fixture, not a measured
lever arm.

To exercise the fixture explicitly in a test process, set
`WHEELOS_VEHICLE_PROFILE_KEY=vehicles/cargo/config/vehicle_profile.pb.txt`.
Do not export this profile from production startup scripts.
