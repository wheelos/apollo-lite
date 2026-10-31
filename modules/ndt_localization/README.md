# NDT-based Lidar Localization

This is a standalone localization module. Its map storage, common utilities,
and LiDAR-height parameters are maintained under `modules/ndt_localization`.

## Introduction
  NDT-based lidar localization provide a straightforward, flexible method which combile inspva information and lidar pointcloud to achieve high localization accuracy.

  Currently this localization method only works for x86_64 platform

## Input
  * Point cloud data from LiDAR sensor ( `/apollo/sensor/velodyne64/compensator/PointCloud2`)
  * Inspva message from integrated navigation sensor ( `/apollo/sensor/gnss/odometry`)
  * Localization map (`<selected-map-directory>/<ndt-map-directory>/<local-map-name>`)
  * Rigid LiDAR extrinsics from the TF tree published by `modules/transform`
  * LiDAR height parameters in `modules/ndt_localization/params/velodyne64_height.yaml`

## Output
  * Localization result is published as a `LocalizationEstimate` message on `/apollo/localization/pose`.

### NDT Localization Setting
under some circumstance, we need to balance the speed and accuracy of the algorithm. So we expose some parameters of NDT matching process, It includes `online_resolution` for online pointcloud, `ndt_max_iterations` for iterative optimization of NDT matching, `ndt_target_resolution` for target resolution, `ndt_line_search_step_size` for searching step size of iteration and `ndt_transformation_epsilon` for convergence condition.

## Generate NDT Localization Map
  NDT Localization map is used for NDT-based localization, which is a voxel-grid representation of the environment. Each cell stores the centroid and relative covariance of the points in the cell. The map is organized as a group of map nodes. For more information, please refer to `modules/ndt_localization/map/ndt_map`.

  Build and run `//modules/ndt_localization/map_creation:ndt_map_creator` to generate an NDT localization map. You need to provide a group of point cloud frames (as .pcd file), corresponding poses file, and UTM zone id. The format of the poses file is `pcd_number timestamp x y z qx qy qz qw`.

The component follows the repository runtime layout:

- `launch/` contains the module launcher.
- `dag/` contains the Cyber component graph.
- `conf/` contains runtime flags.
- `map_creation/` and `ndt_locator/` contain NDT-specific implementation.
