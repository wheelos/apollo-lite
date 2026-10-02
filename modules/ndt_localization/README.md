# NDT-based Lidar Localization

This is a standalone localization module. It is independent of the MSF
runtime component and reuses only shared localization utilities and the MSF
NDT map storage implementation.

## Introduction
  NDT-based lidar localization provide a straightforward, flexible method which combile inspva information and lidar pointcloud to achieve high localization accuracy.

  Currently this localization method only works for x86_64 platform

## Input
  * Point cloud data from LiDAR sensor ( `/apollo/sensor/velodyne64/compensator/PointCloud2`)
  * Inspva message from integrated navigation sensor ( `/apollo/sensor/gnss/odometry`)
  * Localization map (FLAGS_map_dir + "/" + FLAGS_ndt_map_dir + "/" + FLAGS_local_map_name)
  * Rigid LiDAR extrinsics from the TF tree published by `modules/transform`
  * LiDAR height configuration (`conf/velodyne64_height.yaml`)

## Output
  * Localization result defined by Protobuf message `LocalizationEstimate`, which can be found in file `localization/proto/localization.proto`. ( `/apollo/localization/pose`)

### NDT Localization Setting
under some circumstance, we need to balance the speed and accuracy of the algorithm. So we expose some parameters of NDT matching process, It includes `online_resolution` for online pointcloud, `ndt_max_iterations` for iterative optimization of NDT matching, `ndt_target_resolution` for target resolution, `ndt_line_search_step_size` for searching step size of iteration and `ndt_transformation_epsilon` for convergence condition.

## Generate NDT Localization Map
  NDT localization maps use a voxel-grid representation of the environment. Each cell stores the centroid and relative covariance of the points in the cell. Reusable map structures are maintained under `map_support/ndt_map`; they are independent of the retired MSF estimator.

  Build and run `//modules/ndt_localization/map_creation:ndt_map_creator` to generate an NDT localization map. You need to provide a group of point cloud frames (as .pcd file), corresponding poses file, and UTM zone id. The format of the poses file is `pcd_number timestamp x y z qx qy qz qw`.

## Visualization Tool
  NDT localization also use the same online visualization tool as msf, which can show localization map, point cloud, horizontal position of localization results. This tool is simply launched by `./scripts/localization_online_visualizer.sh`.
The component follows the repository runtime layout:

- `launch/` contains the module launcher.
- `dag/` contains the Cyber component graph.
- `conf/` contains runtime flags.
- `map_creation/` and `ndt_locator/` contain NDT-specific implementation.
