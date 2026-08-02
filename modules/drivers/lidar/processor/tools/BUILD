load("@rules_cc//cc:defs.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "lidar_fusion_baseline",
    srcs = ["lidar_fusion_baseline.cc"],
    deps = [
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/processor:lidar_policy_lib",
        "//modules/drivers/lidar/processor:lidar_unified_component_lib",
        "//modules/drivers/lidar/proto:lidar_unified_component_config_cc_proto",
        "//modules/transform:buffer_interface",
    ],
)
