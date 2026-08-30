load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "save_lidar_semantic_pcd",
    srcs = ["save_lidar_semantic_pcd.py"],
    deps = [
        "@core//cyber/python/cyber_py3:record",
        "//modules/lidar_semantic_segmentation/proto:lidar_semantic_segmentation_py_pb2",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_py_pb2",
    ],
)
