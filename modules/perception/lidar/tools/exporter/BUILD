load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

cc_library(
    name = "msgexporterlib",
    srcs = ["msg_exporter.cc"],
    hdrs = ["msg_exporter.h"],
    deps = [
        "//modules/perception/lidar/common:pcl_util",
        "//modules/perception/onboard/transform_wrapper",
        "//modules/transform:transform_query",
        "@com_google_absl//absl/strings",
        "@core//cyber",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

cc_binary(
    name = "msg_exporter",
    srcs = ["msg_exporter_main.cc"],
    deps = [
        ":msgexporterlib",
        "@com_google_absl//absl/strings",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

cc_binary(
    name = "pcd_exporter",
    srcs = ["pcd_exporter_main.cc"],
    deps = [
        "//modules/perception/lidar/common:pcl_util",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cpplint()
