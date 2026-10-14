load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "model_input",
    srcs = ["types/model_input.cc"],
    hdrs = ["types/model_input.h"],
    deps = [
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "centerpoint_runner",
    srcs = ["inference/centerpoint_runner.cc"],
    hdrs = [
        "inference/centerpoint_backend.h",
        "inference/centerpoint_runner.h",
    ],
    deps = [
        ":model_input",
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "offline_result_writer",
    srcs = ["tools/offline_result_writer.cc"],
    hdrs = ["tools/offline_result_writer.h"],
    deps = [
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
    ],
)

cc_binary(
    name = "pcd_to_nvidia_bin",
    srcs = ["tools/pcd_to_nvidia_bin.cc"],
    deps = [
        ":model_input",
        "//modules/perception/lidar/common:pcl_util",
        "@com_google_gflags_gflags//:gflags",
        "@com_google_protobuf//:protobuf",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_test(
    name = "model_input_test",
    size = "small",
    srcs = ["types/model_input_test.cc"],
    deps = [
        ":model_input",
        "@com_google_googletest//:gtest_main",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

filegroup(
    name = "runtime_config",
    srcs = glob([
        "conf/**",
        "dag/**",
    ]),
)

cpplint()
