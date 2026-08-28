load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "semantic_types",
    hdrs = ["types/semantic_types.h"],
)

cc_library(
    name = "range_projection",
    srcs = ["types/range_projection.cc"],
    hdrs = ["types/range_projection.h"],
    deps = [
        ":semantic_types",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "rangeret",
    srcs = ["inference/rangeret.cc"],
    hdrs = ["inference/rangeret.h"],
    deps = [
        ":range_projection",
        ":semantic_types",
        "//modules/lidar_semantic_segmentation/proto:lidar_semantic_segmentation_cc_proto",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "tensorrt_executor",
    srcs = ["inference/tensorrt_executor.cc"],
    hdrs = ["inference/tensorrt_executor.h"],
    deps = [
        ":rangeret",
        "@core//cyber",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cc_library(
    name = "lidar_semantic_segmentation_component",
    srcs = ["component/lidar_semantic_segmentation_component.cc"],
    hdrs = ["component/lidar_semantic_segmentation_component.h"],
    alwayslink = True,
    deps = [
        ":rangeret",
        ":tensorrt_executor",
        "//modules/lidar_semantic_segmentation/proto:lidar_semantic_segmentation_cc_proto",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_binary(
    name = "lidar_semantic_segmentation_component_plugin",
    linkshared = True,
    linkstatic = False,
    deps = [":lidar_semantic_segmentation_component"],
)

cc_test(
    name = "range_projection_test",
    size = "small",
    srcs = ["types/range_projection_test.cc"],
    deps = [
        ":range_projection",
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

install(
    name = "install",
    data_dest = "lidar_semantic_segmentation",
    targets = [":lidar_semantic_segmentation_component_plugin"],
    data = [":runtime_config"],
)

cpplint()
