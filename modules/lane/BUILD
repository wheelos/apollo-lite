load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "image_frame",
    srcs = ["types/image_frame.cc"],
    hdrs = ["types/image_frame.h"],
)

cc_library(
    name = "lane_types",
    hdrs = ["types/lane_types.h"],
)

cc_library(
    name = "ufldv2",
    srcs = ["inference/ufldv2.cc"],
    hdrs = ["inference/ufldv2.h"],
    deps = [
        ":image_frame",
        ":lane_types",
        "@opencv//:core",
        "@opencv//:imgproc",
    ],
)

cc_library(
    name = "tensorrt_executor",
    srcs = ["inference/tensorrt_executor.cc"],
    hdrs = ["inference/tensorrt_executor.h"],
    deps = [
        ":ufldv2",
        "@core//cyber",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cc_library(
    name = "static_lane_calibrator",
    srcs = ["calibration/static_lane_calibrator.cc"],
    hdrs = ["calibration/static_lane_calibrator.h"],
    deps = [":lane_types"],
)

cc_library(
    name = "lane_transform_adapter",
    srcs = ["component/lane_transform_adapter.cc"],
    hdrs = ["component/lane_transform_adapter.h"],
    deps = [
        "//modules/transform:transform_query",
        "@core//cyber/time:time",
        "@eigen",
    ],
)

cc_library(
    name = "lane_component",
    srcs = ["component/lane_detection_component.cc"],
    hdrs = ["component/lane_detection_component.h"],
    alwayslink = True,
    deps = [
        ":image_frame",
        ":lane_transform_adapter",
        ":static_lane_calibrator",
        ":tensorrt_executor",
        ":ufldv2",
        "//modules/lane/proto:lane_component_config_cc_proto",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_cc_proto",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:perception_lane_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

cc_binary(
    name = "lane_component_plugin",
    linkshared = True,
    linkstatic = False,
    deps = [":lane_component"],
)

cc_test(
    name = "image_frame_test",
    size = "small",
    srcs = ["types/image_frame_test.cc"],
    deps = [
        ":image_frame",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ufldv2_test",
    size = "small",
    srcs = ["inference/ufldv2_test.cc"],
    deps = [
        ":image_frame",
        ":lane_types",
        ":ufldv2",
        "@com_google_googletest//:gtest_main",
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
    data_dest = "lane",
    targets = [":lane_component_plugin"],
    data = [":runtime_config"],
)

cpplint()
