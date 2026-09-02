load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "semantic_types",
    hdrs = ["types/semantic_types.h"],
)

cc_library(
    name = "image_frame",
    srcs = ["types/image_frame.cc"],
    hdrs = ["types/image_frame.h"],
    deps = [":semantic_types"],
)

cc_library(
    name = "segformer",
    srcs = ["inference/segformer.cc"],
    hdrs = ["inference/segformer.h"],
    deps = [
        ":image_frame",
        ":semantic_types",
        "//modules/camera_semantic_segmentation/proto:camera_semantic_segmentation_cc_proto",
        "@opencv//:core",
        "@opencv//:imgproc",
    ],
)

cc_library(
    name = "tensorrt_executor",
    srcs = ["inference/tensorrt_executor.cc"],
    hdrs = ["inference/tensorrt_executor.h"],
    deps = [
        ":segformer",
        "@core//cyber",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cc_library(
    name = "camera_semantic_segmentation_component",
    srcs = ["component/camera_semantic_segmentation_component.cc"],
    hdrs = ["component/camera_semantic_segmentation_component.h"],
    alwayslink = True,
    deps = [
        ":image_frame",
        ":segformer",
        ":tensorrt_executor",
        "//modules/camera_semantic_segmentation/proto:camera_semantic_segmentation_cc_proto",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

cc_binary(
    name = "libcamera_semantic_segmentation_component_plugin.so",
    linkshared = True,
    linkstatic = False,
    deps = [":camera_semantic_segmentation_component"],
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
    name = "segformer_test",
    size = "small",
    srcs = ["inference/segformer_test.cc"],
    deps = [
        ":image_frame",
        ":segformer",
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
    data_dest = "camera_semantic_segmentation",
    library_dest = "camera_semantic_segmentation/lib",
    targets = [":libcamera_semantic_segmentation_component_plugin.so"],
    data = [":runtime_config"],
)

cpplint()
