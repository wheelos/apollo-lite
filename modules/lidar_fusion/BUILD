load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fusion",
    srcs = ["core/fusion.cc"],
    hdrs = ["core/fusion.h"],
    deps = [
        ":candidates",
        ":spatial",
        "//modules/lidar_fusion/proto:lidar_fusion_cc_proto",
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
        "//modules/lidar_semantic_segmentation/proto:lidar_semantic_segmentation_cc_proto",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "candidates",
    srcs = ["core/candidates.cc"],
    hdrs = ["core/candidates.h"],
    deps = [
        "//modules/lidar_fusion/proto:lidar_fusion_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "spatial",
    srcs = ["core/spatial.cc"],
    hdrs = ["core/spatial.h"],
    deps = [
        "//modules/lidar_fusion/proto:lidar_fusion_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "temporal",
    srcs = ["core/temporal.cc"],
    hdrs = ["core/temporal.h"],
    deps = [
        ":fusion",
        ":spatial",
        "@eigen",
        "@wheelos_common//tracking/core:tracking",
        "@wheelos_common//tracking/estimation:predictor",
    ],
)

cc_library(
    name = "frame_buffer",
    srcs = ["core/frame_buffer.cc"],
    hdrs = ["core/frame_buffer.h"],
    deps = [":fusion"],
)

cc_library(
    name = "lidar_fusion_component_lib",
    srcs = ["component/lidar_fusion_component.cc"],
    hdrs = ["component/lidar_fusion_component.h"],
    copts = ['-DMODULE_NAME=\\"lidar_fusion\\"'],
    deps = [
        ":frame_buffer",
        ":temporal",
        "@core//cyber",
    ],
    alwayslink = True,
)

cc_binary(
    name = "replay_cached",
    srcs = ["tools/replay_cached.cc"],
    deps = [
        ":fusion",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_binary(
    name = "replay_fusion",
    srcs = ["tools/replay_fusion.cc"],
    deps = [
        ":temporal",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_binary(
    name = "replay",
    srcs = ["tools/replay.cc"],
    target_compatible_with = select({
        "//tools/platform:use_gpu": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        ":fusion",
        ":temporal",
        "//modules/lidar_object_detection:pointpillars_input",
        "//modules/lidar_object_detection:pointpillars_runtime_api",
        "//modules/lidar_semantic_segmentation:rangeret",
        "//modules/lidar_semantic_segmentation:tensorrt_executor",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_binary(
    name = "liblidar_fusion_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":lidar_fusion_component_lib"],
)

cc_test(
    name = "fusion_test",
    size = "small",
    srcs = ["core/fusion_test.cc"],
    deps = [
        ":fusion",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "temporal_test",
    size = "small",
    srcs = ["core/temporal_test.cc"],
    deps = [
        ":temporal",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "candidates_test",
    size = "small",
    srcs = ["core/candidates_test.cc"],
    deps = [
        ":candidates",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "spatial_test",
    size = "small",
    srcs = ["core/spatial_test.cc"],
    deps = [
        ":spatial",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "frame_buffer_test",
    size = "small",
    srcs = ["core/frame_buffer_test.cc"],
    deps = [
        ":frame_buffer",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "component_test",
    size = "small",
    srcs = ["component/lidar_fusion_component_test.cc"],
    data = ["conf/lidar_fusion.pb.txt"],
    tags = ["manual"],
    deps = [
        ":lidar_fusion_component_lib",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
        "@core//cyber",
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
    data = [":runtime_config"],
    data_dest = "lidar_fusion",
    library_dest = "lidar_fusion/lib",
    targets = [":liblidar_fusion_component.so"],
)

cpplint()
