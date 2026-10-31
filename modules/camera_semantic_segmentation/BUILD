load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

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
    target_compatible_with = select({
        "//tools/platform:use_gpu": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
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
    deps = [
        ":image_frame",
        ":segformer",
        ":tensorrt_executor",
        "//modules/camera_semantic_segmentation/proto:camera_semantic_segmentation_cc_proto",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
    alwayslink = True,
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

pkg_files(
    name = "package_files_data",
    srcs = [":runtime_config"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "camera_semantic_segmentation",
    strip_prefix = strip_prefix.from_root("modules/camera_semantic_segmentation"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = [":libcamera_semantic_segmentation_component_plugin.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "camera_semantic_segmentation/lib",
    strip_prefix = strip_prefix.from_root("modules/camera_semantic_segmentation"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_data",
        ":package_files_libraries",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)
