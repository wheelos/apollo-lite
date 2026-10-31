load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointcloud_detection_preprocessor",
    srcs = [
        "pointcloud_detection_preprocessor.cc",
    ],
    hdrs = [
        "pointcloud_detection_preprocessor.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/lidar/lib/pointcloud_detection_preprocessor/pointcloud_down_sample",
        "//modules/perception/pipeline:plugin_factory",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:pointcloud_detection_preprocessor_config_cc_proto",
    ],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_headers",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
