load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cc_library(
    name = "lidar_segmentaion_common",
    srcs = [
        "bg_object_builder.cc",
        "object_split.cc",
    ],
    hdrs = [
        "bg_object_builder.h",
        "object_split.h",
    ],
    copts = PERCEPTION_COPTS,
    deps = [
        "//modules/perception/base",
        "//modules/perception/common/geometry:common",
        "//modules/perception/common/geometry:convex_hull_2d",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/object_builder",
        "@eigen",
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
