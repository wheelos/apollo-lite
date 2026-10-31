load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointcloud_get_objects",
    srcs = [
        "pointcloud_get_objects.cc",
    ],
    hdrs = [
        "pointcloud_get_objects.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base:object",
        "//modules/perception/base:point",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/pipeline:data_frame",
        "//modules/perception/pipeline:plugin",
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
