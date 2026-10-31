load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "roi_service",
    srcs = ["roi_service.cc"],
    hdrs = ["roi_service.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "@wheelos_common//modules/common/util",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/lib/scene_manager:scene_service",
        "//modules/perception/lidar/lib/scene_manager/roi_service/proto:roi_service_cc_proto",
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
