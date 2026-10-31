load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "get_image_data",
    srcs = ["get_image_data.cc"],
    hdrs = ["get_image_data.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/pipeline:data_frame",
        "//modules/perception/pipeline:plugin",
        "//modules/perception/pipeline/proto/plugin:get_image_data_config_cc_proto",
        "@opencv//:imgcodecs",
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
