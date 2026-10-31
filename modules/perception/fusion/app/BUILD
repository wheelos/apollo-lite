load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obstacle_multi_sensor_fusion",
    srcs = ["obstacle_multi_sensor_fusion.cc"],
    hdrs = ["obstacle_multi_sensor_fusion.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base",
        "//modules/perception/fusion/base",
        "//modules/perception/fusion/lib/fusion_system/probabilistic_fusion",
        "//modules/perception/fusion/lib/interface:base_fusion_system",
        "//modules/perception/fusion/lib/interface:base_multisensor_fusion",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline",
        "@core//cyber",
        "@eigen",
    ],
    alwayslink = True,
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
