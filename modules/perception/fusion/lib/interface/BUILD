load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interface",
    hdrs = [
        "base_data_association.h",
        "base_existence_fusion.h",
        "base_gatekeeper.h",
        "base_motion_fusion.h",
        "base_shape_fusion.h",
        "base_tracker.h",
        "base_type_fusion.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/fusion/base:base_forward_declaration",
        "//modules/perception/fusion/base:sensor",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:plugin",
        "@core//cyber",
    ],
)

cc_library(
    name = "base_multisensor_fusion",
    hdrs = ["base_multisensor_fusion.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/fusion/base",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline",
        "@core//cyber",
    ],
)

cc_library(
    name = "base_fusion_system",
    hdrs = ["base_fusion_system.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base:frame",
        "//modules/perception/fusion/base",
        "//modules/perception/lib/registerer",
        "@core//cyber",
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
