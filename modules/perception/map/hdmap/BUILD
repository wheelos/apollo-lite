load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hdmap_input",
    srcs = ["hdmap_input.cc"],
    hdrs = ["hdmap_input.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base:base_type",
        "//modules/perception/base:blob",
        "//modules/perception/base:common",
        "//modules/perception/base:frame",
        "//modules/perception/base:image",
        "//modules/perception/base:impending_collision_edge",
        "//modules/perception/base:object",
        "//modules/perception/base:object_pool",
        "//modules/perception/base:object_pool_types",
        "//modules/perception/base:point_cloud",
        "//modules/perception/base:syncedmem",
        "//modules/perception/common/geometry:common",
        "//modules/perception/lib/config_manager",
        "@core//cyber",
        "@wheelos_common//modules/common/math",
        "@wheelos_common//modules/common/util:util_tool",
        "@wheelos_map//modules/map/hdmap",
        "@wheelos_map//modules/map/hdmap:hdmap_util",
    ],
)

#cc_test(
#    name = "hdmap_input_test",
#    size = "small",
#    srcs = ["hdmap_input_test.cc"],
#    deps = [
#        ":hdmap_input",
#        "@com_google_googletest//:gtest_main",
#        "//modules/perception/base:base_type",
#        "//modules/perception/base:point_cloud",
#        "//modules/perception/common/geometry:common",
#        "//modules/perception/lib/config_manager",
#    ],
#)

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
