load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "conti_ars_detector_test",
    size = "small",
    srcs = ["conti_ars_detector_test.cc"],
    deps = [
        ":conti_ars_detector",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "conti_ars_detector",
    srcs = ["conti_ars_detector.cc"],
    hdrs = ["conti_ars_detector.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base",
        "//modules/perception/common/geometry:roi_filter",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/radar/common:radar_util",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/interface:base_detector",
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
