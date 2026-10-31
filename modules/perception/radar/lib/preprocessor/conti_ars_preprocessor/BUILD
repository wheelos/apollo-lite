load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "conti_ars_preprocessor_test",
    size = "small",
    srcs = ["conti_ars_preprocessor_test.cc"],
    deps = [
        ":conti_ars_preprocessor",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "conti_ars_preprocessor",
    srcs = ["conti_ars_preprocessor.cc"],
    hdrs = ["conti_ars_preprocessor.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/perception/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/interface:base_preprocessor",
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
