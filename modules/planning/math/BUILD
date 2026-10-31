load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "curve_math",
    srcs = ["curve_math.cc"],
    hdrs = ["curve_math.h"],
)

cc_library(
    name = "discrete_points_math",
    srcs = ["discrete_points_math.cc"],
    hdrs = ["discrete_points_math.h"],
    copts = PLANNING_COPTS,
    deps = [
        "@core//cyber",
    ],
)

cc_test(
    name = "curve_math_test",
    size = "small",
    srcs = ["curve_math_test.cc"],
    deps = [
        ":curve_math",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "polynomial_xd",
    srcs = ["polynomial_xd.cc"],
    hdrs = ["polynomial_xd.h"],
    copts = PLANNING_COPTS,
    deps = [
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
