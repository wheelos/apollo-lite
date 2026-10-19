load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "reachable_region",
    srcs = ["reachable_region.cc"],
    hdrs = ["reachable_region.h"],
    deps = [
        "//modules/open_space_planning/common:status",
        "//modules/open_space_planning/common:types",
    ],
)

cc_test(
    name = "reachable_region_test",
    size = "small",
    srcs = ["reachable_region_test.cc"],
    deps = [
        ":reachable_region",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
