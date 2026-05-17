load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "trigger_types",
    hdrs = ["trigger_types.h"],
)

cc_library(
    name = "trigger_arbiter",
    srcs = ["trigger_arbiter.cc"],
    hdrs = ["trigger_arbiter.h"],
    deps = [
        ":trigger_types",
    ],
)

cc_library(
    name = "roadlog_layout",
    srcs = ["roadlog_layout.cc"],
    hdrs = ["roadlog_layout.h"],
    deps = [
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "trigger_arbiter_test",
    size = "small",
    srcs = ["trigger_arbiter_test.cc"],
    deps = [
        ":trigger_arbiter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "roadlog_layout_test",
    size = "small",
    srcs = ["roadlog_layout_test.cc"],
    deps = [
        ":roadlog_layout",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
