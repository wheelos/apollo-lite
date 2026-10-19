load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "parking_slot",
    srcs = ["parking_slot.cc"],
    hdrs = ["parking_slot.h"],
    deps = [
        "//modules/open_space_planning/common:status",
        "@wheelos_common//modules/common/math:vec2d",
    ],
)

cc_test(
    name = "parking_slot_test",
    size = "small",
    srcs = ["parking_slot_test.cc"],
    deps = [
        ":parking_slot",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
