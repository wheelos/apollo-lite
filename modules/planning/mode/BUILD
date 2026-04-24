load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mode_resolution",
    srcs = ["mode_resolution.cc"],
    hdrs = ["mode_resolution.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/planning/environment:capability_extractor",
    ],
)

cc_test(
    name = "mode_resolution_test",
    size = "small",
    srcs = ["mode_resolution_test.cc"],
    deps = [
        ":mode_resolution",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
