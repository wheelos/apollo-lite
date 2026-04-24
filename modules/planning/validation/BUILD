load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "validation_supervisor",
    srcs = ["validation_supervisor.cc"],
    hdrs = ["validation_supervisor.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/common:local_view",
        "//modules/planning/environment:capability_extractor",
        "//modules/planning:planning_runtime_context",
    ],
)

cc_test(
    name = "validation_supervisor_test",
    size = "small",
    srcs = ["validation_supervisor_test.cc"],
    deps = [
        ":validation_supervisor",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
