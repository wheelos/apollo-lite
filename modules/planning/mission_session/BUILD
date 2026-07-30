load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "mission_session_manager",
    srcs = ["mission_session_manager.cc"],
    hdrs = ["mission_session_manager.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:mission_directive_cc_proto",
    ],
)

cc_test(
    name = "mission_session_manager_test",
    size = "small",
    srcs = ["mission_session_manager_test.cc"],
    deps = [
        ":mission_session_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
