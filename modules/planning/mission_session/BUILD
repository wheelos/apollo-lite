load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "mission_session_manager",
    srcs = ["mission_session_manager.cc"],
    hdrs = ["mission_session_manager.h"],
    copts = PLANNING_COPTS,
    deps = [
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:mission_directive_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:motion_execution_cc_proto",
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
