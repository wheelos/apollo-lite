load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "util_lib",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":print_debug_info",
        "//cyber",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
        "@boost.filesystem",
    ],
)

cc_test(
    name = "util_test",
    size = "small",
    srcs = ["util_test.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":util_lib",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
        "//modules/planning/testing:gtest_exit_main",
    ],
)

cc_library(
    name = "common_lib",
    srcs = ["common.cc"],
    hdrs = ["common.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
    ],
)

cc_library(
    name = "math_util_lib",
    srcs = ["math_util.cc"],
    hdrs = ["math_util.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common/math",
    ],
)

cc_library(
    name = "print_debug_info",
    hdrs = ["print_debug_info.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
    ],
)

cpplint()
