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
        "@wheelos_common//modules/common/util",
        "@wheelos_common//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "@boost.filesystem",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
)

cc_test(
    name = "util_test",
    size = "small",
    srcs = ["util_test.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":util_lib",
        "//modules/planning/common:gtest_exit_main",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
)

cc_library(
    name = "common_lib",
    srcs = ["common.cc"],
    hdrs = ["common.h"],
    copts = PLANNING_COPTS,
    deps = [
        "@wheelos_common//modules/common/util",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "@core//cyber",
    ],
)

cc_library(
    name = "math_util_lib",
    srcs = ["math_util.cc"],
    hdrs = ["math_util.h"],
    copts = PLANNING_COPTS,
    deps = [
        "@wheelos_common//modules/common/math",
        "@core//cyber",
    ],
)

cc_library(
    name = "print_debug_info",
    hdrs = ["print_debug_info.h"],
    copts = PLANNING_COPTS,
    deps = [
        "@core//cyber",
    ],
)

cpplint()
