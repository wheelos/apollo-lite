load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "socket_can_client_raw",
    srcs = ["socket_can_client_raw.cc"],
    hdrs = ["socket_can_client_raw.h"],
    deps = [
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/drivers/canbus/can_client",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "socket_can_client_raw_test",
    size = "small",
    srcs = ["socket_can_client_raw_test.cc"],
    deps = [
        "//cyber",
        "//modules/drivers/canbus/can_client/socket:socket_can_client_raw",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
