load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "control_test_base",
    srcs = ["control_test_base.cc"],
    hdrs = ["control_test_base.h"],
    copts = ["-fno-access-control"],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/control:control_component_lib",
        "//modules/control/common:control_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "simple_control_test",
    size = "small",
    srcs = ["simple_control_test.cc"],
    copts = ["-fno-access-control"],
    data = ["//modules/control:test_data"],
    deps = [
        ":control_test_base",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "relative_position_test",
    size = "small",
    srcs = ["relative_position_test.cc"],
    copts = ["-fno-access-control"],
    data = ["//modules/control:test_data"],
    deps = [
        ":control_test_base",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "mpc_controller_test",
    size = "small",
    srcs = ["mpc_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = ["//modules/control:test_data"],
    deps = [
        ":control_test_base",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planning_fault_test",
    size = "small",
    srcs = ["planning_fault_test.cc"],
    data = ["//modules/control:test_data"],
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/status",
        "//modules/common/util:util_lib",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:pad_msg_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/control/common:control_gflags",
        "//modules/control/common:dependency_injector",
        "//modules/control/controller:controller_agent",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:local_view_cc_proto",
        "//modules/control/safety:safety_manager",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
