load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "can_receiver",
    hdrs = ["can_receiver.h"],
    deps = [
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:error_code_cc_proto",
    ],
)

cc_library(
    name = "can_sender",
    hdrs = ["can_sender.h"],
    deps = [
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "@com_google_googletest//:gtest",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:error_code_cc_proto",
    ],
)

cc_library(
    name = "message_manager_base",
    hdrs = [
        "message_manager.h",
        "protocol_data.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:error_code_cc_proto",
    ],
)

cc_test(
    name = "can_sender_test",
    size = "small",
    srcs = ["can_sender_test.cc"],
    deps = [
        "//modules/drivers/canbus/can_client/fake:fake_can_client",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
    ],
)

cc_test(
    name = "can_receiver_test",
    size = "small",
    srcs = ["can_receiver_test.cc"],
    linkstatic = True,
    deps = [
        "//modules/drivers/canbus/can_client/fake:fake_can_client",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:drive_state_cc_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_cc_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:vehicle_signal_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
        "@wheelos_msgs//wheelos_msgs/config_msgs:vehicle_config_cc_proto",
    ],
)

cc_test(
    name = "protocol_data_test",
    size = "small",
    srcs = ["protocol_data_test.cc"],
    linkstatic = True,
    deps = [
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "@com_google_googletest//:gtest_main",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
    ],
)

cc_test(
    name = "message_manager_test",
    size = "small",
    srcs = ["message_manager_test.cc"],
    deps = [
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
    ],
)

cpplint()
