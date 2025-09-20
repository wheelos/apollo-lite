load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "apple_vehicle_factory",
    srcs = [
        "apple_vehicle_factory.cc",
    ],
    hdrs = [
        "apple_vehicle_factory.h",
    ],
    deps = [
        ":apple_controller",
        ":apple_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "apple_message_manager",
    srcs = [
        "apple_message_manager.cc",
    ],
    hdrs = [
        "apple_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/apple/protocol:canbus_apple_protocol",
    ],
)

cc_library(
    name = "apple_controller",
    srcs = [
        "apple_controller.cc",
    ],
    hdrs = [
        "apple_controller.h",
    ],
    deps = [
        ":apple_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/apple/protocol:canbus_apple_protocol",
    ],
)

cpplint()
