load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "yunle_vehicle_factory",
    alwayslink = True,
    srcs = [
        "yunle_vehicle_factory.cc",
    ],
    hdrs = [
        "yunle_vehicle_factory.h",
    ],
    deps = [
        ":yunle_controller",
        ":yunle_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/canbus/vehicle:vehicle_factory_core",
    ],
)

cc_library(
    name = "yunle_message_manager",
    srcs = [
        "yunle_message_manager.cc",
    ],
    hdrs = [
        "yunle_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/yunle/protocol:canbus_yunle_protocol",
    ],
)

cc_library(
    name = "yunle_controller",
    srcs = [
        "yunle_controller.cc",
    ],
    hdrs = [
        "yunle_controller.h",
    ],
    deps = [
        ":yunle_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/yunle/protocol:canbus_yunle_protocol",
    ],
)

cpplint()
