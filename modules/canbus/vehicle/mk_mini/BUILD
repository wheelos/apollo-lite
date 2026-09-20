load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mk_mini_vehicle_factory",
    srcs = [
        "mk_mini_vehicle_factory.cc",
    ],
    hdrs = [
        "mk_mini_vehicle_factory.h",
    ],
    deps = [
        ":mk_mini_controller",
        ":mk_mini_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/canbus/vehicle:vehicle_factory_core",
    ],
    alwayslink = True,
)

cc_library(
    name = "mk_mini_message_manager",
    srcs = [
        "mk_mini_message_manager.cc",
    ],
    hdrs = [
        "mk_mini_message_manager.h",
    ],
    deps = [
        "//modules/canbus/vehicle/mk_mini/protocol:canbus_mk_mini_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
    ],
)

cc_library(
    name = "mk_mini_controller",
    srcs = [
        "mk_mini_controller.cc",
    ],
    hdrs = [
        "mk_mini_controller.h",
    ],
    deps = [
        ":mk_mini_message_manager",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/mk_mini/protocol:canbus_mk_mini_protocol",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
    ],
)

cpplint()
