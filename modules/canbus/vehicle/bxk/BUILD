load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bxk_vehicle_factory",
    srcs = [
        "bxk_vehicle_factory.cc",
    ],
    hdrs = [
        "bxk_vehicle_factory.h",
    ],
    deps = [
        ":bxk_controller",
        ":bxk_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "bxk_message_manager",
    srcs = [
        "bxk_message_manager.cc",
    ],
    hdrs = [
        "bxk_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/bxk/protocol:canbus_bxk_protocol",
    ],
)

cc_library(
    name = "bxk_controller",
    srcs = [
        "bxk_controller.cc",
    ],
    hdrs = [
        "bxk_controller.h",
    ],
    deps = [
        ":bxk_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/bxk/protocol:canbus_bxk_protocol",
    ],
)

cpplint()
