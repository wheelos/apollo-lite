load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "agilex_hunter_vehicle_factory",
    srcs = [
        "agilex_hunter_vehicle_factory.cc",
    ],
    hdrs = [
        "agilex_hunter_vehicle_factory.h",
    ],
    deps = [
        ":agilex_hunter_controller",
        ":agilex_hunter_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "agilex_hunter_message_manager",
    srcs = [
        "agilex_hunter_message_manager.cc",
    ],
    hdrs = [
        "agilex_hunter_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/agilex_hunter/protocol:canbus_agilex_hunter_protocol",
    ],
)

cc_library(
    name = "agilex_hunter_controller",
    srcs = [
        "agilex_hunter_controller.cc",
    ],
    hdrs = [
        "agilex_hunter_controller.h",
    ],
    deps = [
        ":agilex_hunter_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/agilex_hunter/protocol:canbus_agilex_hunter_protocol",
    ],
)

cpplint()
