load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drivers_nano_radar_protocol",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_cc_proto",
        "@wheelos_msgs//wheelos_msgs/drivers_msgs:can_card_parameter_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:nano_radar_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/nano_radar/proto:nano_radar_conf_cc_proto",
    ],
)

cpplint()
