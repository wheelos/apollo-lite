load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_udas_ultrasonic_radar_protocol",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/drivers/radar/udas_ultrasonic_radar/proto:udas_ultrasonic_radar_cc_proto",
    ],
)

cpplint()
