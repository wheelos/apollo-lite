load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_binary(
    name = "libudas_ultrasonic_radar_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":udas_ultrasonic_radar_component_lib",
    ],
)

cc_library(
    name = "udas_ultrasonic_radar_component_lib",
    srcs = [
        "udas_ultrasonic_radar_component.cc",
        "udas_ultrasonic_radar_message_manager.cc",
    ],
    hdrs = [
        "udas_ultrasonic_radar_component.h",
        "udas_ultrasonic_radar_message_manager.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/common_msgs/sensor_msgs:ultrasonic_radar_cc_proto",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/udas_ultrasonic_radar/proto:udas_ultrasonic_radar_cc_proto",
        "//modules/drivers/radar/udas_ultrasonic_radar/proto:ultrasonic_radar_config_cc_proto",
        "//modules/drivers/radar/udas_ultrasonic_radar/protocol:canbus_udas_ultrasonic_radar_protocol",
    ],
    alwayslink = True,
)

cpplint()
