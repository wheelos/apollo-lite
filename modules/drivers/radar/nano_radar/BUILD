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

cc_library(
    name = "nano_radar_message_manager",
    srcs = ["nano_radar_message_manager.cc"],
    hdrs = ["nano_radar_message_manager.h"],
    deps = [
        "//modules/common/util:message_util",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/nano_radar/protocol:drivers_nano_radar_protocol",
    ],
)

cc_library(
    name = "nano_radar_lib",
    srcs = ["nano_radar_canbus_component.cc"],
    hdrs = ["nano_radar_canbus_component.h"],
    copts = ['-DMODULE_NAME=\\"nano_radar\\"'],
    deps = [
        ":nano_radar_message_manager",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/proto:sensor_canbus_conf_cc_proto",
        "//modules/drivers/radar/nano_radar/protocol:drivers_nano_radar_protocol",
        "@eigen",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libnano_radar.so",
    linkshared = True,
    linkstatic = True,
    deps = [":nano_radar_lib"],
)

cpplint()
