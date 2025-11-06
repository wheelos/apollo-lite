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
    name = "libyg_radar_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":yg_radar_component_lib",
    ],
)

cc_library(
    name = "yg_radar_component_lib",
    srcs = [
        "yg_radar_component.cc",
        "yg_radar_message_manager.cc",
    ],
    hdrs = [
        "yg_radar_component.h",
        "yg_radar_message_manager.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/common_msgs/sensor_msgs:radar_cc_proto",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/yg_radar/proto:yg_radar_cc_proto",
        "//modules/drivers/radar/yg_radar/proto:yg_radar_config_cc_proto",
        "//modules/drivers/radar/yg_radar/protocol:canbus_yg_radar_protocol",
    ],
    alwayslink = True,
)

cpplint()
