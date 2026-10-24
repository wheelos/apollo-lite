load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drivers_nano_radar_protocol",
    srcs = [
        "collision_detection_config_400.cc",
        "collision_detection_state_408.cc",
        "collision_detection_warning_60e.cc",
        "object_general_info_60b.cc",
        "object_list_status_60a.cc",
        "radar_config_200.cc",
        "radar_state_201.cc",
        "region_config_401.cc",
        "region_state_402.cc",
        "software_version_700.cc",
    ],
    hdrs = [
        "collision_detection_config_400.h",
        "collision_detection_state_408.h",
        "collision_detection_warning_60e.h",
        "const_vars.h",
        "object_general_info_60b.h",
        "object_list_status_60a.h",
        "radar_config_200.h",
        "radar_state_201.h",
        "region_config_401.h",
        "region_state_402.h",
        "software_version_700.h",
    ],
    deps = [
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/nano_radar/proto:nano_radar_conf_cc_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_cc_proto",
        "@wheelos_msgs//wheelos_msgs/drivers_msgs:can_card_parameter_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:nano_radar_cc_proto",
    ],
)

cpplint()
