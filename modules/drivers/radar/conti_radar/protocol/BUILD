load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drivers_conti_radar_protocol",
    srcs = [
        "cluster_general_info_701.cc",
        "cluster_list_status_600.cc",
        "cluster_quality_info_702.cc",
        "motion_input_speed_300.cc",
        "motion_input_yawrate_301.cc",
        "object_extended_info_60d.cc",
        "object_general_info_60b.cc",
        "object_list_status_60a.cc",
        "object_quality_info_60c.cc",
        "radar_config_200.cc",
        "radar_state_201.cc",
    ],
    hdrs = [
        "cluster_general_info_701.h",
        "cluster_list_status_600.h",
        "cluster_quality_info_702.h",
        "const_vars.h",
        "motion_input_speed_300.h",
        "motion_input_yawrate_301.h",
        "object_extended_info_60d.h",
        "object_general_info_60b.h",
        "object_list_status_60a.h",
        "object_quality_info_60c.h",
        "radar_config_200.h",
        "radar_state_201.h",
    ],
    deps = [
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/conti_radar/proto:conti_radar_conf_cc_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_cc_proto",
        "@wheelos_msgs//wheelos_msgs/drivers_msgs:can_card_parameter_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:conti_radar_cc_proto",
    ],
)

cpplint()
