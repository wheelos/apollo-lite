load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_yunle_protocol",
    srcs = [
        "bms_rt_soc_17904001.cc",
        "bms_soc_101.cc",
        "bms_vol_cur_100.cc",
        "ccu_status_51.cc",
        "sas_angle_feedback_e1.cc",
        "scu_1_121.cc",
        "scu_tq_123.cc",
        "target_speed_feedback_7f1.cc",
        "waring_level_77.cc",
        "wheel_speed_feedback_rpm_168.cc",
    ],
    hdrs = [
        "bms_rt_soc_17904001.h",
        "bms_soc_101.h",
        "bms_vol_cur_100.h",
        "ccu_status_51.h",
        "sas_angle_feedback_e1.h",
        "scu_1_121.h",
        "scu_tq_123.h",
        "target_speed_feedback_7f1.h",
        "waring_level_77.h",
        "wheel_speed_feedback_rpm_168.h",
    ],
    deps = [
        "//modules/canbus/vehicle:chassis_extension_tools",
        "//modules/canbus/vehicle/yunle/proto:yunle_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
    ],
)

cpplint()
