load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_yg_radar_protocol",
    srcs = [
        "radar_object_info_540.cc",
        "radar_object_list_560.cc",
    ],
    hdrs = [
        "radar_object_info_540.h",
        "radar_object_list_560.h",
    ],
    deps = [
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/drivers/radar/yg_radar/proto:yg_radar_cc_proto",
    ],
)

cpplint()
