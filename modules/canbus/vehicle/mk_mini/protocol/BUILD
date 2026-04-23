load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_mk_mini_protocol",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/canbus/vehicle:chassis_extension_tools",
        "//modules/canbus/vehicle/mk_mini/proto:mk_mini_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cpplint()
