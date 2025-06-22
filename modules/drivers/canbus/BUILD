load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "drivers/lib",
    targets = [
        ":sensor_gflags",
        ":sensor_canbus_lib"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "sensor_gflags",
    srcs = ["sensor_gflags.cc"],
    hdrs = ["sensor_gflags.h"],
    alwayslink = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "sensor_canbus_lib",
    hdrs = ["sensor_canbus.h"],
    alwayslink = True,
    deps = [
        ":sensor_gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
    ],
)

cpplint()
