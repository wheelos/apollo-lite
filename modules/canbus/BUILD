load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "canbus_component_lib",
    srcs = ["canbus_component.cc"],
    hdrs = ["canbus_component.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/vehicle:vehicle_factory",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
        "@com_github_gflags_gflags//:gflags",
        "@core//cyber",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/monitor_log",
        "@wheelos_common//modules/common/status",
        "@wheelos_msgs//wheelos_msgs/guardian_msgs:guardian_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libcanbus_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":canbus_component_lib",
    ],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libcanbus_component.so",
    ],
    deps = [
        ":pb_canbus",
    ],
)

install_files(
    name = "pb_canbus",
    dest = "modules/canbus",
    files = [
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_py_pb2",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

filegroup(
    name = "test_data",
    srcs = glob(["testdata/**"]),
)

cpplint()
