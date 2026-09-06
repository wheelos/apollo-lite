load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

LOCALIZATION_HEALTH_COPTS = ["-DMODULE_NAME=\\\"localization_health\\\""]

cc_library(
    name = "reason_aggregator",
    srcs = ["reason_aggregator.cc"],
    hdrs = ["reason_aggregator.h"],
    copts = LOCALIZATION_HEALTH_COPTS,
    deps = [
        "//modules/localization_health/proto:localization_health_cc_proto",
    ],
)

cc_library(
    name = "independent_checker",
    srcs = ["independent_checker.cc"],
    hdrs = ["independent_checker.h"],
    copts = LOCALIZATION_HEALTH_COPTS,
    deps = [
        ":reason_aggregator",
        "//modules/localization_health/proto:localization_health_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
    ],
)

cc_library(
    name = "capability_deriver",
    srcs = ["capability_deriver.cc"],
    hdrs = ["capability_deriver.h"],
    copts = LOCALIZATION_HEALTH_COPTS,
    deps = [
        ":reason_aggregator",
        "//modules/localization_health/proto:localization_health_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
    ],
)

cc_library(
    name = "health_state_machine",
    srcs = ["health_state_machine.cc"],
    hdrs = ["health_state_machine.h"],
    copts = LOCALIZATION_HEALTH_COPTS,
    deps = [
        ":reason_aggregator",
        "//modules/localization_health/proto:localization_health_cc_proto",
    ],
)

cc_library(
    name = "localization_health",
    srcs = ["localization_health.cc"],
    hdrs = ["localization_health.h"],
    copts = LOCALIZATION_HEALTH_COPTS,
    deps = [
        ":capability_deriver",
        ":health_state_machine",
        ":independent_checker",
        ":reason_aggregator",
        "//modules/localization_health/proto:localization_health_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
    ],
)

cc_library(
    name = "localization_health_component_lib",
    srcs = ["localization_health_component.cc"],
    hdrs = ["localization_health_component.h"],
    copts = LOCALIZATION_HEALTH_COPTS,
    alwayslink = True,
    deps = [
        ":localization_health",
        "@core//cyber",
    ],
)

cc_binary(
    name = "liblocalization_health_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":localization_health_component_lib"],
)

cc_test(
    name = "localization_health_test",
    size = "small",
    srcs = ["localization_health_test.cc"],
    deps = [
        ":localization_health",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.pb.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    library_dest = "localization_health/lib",
    data_dest = "localization_health",
    data = [
        ":runtime_data",
        ":localization_health.BUILD",
    ],
    targets = [
        ":liblocalization_health_component.so",
    ],
    deps = [
        ":pb_hdrs",
        ":pb_localization_health",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "localization_health/include",
    data = [
        "//modules/localization_health/proto:localization_health_cc_proto",
    ],
)

install_files(
    name = "pb_localization_health",
    dest = "localization_health",
    files = [
        "//modules/localization_health/proto:localization_health_py_pb2",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_localization_health_src",
        ":install_localization_health_hdrs",
    ],
)

install_src_files(
    name = "install_localization_health_src",
    src_dir = ["."],
    dest = "localization_health/src",
    filter = "*",
)

install_src_files(
    name = "install_localization_health_hdrs",
    src_dir = ["."],
    dest = "localization_health/include",
    filter = "*.h",
)

cpplint()
