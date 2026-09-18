load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_src_files")

package(default_visibility = ["//visibility:public"])

GUARDIAN_COPTS = ['-DMODULE_NAME=\\"guardian\\"']

cc_library(
    name = "guardian_component_lib",
    srcs = ["guardian_component.cc"],
    hdrs = ["guardian_component.h"],
    copts = GUARDIAN_COPTS,
    deps = [
        "//modules/collision_guardian/proto:collision_guardian_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/guardian/proto:guardian_conf_cc_proto",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:control_cmd_cc_proto",
        "@wheelos_msgs//wheelos_msgs/guardian_msgs:guardian_cc_proto",
        "@wheelos_msgs//wheelos_msgs/monitor_msgs:system_status_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libguardian_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":guardian_component_lib",
    ],
)

install(
    name = "install",
    data = [
        ":guardian.BUILD",
        ":runtime_data",
    ],
    data_dest = "guardian",
    library_dest = "guardian/lib",
    targets = [
        ":libguardian_component.so",
    ],
    deps = [
        ":pb_hdrs",
    ],
)

install(
    name = "pb_hdrs",
    data = [
        "//modules/guardian/proto:guardian_conf_cc_proto",
    ],
    data_dest = "guardian/include",
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_guardian_hdrs",
        ":install_guardian_src",
    ],
)

install_src_files(
    name = "install_guardian_src",
    dest = "guardian/src",
    filter = "*",
    src_dir = ["."],
)

install_src_files(
    name = "install_guardian_hdrs",
    dest = "guardian/include",
    filter = "*.h",
    src_dir = ["."],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cpplint()
