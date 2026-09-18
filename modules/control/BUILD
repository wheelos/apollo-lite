load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

cc_library(
    name = "control_lib",
    copts = CONTROL_COPTS,
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/util",
        "//modules/control/common",
        "//modules/control/controller",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:preprocessor_cc_proto",
        "//modules/control/safety:safety_manager",
        "//modules/control/submodules:lat_lon_controller_submodule_lib",
        "//modules/control/submodules:mpc_controller_submodule_lib",
        "//modules/control/submodules:postprocessor_submodule_lib",
        "//modules/control/submodules:preprocessor_submodule_lib",
        "@com_github_gflags_gflags//:gflags",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:control_cmd_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:pad_msg_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
    ],
)

cc_library(
    name = "control_component_lib",
    srcs = ["control_component.cc"],
    hdrs = ["control_component.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":control_lib",
        "//modules/control/common:dependency_injector",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libcontrol_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":control_component_lib"],
)

install(
    name = "testdata",
    data = [
        ":test_data",
    ],
    data_dest = "control/addition_data",
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    data_dest = "control",
    library_dest = "control/lib",
    targets = [
        ":libcontrol_component.so",
    ],
    deps = [
        ":pb_hdrs",
        ":testdata",
        "//modules/control/common:install",
        "//modules/control/submodules:install",
        "//modules/control/tools:install",
    ],
)

install(
    name = "pb_hdrs",
    data = [
        "//modules/control/proto:calibration_table_cc_proto",
        "//modules/control/proto:control_common_conf_cc_proto",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:gain_scheduler_conf_cc_proto",
        "//modules/control/proto:lat_controller_conf_cc_proto",
        "//modules/control/proto:leadlag_conf_cc_proto",
        "//modules/control/proto:local_view_cc_proto",
        "//modules/control/proto:lon_controller_conf_cc_proto",
        "//modules/control/proto:mpc_controller_conf_cc_proto",
        "//modules/control/proto:mrac_conf_cc_proto",
        "//modules/control/proto:pid_conf_cc_proto",
        "//modules/control/proto:preprocessor_cc_proto",
    ],
    data_dest = "control/include",
)

install_files(
    name = "pb_control",
    dest = "control",
    files = [
        "@wheelos_msgs//wheelos_msgs/control_msgs:pad_msg_py_pb2",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_control_hdrs",
        ":install_control_src",
    ],
)

install_src_files(
    name = "install_control_src",
    dest = "control/src",
    filter = "*",
    src_dir = ["."],
)

install_src_files(
    name = "install_control_hdrs",
    dest = "control/include",
    filter = "*.h",
    src_dir = ["."],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_test(
    name = "control_component_test",
    size = "small",
    srcs = ["control_component_test.cc"],
    data = ["//modules/control:test_data"],
    linkstatic = True,
    deps = [
        ":control_component_lib",
        "//modules/common/util:util_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

# TODO(storypku): split test_data
filegroup(
    name = "test_data",
    srcs = glob([
        "testdata/**",
    ]),
)

cpplint()
