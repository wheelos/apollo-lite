load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

cc_library(
    name = "control_lib",
    copts = CONTROL_COPTS,
    deps = [
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/latency_recorder",
        "@wheelos_common//modules/common/monitor_log",
        "@wheelos_common//modules/common/util",
        "//modules/control/common",
        "//modules/control/controller",
        "@wheelos_msgs//wheelos_msgs/control_msgs:control_cmd_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:control_runtime_status_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:safety_stop_cc_proto",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:preprocessor_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:motion_execution_cc_proto",
        "//modules/control/safety:safety_manager",
        "@com_github_gflags_gflags//:gflags",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:pad_msg_cc_proto",
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
        "//modules/control/common:executor_arbiter",
        "//modules/control/common:motion_command_adapter",
        "//modules/control/common:motion_execution_manager",
        "//modules/control/common:motion_execution_monitor",
        "//modules/control/common:motion_primitive_executor",
        "//modules/control/controller:controller_profile_manager",
        "@wheelos_common//modules/common/configs:vehicle_config_helper",
        "@wheelos_common//execution_state:client",
        "@wheelos_common//execution_state:flags",
        "@wheelos_msgs//wheelos_msgs/control_msgs:safety_stop_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libcontrol_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":control_component_lib"],
)

pkg_files(
    name = "testdata_data",
    srcs = [":test_data"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "control/addition_data",
    strip_prefix = strip_prefix.from_root("modules/control"),
)

pkg_filegroup(
    name = "testdata",
    srcs = [":testdata_data"],
)

pkg_files(
    name = "package_files_data",
    srcs = [":runtime_data"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "control",
    strip_prefix = strip_prefix.from_root("modules/control"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = [":libcontrol_component.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "control/lib",
    strip_prefix = strip_prefix.from_root("modules/control"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_data",
        ":package_files_libraries",
        ":pb_hdrs",
        ":testdata",
        "//modules/control/common:package_files",
        "//modules/control/tools:package_files",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

pkg_files(
    name = "pb_hdrs_data",
    srcs = [
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
    attributes = pkg_attributes(mode = "0644"),
    prefix = "control/include",
    strip_prefix = strip_prefix.from_root("modules/control"),
)

pkg_filegroup(
    name = "pb_hdrs",
    srcs = [":pb_hdrs_data"],
)

pkg_files(
    name = "pb_control_data",
    srcs = ["@wheelos_msgs//wheelos_msgs/control_msgs:pad_msg_py_pb2"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "control",
    strip_prefix = strip_prefix.from_pkg(),
)

pkg_filegroup(
    name = "pb_control",
    srcs = [":pb_control_data"],
)

pkg_filegroup(
    name = "source_files",
    srcs = [
        ":package_control_hdrs",
        ":package_control_src",
    ],
)

pkg_install(
    name = "source_dir",
    srcs = [":source_files"],
    tags = ["manual"],
)

pkg_files(
    name = "package_control_src_data",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ) + [
        "//modules/control/common:exported_source_files",
        "//modules/control/controller:exported_source_files",
        "//modules/control/proto:exported_source_files",
        "//modules/control/safety:exported_source_files",
        "//modules/control/tools:exported_source_files",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "control/src",
    strip_prefix = strip_prefix.from_root("modules/control"),
)

pkg_filegroup(
    name = "package_control_src",
    srcs = [":package_control_src_data"],
)

pkg_files(
    name = "package_control_hdrs_data",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ) + [
        "//modules/control/common:exported_source_headers",
        "//modules/control/controller:exported_source_headers",
        "//modules/control/safety:exported_source_headers",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "control/include",
    strip_prefix = strip_prefix.from_root("modules/control"),
)

pkg_filegroup(
    name = "package_control_hdrs",
    srcs = [":package_control_hdrs_data"],
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

# TODO(storypku): split test_data
filegroup(
    name = "test_data",
    srcs = glob([
        "testdata/**",
    ]),
)
