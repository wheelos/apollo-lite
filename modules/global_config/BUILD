load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "vehicle_config_data",
    srcs = ["vehicle_param.pb.txt"],
)

filegroup(
    name = "global_flagfile",
    srcs = ["global_flagfile.txt"],
)

pkg_files(
    name = "package_files_archives",
    srcs = [
        "@wheelos_common//modules/common/configs:config_gflags",
        "@wheelos_common//modules/common/configs:vehicle_config_helper",
        "@wheelos_common//modules/common/filters",
        "@wheelos_common//modules/common/kv_db",
        "@wheelos_common//modules/common/latency_recorder",
        "@wheelos_common//modules/common/monitor_log",
        "@wheelos_common//modules/common/status",
        "@wheelos_common//modules/common/vehicle_model",
        "@wheelos_common//modules/common/vehicle_state:vehicle_reference",
        "@wheelos_common//modules/common/vehicle_state:vehicle_state_provider",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lib",
    strip_prefix = strip_prefix.from_pkg(),
)

pkg_files(
    name = "package_files_binaries",
    srcs = ["@wheelos_common//modules/common/kv_db:kv_db_tool"],
    attributes = pkg_attributes(mode = "0755"),
    include_runfiles = True,
    prefix = "common/bin",
    strip_prefix = strip_prefix.from_pkg(),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_archives",
        ":package_files_binaries",
        ":package_runtime_data",
        ":pb_common",
        ":pb_hdrs",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

pkg_files(
    name = "package_runtime_data_data",
    srcs = [
        ":global_flagfile",
        ":vehicle_config_data",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "common/data",
    strip_prefix = strip_prefix.from_root("modules/global_config"),
)

pkg_filegroup(
    name = "package_runtime_data",
    srcs = [":package_runtime_data_data"],
)

pkg_files(
    name = "pb_hdrs_data",
    srcs = [
        "@wheelos_common//modules/common/adapters/proto:adapter_config_cc_proto",
        "@wheelos_common//modules/common/latency_recorder/proto:latency_record_cc_proto",
        "@wheelos_common//modules/common/vehicle_model/proto:vehicle_model_config_cc_proto",
        "@wheelos_common//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "@wheelos_msgs//wheelos_msgs/monitor_msgs:monitor_log_cc_proto",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "common/include",
    strip_prefix = strip_prefix.from_pkg(),
)

pkg_filegroup(
    name = "pb_hdrs",
    srcs = [":pb_hdrs_data"],
)

pkg_files(
    name = "pb_common_data",
    srcs = [
        "@wheelos_msgs//wheelos_msgs/basic_msgs:direction_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:drive_event_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:drive_state_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:error_code_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:pnc_point_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:vehicle_signal_py_pb2",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "common",
    strip_prefix = strip_prefix.from_pkg(),
)

pkg_filegroup(
    name = "pb_common",
    srcs = [":pb_common_data"],
)
