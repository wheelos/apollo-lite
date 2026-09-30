load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "vehicle_config_data",
    srcs = ["vehicle_param.pb.txt"],
)

filegroup(
    name = "global_flagfile",
    srcs = ["global_flagfile.txt"],
)

install(
    name = "install",
    data_dest = "common",
    library_dest = "common/lib",
    runtime_dest = "common/bin",
    deps = [
        ":install_runtime_data",
        ":pb_common",
        ":pb_hdrs",
    ],
    targets = [
        "@wheelos_common//modules/common/configs:config_gflags",
        "@wheelos_common//modules/common/configs:vehicle_config_helper",
        "@wheelos_common//modules/common/filters:filters",
        "@wheelos_common//modules/common/kv_db:kv_db",
        "@wheelos_common//modules/common/kv_db:kv_db_tool",
        "@wheelos_common//modules/common/latency_recorder:latency_recorder",
        "@wheelos_common//modules/common/monitor_log:monitor_log",
        "@wheelos_common//modules/common/status:status",
        "@wheelos_common//modules/common/vehicle_model:vehicle_model",
        "@wheelos_common//modules/common/vehicle_state:vehicle_reference",
        "@wheelos_common//modules/common/vehicle_state:vehicle_state_provider",
    ],
)

install(
    name = "install_runtime_data",
    data = [
        ":global_flagfile",
        ":vehicle_config_data",
    ],
    data_dest = "common/data",
)

install(
    name = "pb_hdrs",
    data = [
        "@wheelos_common//modules/common/adapters/proto:adapter_config_cc_proto",
        "@wheelos_common//modules/common/latency_recorder/proto:latency_record_cc_proto",
        "@wheelos_common//modules/common/vehicle_model/proto:vehicle_model_config_cc_proto",
        "@wheelos_common//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "@wheelos_msgs//wheelos_msgs/monitor_msgs:monitor_log_cc_proto",
    ],
    data_dest = "common/include",
)

install_files(
    name = "pb_common",
    dest = "common",
    files = [
        "@wheelos_msgs//wheelos_msgs/basic_msgs:direction_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:drive_event_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:drive_state_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:error_code_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:pnc_point_py_pb2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:vehicle_signal_py_pb2",
    ],
)
