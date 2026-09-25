load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "localization_testdata",
    srcs = glob(["testdata/*"]),
)

install(
    name = "testdata",
    data = [
        ":localization_testdata",
    ],
    data_dest = "localization/addition_data",
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "conf/*.pb.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    data = [
        "localization.BUILD",
        ":runtime_data",
    ],
    data_dest = "localization",
    deps = [
        ":pb_hdrs",
        ":pb_localization",
        ":testdata",
        "//modules/localization/common:install",
        "//modules/localization/msf:install",
        "//modules/localization/msf/local_tool/data_extraction:install",
        "//modules/localization/msf/local_tool/local_visualization/offline_visual:install",
        "//modules/localization/msf/local_tool/local_visualization/online_visual:install",
        "//modules/localization/msf/local_tool/map_creation:install",
    ],
)

install(
    name = "pb_hdrs",
    data = [
        "//modules/localization/proto:gnss_pnt_result_cc_proto",
        "//modules/localization/proto:localization_config_cc_proto",
        "//modules/localization/proto:measure_cc_proto",
        "//modules/localization/proto:sins_pva_cc_proto",
    ],
    data_dest = "localization/include",
)

install_files(
    name = "pb_localization",
    dest = "localization",
    files = [
        "//modules/localization/proto:gnss_pnt_result_py_pb2",
        "//modules/localization/proto:localization_config_py_pb2",
        "//modules/localization/proto:measure_py_pb2",
        "//modules/localization/proto:sins_pva_py_pb2",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:gps_py_pb2",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:imu_py_pb2",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_py_pb2",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_status_py_pb2",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:pose_py_pb2",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_localization_hdrs",
        ":install_localization_src",
    ],
)

install_src_files(
    name = "install_localization_src",
    dest = "localization/src",
    filter = "*",
    src_dir = ["."],
)

install_src_files(
    name = "install_localization_hdrs",
    dest = "localization/include",
    filter = "*.h",
    src_dir = ["."],
)

cpplint()
