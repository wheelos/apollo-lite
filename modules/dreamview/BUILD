load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools/platform:build_defs.bzl", "copts_if_teleop")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

cc_binary(
    name = "dreamview",
    srcs = ["main.cc"],
    copts = DREAMVIEW_COPTS + copts_if_teleop(),
    data = [
        "@dreamview_frontend_assets//:all_files",
    ],
    deps = [
        "//modules/dreamview/backend",
    ],
)

filegroup(
    name = "dreamview_conf",
    srcs = glob([
        "conf/*.conf",
        "conf/*.txt",
        "conf/hmi_modes/**/*.txt",
        "conf/recording/**/*.yaml",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    data = [
        ":dreamview_conf",
        "@dreamview_frontend_assets//:all_files",
    ],
    data_dest = "dreamview",
    library_dest = "dreamview/lib",
    runtime_dest = "dreamview/bin",
    targets = ["dreamview"],
    deps = [
        ":pb_dreamview",
        ":pb_hdrs",
        "//modules/dreamview/backend:install",
        "//modules/dreamview/backend/common:install",
        "//modules/dreamview/backend/hmi:install",
        "//modules/dreamview/backend/map:install",
        "//modules/dreamview/backend/sim_control_manager:install",
        "//modules/dreamview/backend/sim_control_manager/common:install",
    ],
)

install(
    name = "pb_hdrs",
    data = [
        "//modules/dreamview/backend/sim_control_manager/proto:dynamic_model_conf_cc_proto",
        "//modules/dreamview/backend/sim_control_manager/proto:sim_control_internal_cc_proto",
        "//modules/dreamview/backend/teleop/proto:daemon_cmd_cc_proto",
        "//modules/dreamview/backend/teleop/proto:daemon_rpt_cc_proto",
        "//modules/dreamview/backend/teleop/proto:modem_info_cc_proto",
        "//modules/dreamview/proto:camera_update_cc_proto",
        "//modules/dreamview/proto:data_collection_table_cc_proto",
        "//modules/dreamview/proto:dv_plugin_msg_cc_proto",
        "//modules/dreamview/proto:hmi_config_cc_proto",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "//modules/dreamview/proto:plugin_config_cc_proto",
        "//modules/dreamview/proto:point_cloud_cc_proto",
        "//modules/dreamview/proto:preprocess_table_cc_proto",
        "//modules/dreamview/proto:record_cc_proto",
        "//modules/dreamview/proto:scenario_cc_proto",
        "//modules/dreamview/proto:simulation_world_cc_proto",
    ],
    data_dest = "dreamview/include",
)

install_files(
    name = "pb_dreamview",
    dest = "dreamview",
    files = [
        "//modules/dreamview/proto:preprocess_table_py_pb2",
        "@wheelos_msgs//wheelos_msgs/dreamview_msgs:chart_py_pb2",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_dreamview_hdrs",
        ":install_dreamview_src",
    ],
)

install_src_files(
    name = "install_dreamview_src",
    dest = "dreamview/src",
    filter = "*",
    src_dir = ["."],
)

install_src_files(
    name = "install_dreamview_hdrs",
    dest = "dreamview/include",
    filter = "*.h",
    src_dir = ["."],
)

cpplint()
