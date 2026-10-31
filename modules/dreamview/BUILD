load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")
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
    linkstatic = False,
    deps = [
        "//modules/dreamview/backend",
    ],
)

filegroup(
    name = "dreamview_conf",
    srcs = glob([
        "conf/*.conf",
        "conf/hmi_modes/**/*.txt",
        "conf/recording/**/*.yaml",
        "launch/*.launch",
    ]),
)

pkg_files(
    name = "package_files_data",
    srcs = [
        ":dreamview_conf",
        "@dreamview_frontend_assets//:all_files",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "dreamview",
    strip_prefix = strip_prefix.from_pkg(),
)

pkg_files(
    name = "package_files_binaries",
    srcs = ["dreamview"],
    attributes = pkg_attributes(mode = "0755"),
    include_runfiles = True,
    prefix = "dreamview/bin",
    strip_prefix = strip_prefix.from_root("modules/dreamview"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_binaries",
        ":package_files_data",
        ":pb_dreamview",
        ":pb_hdrs",
        "//modules/dreamview/backend:package_files",
        "//modules/dreamview/backend/common:package_files",
        "//modules/dreamview/backend/hmi:package_files",
        "//modules/dreamview/backend/map:package_files",
        "//modules/dreamview/backend/sim_control_manager:package_files",
        "//modules/dreamview/backend/sim_control_manager/common:package_files",
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
    attributes = pkg_attributes(mode = "0644"),
    prefix = "dreamview/include",
    strip_prefix = strip_prefix.from_root("modules/dreamview"),
)

pkg_filegroup(
    name = "pb_hdrs",
    srcs = [":pb_hdrs_data"],
)

pkg_files(
    name = "pb_dreamview_data",
    srcs = [
        "//modules/dreamview/proto:preprocess_table_py_pb2",
        "@wheelos_msgs//wheelos_msgs/dreamview_msgs:chart_py_pb2",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "dreamview",
    strip_prefix = strip_prefix.from_pkg(),
)

pkg_filegroup(
    name = "pb_dreamview",
    srcs = [":pb_dreamview_data"],
)

pkg_filegroup(
    name = "source_files",
    srcs = [
        ":package_dreamview_hdrs",
        ":package_dreamview_src",
    ],
)

pkg_install(
    name = "source_dir",
    srcs = [":source_files"],
    tags = ["manual"],
)

pkg_files(
    name = "package_dreamview_src_data",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ) + [
        "//modules/dreamview/backend:exported_source_files",
        "//modules/dreamview/backend/common:exported_source_files",
        "//modules/dreamview/backend/handlers:exported_source_files",
        "//modules/dreamview/backend/hmi:exported_source_files",
        "//modules/dreamview/backend/map:exported_source_files",
        "//modules/dreamview/backend/perception_camera_updater:exported_source_files",
        "//modules/dreamview/backend/plugins:exported_source_files",
        "//modules/dreamview/backend/point_cloud:exported_source_files",
        "//modules/dreamview/backend/sim_control_manager:exported_source_files",
        "//modules/dreamview/backend/sim_control_manager/common:exported_source_files",
        "//modules/dreamview/backend/sim_control_manager/core:exported_source_files",
        "//modules/dreamview/backend/sim_control_manager/dynamic_model/perfect_control:exported_source_files",
        "//modules/dreamview/backend/sim_control_manager/proto:exported_source_files",
        "//modules/dreamview/backend/simulation_world:exported_source_files",
        "//modules/dreamview/backend/teleop:exported_source_files",
        "//modules/dreamview/backend/teleop/proto:exported_source_files",
        "//modules/dreamview/backend/testdata:exported_source_files",
        "//modules/dreamview/proto:exported_source_files",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "dreamview/src",
    strip_prefix = strip_prefix.from_root("modules/dreamview"),
)

pkg_filegroup(
    name = "package_dreamview_src",
    srcs = [":package_dreamview_src_data"],
)

pkg_files(
    name = "package_dreamview_hdrs_data",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ) + [
        "//modules/dreamview/backend:exported_source_headers",
        "//modules/dreamview/backend/common:exported_source_headers",
        "//modules/dreamview/backend/handlers:exported_source_headers",
        "//modules/dreamview/backend/hmi:exported_source_headers",
        "//modules/dreamview/backend/map:exported_source_headers",
        "//modules/dreamview/backend/perception_camera_updater:exported_source_headers",
        "//modules/dreamview/backend/plugins:exported_source_headers",
        "//modules/dreamview/backend/point_cloud:exported_source_headers",
        "//modules/dreamview/backend/sim_control_manager:exported_source_headers",
        "//modules/dreamview/backend/sim_control_manager/common:exported_source_headers",
        "//modules/dreamview/backend/sim_control_manager/core:exported_source_headers",
        "//modules/dreamview/backend/sim_control_manager/dynamic_model/perfect_control:exported_source_headers",
        "//modules/dreamview/backend/simulation_world:exported_source_headers",
        "//modules/dreamview/backend/teleop:exported_source_headers",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "dreamview/include",
    strip_prefix = strip_prefix.from_root("modules/dreamview"),
)

pkg_filegroup(
    name = "package_dreamview_hdrs",
    srcs = [":package_dreamview_hdrs_data"],
)
