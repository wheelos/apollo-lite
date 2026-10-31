load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "plugin_manager",
    srcs = ["plugin_manager.cc"],
    hdrs = ["plugin_manager.h"],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/proto:dv_plugin_msg_cc_proto",
        "//modules/dreamview/proto:plugin_config_cc_proto",
        "@core//cyber",
    ],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_headers",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
