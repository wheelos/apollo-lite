load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "record_tool",
    srcs = ["record_tool.cc"],
    deps = [
        "//modules/localization/msf/common/io:common_io",
        "@boost.filesystem",
        "@core//cyber",
        "@local_config_pcl//:common",
        "@local_config_pcl//:io",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_binary(
    name = "live_pointcloud_viewer",
    srcs = ["live_pointcloud_viewer.cc"],
    deps = [
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/transform:transform_query",
        "@civetweb//:civetserver",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_nlohmann_json//:json",
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cpplint()
