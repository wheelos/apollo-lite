load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "record_tool",
    srcs = ["record_tool.cc"],
    deps = [
        "//cyber",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/localization/msf/common/io:common_io",
        "@boost.filesystem",
        "@local_config_pcl//:common",
        "@local_config_pcl//:io",
    ],
)

cc_binary(
    name = "live_pointcloud_viewer",
    srcs = ["live_pointcloud_viewer.cc"],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/transform:buffer",
        "@civetweb//:civetserver",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_nlohmann_json//:json",
        "@eigen",
    ],
)

cpplint()
