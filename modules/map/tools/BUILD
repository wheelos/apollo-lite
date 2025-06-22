load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "modules/map/tools",
    targets = [
        ":map_tool",
        ":map_xysl",
        ":refresh_default_end_way_point",
        ":sim_map_generator",
        ":proto_map_generator",
        ":bin_map_generator",
        ":quaternion_euler",
    ],
    deps = [
    ],
)

cc_binary(
    name = "map_tool",
    srcs = ["map_tool.cc"],
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "map_xysl",
    srcs = ["map_xysl.cc"],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/common/util:string_util",
        "//modules/map/hdmap:hdmap_util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "refresh_default_end_way_point",
    srcs = ["refresh_default_end_way_point.cc"],
    deps = [
        "//cyber",
        "//modules/common_msgs/routing_msgs:poi_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/map/hdmap:hdmap_util",
    ],
)

cc_binary(
    name = "sim_map_generator",
    srcs = ["sim_map_generator.cc"],
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "proto_map_generator",
    srcs = ["proto_map_generator.cc"],
    deps = [
        "//cyber",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "bin_map_generator",
    srcs = ["bin_map_generator.cc"],
    deps = [
        "//cyber",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "quaternion_euler",
    srcs = ["quaternion_euler.cc"],
    deps = [
        "//modules/common/math",
        "//modules/common/util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
