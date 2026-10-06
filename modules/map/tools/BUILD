load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "modules/map/tools",
    targets = [
        ":map_tool",
        ":map_xysl",
        ":map_selection_tool",
        ":refresh_default_end_way_point",
        ":sim_map_generator",
        ":bin_map_generator",
        ":quaternion_euler",
    ],
    deps = [
    ],
)

cc_binary(
    name = "map_selection_tool",
    srcs = ["map_selection_tool.cc"],
    deps = [
        "@wheelos_common//modules/common/map:map_selection",
    ],
)

cc_binary(
    name = "map_tool",
    srcs = ["map_tool.cc"],
    deps = [
        "@wheelos_common//modules/common/configs:config_gflags",
        "@wheelos_map//modules/map/hdmap:hdmap_util",
        "@com_github_gflags_gflags//:gflags",
        "@core//cyber/common:file",
        "@core//cyber/common:log",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_cc_proto",
    ],
)

cc_binary(
    name = "map_xysl",
    srcs = ["map_xysl.cc"],
    deps = [
        "@wheelos_common//modules/common/configs:config_gflags",
        "@wheelos_common//modules/common/util",
        "@wheelos_common//modules/common/util:string_util",
        "@wheelos_map//modules/map/hdmap:hdmap_util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "refresh_default_end_way_point",
    srcs = ["refresh_default_end_way_point.cc"],
    deps = [
        "@wheelos_map//modules/map/hdmap:hdmap_util",
        "@core//cyber/common:file",
        "@core//cyber/common:log",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:poi_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
)

alias(
    name = "bin_map_generator",
    actual = "@wheelos_map//modules/map/tools:bin_map_generator",
)

alias(
    name = "sim_map_generator",
    actual = "@wheelos_map//modules/map/tools:sim_map_generator",
)

cc_binary(
    name = "quaternion_euler",
    srcs = ["quaternion_euler.cc"],
    deps = [
        "@wheelos_common//modules/common/math",
        "@wheelos_common//modules/common/util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
