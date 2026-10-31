load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "package_files_binaries",
    srcs = [
        ":bin_map_generator",
        ":map_tool",
        ":map_xysl",
        ":quaternion_euler",
        ":refresh_default_end_way_point",
        ":sim_map_generator",
    ],
    attributes = pkg_attributes(mode = "0755"),
    include_runfiles = True,
    prefix = "modules/map/tools",
    strip_prefix = strip_prefix.from_root("modules/map/tools"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_binaries"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
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

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
